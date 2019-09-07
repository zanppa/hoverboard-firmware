#include "stm32f1xx_hal.h"
#include <math.h>
#include "defines.h"
#include "setup.h"
#include "config.h"
#include "cfgbus.h"
#include "bldc.h"

//#include "control.h" // Debug
#include "math.h"
#include "svm.h"

// References for left and right motors
volatile svm_ref_t svm_left = {0, 0};
volatile svm_ref_t svm_right = {0, 0};

// Map sectors to correct timer capture/compare registers to generate the modulation pattern
// Array is [sector][vector] where vector states in which order the switches are turned.
// 000 -> Active 1 -> Active 2 -> 111 -> Active 2 -> Active 1 -> 000
// This works as long as left and right use same channel mapping (CCR1=U etc.)
// CCR is uint32_t, this should be used like (uint_32t *)(TIM8+mod_pattern[0][0]) or something
// Offsets should be CCR1=0x34, CCR2=0x38 and CCR3=0x3C so uint8_t is enough
static const uint8_t svm_mod_pattern[6][3] = {
  {offsetof(TIM_TypeDef, LEFT_TIM_V), offsetof(TIM_TypeDef, LEFT_TIM_U), offsetof(TIM_TypeDef, LEFT_TIM_W)},
  {offsetof(TIM_TypeDef, LEFT_TIM_V), offsetof(TIM_TypeDef, LEFT_TIM_W), offsetof(TIM_TypeDef, LEFT_TIM_U)},
  {offsetof(TIM_TypeDef, LEFT_TIM_W), offsetof(TIM_TypeDef, LEFT_TIM_V), offsetof(TIM_TypeDef, LEFT_TIM_U)},
  {offsetof(TIM_TypeDef, LEFT_TIM_W), offsetof(TIM_TypeDef, LEFT_TIM_U), offsetof(TIM_TypeDef, LEFT_TIM_V)},
  {offsetof(TIM_TypeDef, LEFT_TIM_U), offsetof(TIM_TypeDef, LEFT_TIM_W), offsetof(TIM_TypeDef, LEFT_TIM_V)},
  {offsetof(TIM_TypeDef, LEFT_TIM_U), offsetof(TIM_TypeDef, LEFT_TIM_V), offsetof(TIM_TypeDef, LEFT_TIM_W)}
};


// Calculate the timer values given the desired voltage vector
// length (modulation index) and angle in fixed point, return
// vector legths relative to the PWM period in t0, t1 and t2
// (t0 is half of the total zero vector length)
static void calculate_modulator(uint16_t midx, uint16_t angle, uint16_t *t0, uint16_t *t1, uint16_t *t2) {
  uint16_t ta1;
  uint16_t ta2;

  // Clamp modulation index to 1.0
  if(midx >= 4096) midx = 4096;

  // Clamp angle to 0...60 degrees
  // angle &= FIXED_MASK;	// 0...360 degrees
  angle = angle % FIXED_60DEG;

  // Calculate the vector times
  ta1 = fx_mulu(midx, array_sin(angle));
  ta1 = fx_mulu(ta1, PWM_PERIOD);

  ta2 = fx_mulu(midx,  array_sin(FIXED_60DEG - angle));
  ta2 = fx_mulu(ta2, PWM_PERIOD);

  (*t0) = (PWM_PERIOD - ta1 - ta2) / 2;
  (*t1) = ta1;
  (*t2) = ta2;
}


// Convert fixed point angle into sector number
static inline uint8_t angle_to_sector(uint16_t angle) {
  angle &= FIXED_MASK;
  if(angle < FIXED_60DEG) return 5;
  else if(angle < 2*FIXED_60DEG) return 0;
  else if(angle < 3*FIXED_60DEG) return 1;
  else if(angle < 4*FIXED_60DEG) return 2;
  else if(angle < 5*FIXED_60DEG) return 3;
  else return 4;
}

// Timer 1 update handles space vector modulation for both motors
void TIM1_UP_IRQHandler() {
  uint16_t t0, t1, t2;
  uint8_t sector;

  // Clear the update interrupt flag
  TIM1->SR = 0; //&= ~TIM_SR_UIF;

  // DEBUG: LED on
  HAL_GPIO_TogglePin(LED_PORT,LED_PIN);

#ifdef LEFT_MOTOR_SVM
  // Get the vector times from the modulator
  sector = angle_to_sector(svm_left.angle);
  calculate_modulator(svm_left.modulation_index, svm_left.angle, &t0, &t1, &t2);

  // TODO: Vectors are 111 -> Active 2 -> Active 1 -> 000 and back
  // Since the timer compare is wrong way
  *((uint32_t *)(LEFT_TIM_BASE + svm_mod_pattern[sector][0])) = t0;
  if(sector & 0x01) // Every odd sector uses "right" vector first
    *((uint32_t *)(LEFT_TIM_BASE + svm_mod_pattern[sector][1])) = t0 + t2;
  else // Even sectors uses "left" vector first
    *((uint32_t *)(LEFT_TIM_BASE + svm_mod_pattern[sector][1])) = t0 + t1;
  *((uint32_t *)(LEFT_TIM_BASE + svm_mod_pattern[sector][2])) = t0 + t1 + t2;
#endif

#ifdef RIGHT_MOTOR_SVM
  sector = angle_to_sector(svm_right.angle);
  calculate_modulator(svm_right.modulation_index, svm_right.angle, &t0, &t1, &t2);

  // TODO: Vectors are 111 -> Active 2 -> Active 1 -> 000 and back
  // Since the timer compare is wrong way
  *((uint32_t *)(RIGHT_TIM_BASE + svm_mod_pattern[sector][0])) = t0;
  if(sector & 0x01) // Every odd sector uses "right" vector first
    *((uint32_t *)(RIGHT_TIM_BASE + svm_mod_pattern[sector][1])) = t0 + t2;
  else // Even sectors uses "left" vector first
    *((uint32_t *)(RIGHT_TIM_BASE + svm_mod_pattern[sector][1])) = t0 + t1;
  *((uint32_t *)(RIGHT_TIM_BASE + svm_mod_pattern[sector][2])) = t0 + t1 + t2;
#endif

  // Debug: LED off
  HAL_GPIO_TogglePin(LED_PORT,LED_PIN);
}


// Timer 8 handler, not used at the moment
void TIM8_UP_IRQHandler() {
  // Do nothing, timer 1 handles the calculations
  // And both timers should run at the same rate

  // Clear the update interrupt flag
  TIM8->SR = 0; //&= ~TIM_SR_UIF;
}

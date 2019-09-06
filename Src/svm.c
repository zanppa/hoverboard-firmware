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

// Array to convert HALL sensor readings (order ABC) to sector number
// Note that index 0 and 7 are "guards" and should never happen when sensors work properly
static const uint8_t hall_to_sector[8] = { 2, 5, 1, 0, 3, 4, 2, 2 };

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


// Left motor timer update event handler
void TIM1_UP_IRQHandler() {
  uint16_t t0, t1, t2;
  uint16_t midx = 3000;	// DEBUG: Use fixed value
  uint16_t angle = 300; // DEBUG: Use fixed value

  // Debug: blink the led here
  HAL_GPIO_TogglePin(LED_PORT,LED_PIN);
  //led_update();


  // Read HALL sensors to detect rotor position
  //uint8_t sector_l = (LEFT_HALL_PORT->IDR >> LEFT_HALL_LSB_PIN) & 0b111;
  //sector_l = hall_to_sector[sector_l];

  //uint8_t sector_r = (RIGHT_HALL_PORT->IDR >> RIGHT_HALL_LSB_PIN) & 0b111;
  //sector_r = hall_to_sector[sector_r];

  uint8_t sector_l = angle_to_sector(angle);
  uint8_t sector_r = angle_to_sector(angle);

  // Get the vector times from the modulator
  calculate_modulator(midx, angle, &t0, &t1, &t2);

  // Set the timer values
  // Debug: just test something, do not connect motor!
  LEFT_TIM->LEFT_TIM_U = 200;
  LEFT_TIM->LEFT_TIM_V = 1000;
  LEFT_TIM->LEFT_TIM_W = 1500;

  // TODO: Enable this later
  //*((uint16_t *)(LEFT_TIM + svm_mod_pattern[sector_l][0])) = t0;
  //*((uint16_t *)(LEFT_TIM + svm_mod_pattern[sector_l][1])) = t1;
  //*((uint16_t *)(LEFT_TIM + svm_mod_pattern[sector_l][2])) = t2;

  // Update modbus variables (globals)
  cfg.vars.pos_l = sector_l;
  cfg.vars.pos_r = sector_r;
}

// Right motor timer update event handler
void TIM8_UP_IRQHandler() {
  // Do nothing, timer 1 handles the calculations
  // And both timers should run at the same rate
}

// Calculate the timer values given the desired voltage vector
// length (modulation index) and angle in fixed point, return
// vector change times relative to the PWM period in t0, t1 and t2
// (t0 is half of the total zero vector length)
void calculate_modulator(uint16_t midx, uint16_t angle, uint16_t *t0, uint16_t *t1, uint16_t *t2) {
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
  (*t1) = (*t0) + ta1;
  (*t2) = (*t1) + ta2;
}

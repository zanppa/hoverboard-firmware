#include "stm32f1xx_hal.h"
#include <math.h>
#include "defines.h"
#include "setup.h"
#include "config.h"
#include "cfgbus.h"
#include "bldc.h"

#include "control.h" // Debug
#include "math.h"

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

// Right motor timer update event handler
void TIM1_UP_IRQHandler() {
  // Read HALL sensors to detect rotor position
  uint8_t sector_l = (LEFT_HALL_PORT->IDR >> LEFT_HALL_LSB_PIN) & 0b111;
  sector_l = hall_to_sector[sector_l];

  uint8_t sector_r = (RIGHT_HALL_PORT->IDR >> RIGHT_HALL_LSB_PIN) & 0b111;
  sector_r = hall_to_sector[sector_r];

  // Debug: blink the led here
  led_update();

  // Update modbus variables (globals)
  cfg.vars.pos_l = sector_l;
  cfg.vars.pos_r = sector_r;
}

// Left motor timer update event handler
void TIM8_UP_IRQHandler() {
  // Do nothing, timer 1 handles the calculations
  // And both timers should run at the same rate
}

// Calculate the timer values given the desired voltage vector
// length and angle
void calculate_modulator(uint16_t midx, uint16_t angle) {
  uint16_t ta1;
  uint16_t ta2;
  uint16_t t0;

  // Clamp modulation index to 1.0
  if(midx > 4096) midx = 4096;

  // Calculate the vector times
  ta1 = fx_mulu(midx, array_sin(angle));
  ta1 = fx_mulu(ta1, PWM_PERIOD);

  ta2 = fx_mulu(midx,  array_sin(683 - angle));
  ta2 = fx_mulu(ta2, PWM_PERIOD);

  t0 = (PWM_PERIOD - ta1 - ta2) / 2;
}

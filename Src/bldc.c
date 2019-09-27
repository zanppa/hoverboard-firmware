
#include "stm32f1xx_hal.h"

#include <math.h>

#include "defines.h"
#include "config.h"
#include "bldc.h"

//uint8_t enable = 0;

extern volatile motor_state_t motor_state[2];

// Same method for bldc commutation, in this case [sector] contains first positive phase and then negative, last is zero
static const uint8_t bldc_mod_pattern[6][3] = {
  {offsetof(TIM_TypeDef, LEFT_TIM_U), offsetof(TIM_TypeDef, LEFT_TIM_W), offsetof(TIM_TypeDef, LEFT_TIM_V)},
  {offsetof(TIM_TypeDef, LEFT_TIM_V), offsetof(TIM_TypeDef, LEFT_TIM_W), offsetof(TIM_TypeDef, LEFT_TIM_U)},  // Was 0
  {offsetof(TIM_TypeDef, LEFT_TIM_V), offsetof(TIM_TypeDef, LEFT_TIM_U), offsetof(TIM_TypeDef, LEFT_TIM_W)},  // Was 1
  {offsetof(TIM_TypeDef, LEFT_TIM_W), offsetof(TIM_TypeDef, LEFT_TIM_U), offsetof(TIM_TypeDef, LEFT_TIM_V)},
  {offsetof(TIM_TypeDef, LEFT_TIM_W), offsetof(TIM_TypeDef, LEFT_TIM_V), offsetof(TIM_TypeDef, LEFT_TIM_U)},
  {offsetof(TIM_TypeDef, LEFT_TIM_U), offsetof(TIM_TypeDef, LEFT_TIM_V), offsetof(TIM_TypeDef, LEFT_TIM_W)}
};

// Timer 8 handler updates the BLDC PWM references
// This timer runs at twise the switching frequency
void TIM8_UP_IRQHandler() {
  uint8_t sector;
  int16_t ampl;

  // Clear the update interrupt flag
  TIM8->SR = 0; //&= ~TIM_SR_UIF;

#ifdef LEFT_MOTOR_BLDC
  sector = motor_state[STATE_LEFT].act.sector;
  ampl = motor_state[STATE_LEFT].ctrl.amplitude;
  ampl = CLAMP(ampl, BLDC_SHORT_PULSE - (PWM_PERIOD/2), (PWM_PERIOD/2) - BLDC_SHORT_PULSE);

  *((uint16_t *)(LEFT_TIM_BASE + bldc_mod_pattern[sector][0])) = (PWM_PERIOD/2) + ampl;
  *((uint16_t *)(LEFT_TIM_BASE + bldc_mod_pattern[sector][1])) = (PWM_PERIOD/2) - ampl;
  *((uint16_t *)(LEFT_TIM_BASE + bldc_mod_pattern[sector][2])) = PWM_PERIOD/2;
#endif

#ifdef RIGHT_MOTOR_BLDC
  sector = motor_state[STATE_RIGHT].act.sector;
  ampl = motor_state[STATE_RIGHT].ctrl.amplitude;
  ampl = CLAMP(ampl, BLDC_SHORT_PULSE - (PWM_PERIOD/2), (PWM_PERIOD/2) - BLDC_SHORT_PULSE);

  *((uint16_t *)(RIGHT_TIM_BASE + bldc_mod_pattern[sector][0])) = (PWM_PERIOD/2) + ampl;
  *((uint16_t *)(RIGHT_TIM_BASE + bldc_mod_pattern[sector][1])) = (PWM_PERIOD/2) - ampl;
  *((uint16_t *)(RIGHT_TIM_BASE + bldc_mod_pattern[sector][2])) = PWM_PERIOD/2;
#endif
}


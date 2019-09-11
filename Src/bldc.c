
#include "stm32f1xx_hal.h"

#include <math.h>

#include "defines.h"
#include "config.h"
#include "bldc.h"

//uint8_t enable = 0;

extern volatile motor_state_t motor_state[2];

// Same method for bldc commutation, in this case [sector] contains first positive phase and then negative, last is zero
static const uint8_t bldc_mod_pattern[6][3] = {
  {offsetof(TIM_TypeDef, LEFT_TIM_V), offsetof(TIM_TypeDef, LEFT_TIM_W), offsetof(TIM_TypeDef, LEFT_TIM_U)},
  {offsetof(TIM_TypeDef, LEFT_TIM_V), offsetof(TIM_TypeDef, LEFT_TIM_U), offsetof(TIM_TypeDef, LEFT_TIM_W)},
  {offsetof(TIM_TypeDef, LEFT_TIM_W), offsetof(TIM_TypeDef, LEFT_TIM_U), offsetof(TIM_TypeDef, LEFT_TIM_V)},
  {offsetof(TIM_TypeDef, LEFT_TIM_W), offsetof(TIM_TypeDef, LEFT_TIM_V), offsetof(TIM_TypeDef, LEFT_TIM_U)},
  {offsetof(TIM_TypeDef, LEFT_TIM_U), offsetof(TIM_TypeDef, LEFT_TIM_V), offsetof(TIM_TypeDef, LEFT_TIM_W)},
  {offsetof(TIM_TypeDef, LEFT_TIM_U), offsetof(TIM_TypeDef, LEFT_TIM_W), offsetof(TIM_TypeDef, LEFT_TIM_V)}
};


inline void blockPWM(int16_t pwm, uint8_t pos, int16_t *u, int16_t *v, int16_t *w) {
  switch(pos) {
    case 0:
      *u = 0;
      *v = pwm;
      *w = -pwm;
      break;
    case 1:
      *u = -pwm;
      *v = pwm;
      *w = 0;
      break;
    case 2:
      *u = -pwm;
      *v = 0;
      *w = pwm;
      break;
    case 3:
      *u = 0;
      *v = -pwm;
      *w = pwm;
      break;
    case 4:
      *u = pwm;
      *v = -pwm;
      *w = 0;
      break;
    case 5:
      *u = pwm;
      *v = 0;
      *w = -pwm;
      break;
    default:
      *u = 0;
      *v = 0;
      *w = 0;
  }
}

// Timer 8 handler updates the BLDC PWM references
// This timer runs at twise the switching frequency
void TIM8_UP_IRQHandler() {
  int16_t u, v, w;

  // Clear the update interrupt flag
  TIM8->SR = 0; //&= ~TIM_SR_UIF;

#ifdef LEFT_MOTOR_BLDC
  blockPWM(motor_state[STATE_LEFT].ctrl.amplitude, motor_state[STATE_LEFT].act.sector, &u, &v, &w);
  LEFT_TIM->LEFT_TIM_U = CLAMP(u + PWM_PERIOD / 2, BLDC_SHORT_PULSE, PWM_PERIOD-BLDC_SHORT_PULSE);
  LEFT_TIM->LEFT_TIM_V = CLAMP(v + PWM_PERIOD / 2, BLDC_SHORT_PULSE, PWM_PERIOD-BLDC_SHORT_PULSE);
  LEFT_TIM->LEFT_TIM_W = CLAMP(w + PWM_PERIOD / 2, BLDC_SHORT_PULSE, PWM_PERIOD-BLDC_SHORT_PULSE);
#endif

#ifdef RIGHT_MOTOR_BLDC
  blockPWM(motor_state[STATE_RIGHT].ctrl.amplitude, motor_state[STATE_RIGHT].act.sector, &u, &v, &w);
  RIGHT_TIM->RIGHT_TIM_U = CLAMP(u + PWM_PERIOD / 2, BLDC_SHORT_PULSE, PWM_PERIOD-BLDC_SHORT_PULSE);
  RIGHT_TIM->RIGHT_TIM_V = CLAMP(v + PWM_PERIOD / 2, BLDC_SHORT_PULSE, PWM_PERIOD-BLDC_SHORT_PULSE);
  RIGHT_TIM->RIGHT_TIM_W = CLAMP(w + PWM_PERIOD / 2, BLDC_SHORT_PULSE, PWM_PERIOD-BLDC_SHORT_PULSE);
#endif
}


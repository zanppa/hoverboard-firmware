
#include "stm32f1xx_hal.h"

#include <math.h>

#include "defines.h"
#include "config.h"
#include "bldc.h"

//uint8_t enable = 0;

extern volatile motor_state_t motor_state[2];

static uint32_t offsetcount = 0;
static adc_offsets_t offsets = {0};
extern volatile adc_buf_t adc_buffer;

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
      *v = pwm;	// DEBUG, was 0
      *w = -pwm;	// DEBUG, was 0
  }
}


//scan 8 channels with 2ADCs @ 20 clk cycles per sample
//meaning ~80 ADC clock cycles @ 8MHz until new DMA interrupt =~ 100KHz
//=640 cpu cycles
void DMA1_Channel1_IRQHandler() {
  DMA1->IFCR = DMA_IFCR_CTCIF1;

  // callibrate ADC offset before startup by averaging 1024 samples.
  if(offsetcount < 1024) {
    offsetcount++;
    offsets.rl1 += adc_buffer.rl1;
    offsets.rl2 += adc_buffer.rl2;
    offsets.rr1 += adc_buffer.rr1;
    offsets.rr2 += adc_buffer.rr2;
    offsets.dcl += adc_buffer.dcl;
    offsets.dcr += adc_buffer.dcr;
    offsets.temp += adc_buffer.temp;
    offsets.vbat += adc_buffer.vbat;
    return;
  } else if (offsetcount == 1024) {
	offsetcount++;
    offsets.rl1 /= 1024;
    offsets.rl2 /= 1024;
    offsets.rr1 /= 1024;
    offsets.rr2 /= 1024;
    offsets.dcl /= 1024;
    offsets.dcr /= 1024;
    offsets.temp /= 1024;
    offsets.vbat /= 1024;
  }

  // Disable PWM when current limit is reached (current chopping)
  if(ABS(adc_buffer.dcl - offsets.dcl) > DC_CUR_THRESHOLD || motor_state[STATE_LEFT].ctrl.enable == 0) {
    LEFT_TIM->BDTR &= ~TIM_BDTR_MOE;
  } else {
    LEFT_TIM->BDTR |= TIM_BDTR_MOE;
  }

  if(ABS(adc_buffer.dcr - offsets.dcr) > DC_CUR_THRESHOLD || motor_state[STATE_RIGHT].ctrl.enable == 0) {
    RIGHT_TIM->BDTR &= ~TIM_BDTR_MOE;
  } else {
    RIGHT_TIM->BDTR |= TIM_BDTR_MOE;
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


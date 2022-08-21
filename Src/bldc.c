// BLDC modulator
/*
Copyright (C) 2019 Lauri Peltonen

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#include "stm32f1xx_hal.h"

#include <math.h>

#include "defines.h"
#include "config.h"
#include "bldc.h"
#include "control.h"

static const int16_t bldc_min_pulse = BLDC_SHORT_PULSE - (PWM_PERIOD/2);
static const int16_t bldc_max_pulse = (PWM_PERIOD/2) - BLDC_SHORT_PULSE;


// RDSon measurement trigger
extern ADC_HandleTypeDef adc_rdson;

extern volatile motor_state_t motor_state[2];

// Same method for bldc commutation, in this case [sector] contains first positive phase and then negative, last is zero
#if defined(LEFT_MOTOR_BLDC) || defined(RIGHT_MOTOR_BLDC)
static const uint8_t bldc_mod_pattern[6][3] = {
  {offsetof(TIM_TypeDef, LEFT_TIM_V), offsetof(TIM_TypeDef, LEFT_TIM_W), offsetof(TIM_TypeDef, LEFT_TIM_U)},  // Was 0
  {offsetof(TIM_TypeDef, LEFT_TIM_V), offsetof(TIM_TypeDef, LEFT_TIM_U), offsetof(TIM_TypeDef, LEFT_TIM_W)},  // Was 1
  {offsetof(TIM_TypeDef, LEFT_TIM_W), offsetof(TIM_TypeDef, LEFT_TIM_U), offsetof(TIM_TypeDef, LEFT_TIM_V)},
  {offsetof(TIM_TypeDef, LEFT_TIM_W), offsetof(TIM_TypeDef, LEFT_TIM_V), offsetof(TIM_TypeDef, LEFT_TIM_U)},
  {offsetof(TIM_TypeDef, LEFT_TIM_U), offsetof(TIM_TypeDef, LEFT_TIM_V), offsetof(TIM_TypeDef, LEFT_TIM_W)},
  {offsetof(TIM_TypeDef, LEFT_TIM_U), offsetof(TIM_TypeDef, LEFT_TIM_W), offsetof(TIM_TypeDef, LEFT_TIM_V)}
};
#endif

// Timer 8 handler updates the BLDC PWM references
// This timer runs at twice the switching frequency
void TIM8_UP_IRQHandler() {
#if defined(LEFT_MOTOR_BLDC) || defined(RIGHT_MOTOR_BLDC)
  uint8_t sector;
  int16_t ampl_pos, ampl_neg;
  int16_t ampl_zero = 0;
#if defined(BLDC_FIELD_WEAKENING)
  uint16_t weak;
#endif
#endif

  // Clear the update interrupt flag
  TIM8->SR = 0; //&= ~TIM_SR_UIF;

  motor_state[STATE_LEFT].act.sector = read_left_hall();;
  motor_state[STATE_RIGHT].act.sector = read_right_hall();;

#ifdef LEFT_MOTOR_BLDC
  sector = motor_state[STATE_LEFT].act.sector;
  ampl_pos = fx_mul(motor_state[STATE_LEFT].ctrl.amplitude, PWM_PERIOD);
  ampl_neg = ampl_pos;

#if defined(BLDC_FIELD_WEAKENING)
  // Field weakening
  weak = motor_state[STATE_LEFT].ctrl.angle * 2; // Multiply by 2 since we use that in calculations
  // TODO: Clamp weak between 0 and FIXED_ONE?
  if(weak != 0) {
    if((sector & 1) && ampl_pos > 0) {
      // Odd sectors in positive direction or
      // even sectors in negative direction,
      // "negative" vector amplitude does not change
      ampl_pos = fx_lerp0(ampl_neg, weak);	// Positive goes to zero between 0 ... 0.5
      ampl_zero = fx_lerp1(ampl_neg, weak - FIXED_ONE);	// "Zero" goes to full between 0.5 ... 1
    } else {
      // Other way around, "positive" vector does not change
      ampl_neg = fx_lerp0(ampl_neg, weak);	// Positive goes to zero between 0 ... 0.5
      ampl_zero = -fx_lerp1(ampl_neg, weak - FIXED_ONE);	// "Zero" goes to negative full between 0.5 ... 1
    }
  }
#endif // BLDC_FIELD_WEAKENING

  // Make sure minimum pulse limitations still apply
  ampl_pos = CLAMP(ampl_pos, bldc_min_pulse, bldc_max_pulse);
  ampl_neg = CLAMP(ampl_neg, bldc_min_pulse, bldc_max_pulse);
  ampl_zero = CLAMP(ampl_zero, bldc_min_pulse, bldc_max_pulse);

  *((uint16_t *)(LEFT_TIM_BASE + bldc_mod_pattern[sector][0])) = (PWM_PERIOD/2) - ampl_pos;
  *((uint16_t *)(LEFT_TIM_BASE + bldc_mod_pattern[sector][1])) = (PWM_PERIOD/2) + ampl_neg;
  *((uint16_t *)(LEFT_TIM_BASE + bldc_mod_pattern[sector][2])) = PWM_PERIOD/2 - ampl_zero;
#endif


#ifdef RIGHT_MOTOR_BLDC
  sector = motor_state[STATE_RIGHT].act.sector;
  ampl_pos = fx_mul(motor_state[STATE_RIGHT].ctrl.amplitude, PWM_PERIOD);
  ampl_neg = ampl_pos;
  ampl_zero = 0;

#if defined(BLDC_FIELD_WEAKENING)
  // Field weakening
  weak = motor_state[STATE_RIGHT].ctrl.angle * 2; // Multiply by 2 since we use that in calculations
  // TODO: Clamp weak between 0 and FIXED_ONE?
  if(weak != 0) {
    if((sector & 1) && ampl_pos > 0) {
      // Odd sectors in positive direction or
      // even sectors in negative direction,
      // "negative" vector amplitude does not change
      ampl_pos = fx_lerp0(ampl_neg, weak);	// Positive goes to zero between 0 ... 0.5
      ampl_zero = fx_lerp1(ampl_neg, weak - FIXED_ONE);	// "Zero" goes to full between 0.5 ... 1
    } else {
      // Other way around, "positive" vector does not change
      ampl_neg = fx_lerp0(ampl_neg, weak);	// Positive goes to zero between 0 ... 0.5
      ampl_zero = -fx_lerp1(ampl_neg, weak - FIXED_ONE);	// "Zero" goes to negative full between 0.5 ... 1
    }
  }
#endif

  // Make sure minimum pulse limitations still apply
  ampl_pos = CLAMP(ampl_pos, bldc_min_pulse, bldc_max_pulse);
  ampl_neg = CLAMP(ampl_neg, bldc_min_pulse, bldc_max_pulse);
  ampl_zero = CLAMP(ampl_zero, bldc_min_pulse, bldc_max_pulse);

  *((uint16_t *)(RIGHT_TIM_BASE + bldc_mod_pattern[sector][0])) = (PWM_PERIOD/2) - ampl_pos;
  *((uint16_t *)(RIGHT_TIM_BASE + bldc_mod_pattern[sector][1])) = (PWM_PERIOD/2) + ampl_neg;
  *((uint16_t *)(RIGHT_TIM_BASE + bldc_mod_pattern[sector][2])) = PWM_PERIOD/2 - ampl_zero;
#endif
}


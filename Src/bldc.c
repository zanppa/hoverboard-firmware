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

// RDSon measurement trigger
extern ADC_HandleTypeDef adc_rdson;

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

#if defined(DATALOGGER_ENABLE)
extern DATALOGGER_TYPE datalogger[DATALOGGER_MAX+1];
extern DATALOGGER_COUNT_TYPE datalogger_write_offset;
#endif

// Timer 8 handler updates the BLDC PWM references
// This timer runs at twise the switching frequency
void TIM8_UP_IRQHandler() {
  uint8_t sector;
  int16_t ampl;

  // Clear the update interrupt flag
  TIM8->SR = 0; //&= ~TIM_SR_UIF;

// If only BLDC modulation is used, trigger rdson measurement here
#if defined(I_MEAS_RDSON) && defined(LEFT_MOTOR_BLDC) && defined(RIGHT_MOTOR_BLDC)
  if(!(TIM8->CR1 & TIM_CR1_DIR)) {
    // Trigger ADC rdson measurement when we have 000 zero vector
    // TODO: Use automatic trigger from timer?
    adc_rdson.Instance->CR2 |= ADC_CR2_SWSTART;
  }
#endif

// If BLDC is used and datalogger is enabled, do the capture here, on 111 zero vector
#if defined(DATALOGGER_ENABLE) && (defined(LEFT_MOTOR_BLDC) || defined(RIGHT_MOTOR_BLDC))
  if((TIM8->CR1 & TIM_CR1_DIR)) {
    // Falling counter --> current measurement is valid etc
    if(datalogger_trigger) {
      // Store values
      if(datalogger_var0) datalogger[datalogger_write_offset][0] = (DATALOGGER_TYPE)*datalogger_var0;
      if(datalogger_var1) datalogger[datalogger_write_offset][1] = (DATALOGGER_TYPE)*datalogger_var1;
      if(datalogger_var2) datalogger[datalogger_write_offset][2] = (DATALOGGER_TYPE)*datalogger_var2;
      if(datalogger_var3) datalogger[datalogger_write_offset][3] = (DATALOGGER_TYPE)*datalogger_var3;

      datalogger_write_offset = (datalogger_write_offset + 1) && DATALOGGER_MAX;
      if(!datalogger_write_offset)
        datalogger_trigger = 0;		// Disable writing when buffer is fulle
    }
  }
#endif

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


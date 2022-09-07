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

static int16_t left_period_tick = 0;
static int16_t right_period_tick = 0;

static uint8_t left_expected_sector = 0;
static uint8_t right_expected_sector = 0;


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
  uint8_t sector, prev_sector;
  uint16_t angle;
#if defined(LEFT_MOTOR_BLDC) || defined(RIGHT_MOTOR_BLDC)
  int16_t ampl_pos, ampl_neg;
  int16_t ampl_zero = 0;
#if defined(BLDC_FIELD_WEAKENING)
  uint16_t weak;
#endif
#endif

#if defined(LEFT_MOTOR_SVM) || defined(RIGHT_MOTOR_SVM)
  uint8_t mode;
#endif

  // Clear the update interrupt flag
  TIM8->SR = 0; //&= ~TIM_SR_UIF;

  // DEBUG: LED on
  //HAL_GPIO_TogglePin(LED_PORT,LED_PIN);

  // Motor position and speed detection, first left motor
  sector = read_left_hall();
  prev_sector = motor_state[STATE_LEFT].act.sector;
  motor_state[STATE_LEFT].act.sector = sector;
  angle = motor_state[STATE_LEFT].act.angle;

#if defined(LEFT_MOTOR_SVM)
  mode = motor_state[STATE_LEFT].ref.control_mode;
#endif

  if(sector != prev_sector) {  // Sector changed, calculate new speed
    // Update rotor position
    angle = sector * ANGLE_60DEG - ANGLE_30DEG;    // New angle at the edge of a sector

#if defined(LEFT_MOTOR_FOC) || defined(LEFT_MOTOR_SVM)
    // Calculate min and max angles in this sector
    // To prevent the modulator angle estimation from exceeding the sector
    motor_state[STATE_LEFT].ctrl.angle_min = angle;
    motor_state[STATE_LEFT].ctrl.angle_max = angle + ANGLE_60DEG;
#endif

    // Stall detection, if did not go to expected sector then rotation direction abruptly changed --> assume stall
    if(sector != left_expected_sector)
      left_period_tick = PERIOD_STOP; // Set speed to zero to prevent rapid oscillation in speed


    // Calculate what sector should be if going in positive direction
    if(prev_sector == 5) prev_sector = 0;
    else prev_sector++;

    if(sector != prev_sector) {
      left_period_tick = -left_period_tick; // Not the expected sector --> going to negative direction
      angle += ANGLE_60DEG;		// Other end of the sector for rotor angle
    }

    // Update expected sector according to rotation direction
    if(left_period_tick < 0) {
      if(sector == 0) left_expected_sector = 5;
      else left_expected_sector = sector - 1;
    } else {
      if(sector == 5) left_expected_sector = 0;
      else left_expected_sector = sector + 1;
    }


    motor_state[STATE_LEFT].act.period = left_period_tick;
    left_period_tick = 0;

#if defined(LEFT_MOTOR_FOC) || defined(LEFT_MOTOR_SVM)
    if(mode != CONTROL_UF && mode != CONTROL_ANGLE) {
      // Actual speed should be reflected in control speed in FOC or normal SVM modes
      if(left_period_tick == PERIOD_STOP || left_period_tick == -PERIOD_STOP)
        motor_state[STATE_LEFT].ctrl.speed = 0; // Rotation speed to zero
      else
        motor_state[STATE_LEFT].ctrl.speed = ANGLE_60DEG / left_period_tick; // Rotation speed / modulator interrupt
    }
#endif


  } else {
    if(left_period_tick < PERIOD_STOP) {
      left_period_tick++;
    } else {
      // Assume stall if rotating too slowly
      motor_state[STATE_LEFT].act.period = PERIOD_STOP;

#if defined(LEFT_MOTOR_FOC) || defined(LEFT_MOTOR_SVM)
    if(mode != CONTROL_UF && mode != CONTROL_ANGLE) {
      motor_state[STATE_LEFT].ctrl.speed = 0;
    }
#endif

    }
  }

#if defined(LEFT_MOTOR_FOC) || defined(LEFT_MOTOR_SVM)
  if(mode == CONTROL_UF || mode == CONTROL_ANGLE) {
    // Update the actual rotor position and SVM speed
    motor_state[STATE_LEFT].act.angle += motor_state[STATE_LEFT].ctrl.speed;
  } else {
    // Update the actual rotor position and SVM speed
    motor_state[STATE_LEFT].act.angle = angle + motor_state[STATE_LEFT].ctrl.speed;
  }
#endif


  // Then right motor position and speed
  sector = read_right_hall();
  prev_sector = motor_state[STATE_RIGHT].act.sector;
  motor_state[STATE_RIGHT].act.sector = sector;
  angle = motor_state[STATE_RIGHT].act.angle;

#if defined(RIGHT_MOTOR_SVM)
  mode = motor_state[STATE_RIGHT].ref.control_mode;
#endif

  if(sector != prev_sector) {  // Sector changed, calculate new speed
    // Update rotor position
    angle = sector * ANGLE_60DEG - ANGLE_30DEG;    // New angle at the edge of a sector

#if defined(RIGHT_MOTOR_FOC) || defined(RIGHT_MOTOR_SVM)
    // Calculate min and max angles in this sector
    // To prevent the modulator angle estimation from exceeding the sector
    motor_state[STATE_RIGHT].ctrl.angle_min = angle;
    motor_state[STATE_RIGHT].ctrl.angle_max = angle + ANGLE_60DEG;
#endif

    // Stall detection, if did not go to expected sector then rotation direction abruptly changed --> assume stall
    if(sector != right_expected_sector)
      right_period_tick = PERIOD_STOP; // Set speed to zero to prevent rapid oscillation in speed

    // Calculate expected sector if going in positive direction
    if(prev_sector == 5) prev_sector = 0;
    else prev_sector++;

    if(sector != prev_sector) {
      right_period_tick = -right_period_tick; // Not the expected sector --> going to negative direction
      angle += ANGLE_60DEG;		// Other end of the sector for rotor angle
    }

    // Update expected sector according to rotation direction
    if(right_period_tick < 0) {
      if(sector == 0) right_expected_sector = 5;
      else right_expected_sector = sector - 1;
    } else {
      if(sector == 5) right_expected_sector = 0;
      else right_expected_sector = sector + 1;
    }

    motor_state[STATE_RIGHT].act.period = right_period_tick;
    right_period_tick = 0;

#if defined(RIGHT_MOTOR_FOC) || defined(RIGHT_MOTOR_SVM)
    if(mode != CONTROL_UF && mode != CONTROL_ANGLE) {
      // Actual speed should be reflected in control speed in FOC or normal SVM modes
      if(right_period_tick == PERIOD_STOP || right_period_tick == -PERIOD_STOP)
        motor_state[STATE_RIGHT].ctrl.speed = 0; // Rotation speed to zero
      else
        motor_state[STATE_RIGHT].ctrl.speed = ANGLE_60DEG / right_period_tick; // Rotation speed / modulator interrupt
    }
#endif

  } else {
    if(right_period_tick < PERIOD_STOP) {
      right_period_tick++;
    } else {
      // Assume stall if rotating too slowly
      motor_state[STATE_RIGHT].act.period = PERIOD_STOP;

#if defined(RIGHT_MOTOR_FOC) || defined(RIGHT_MOTOR_SVM)
    if(mode != CONTROL_UF && mode != CONTROL_ANGLE) {
      motor_state[STATE_RIGHT].ctrl.speed = 0;
    }
#endif

    }
  }

#if defined(RIGHT_MOTOR_FOC) || defined(RIGHT_MOTOR_SVM)
  if(mode == CONTROL_UF || mode == CONTROL_ANGLE) {
    // Update the actual rotor position and SVM speed
    motor_state[STATE_RIGHT].act.angle += motor_state[STATE_RIGHT].ctrl.speed;
  } else {
    // Update the actual rotor position and SVM speed
    motor_state[STATE_RIGHT].act.angle = angle + motor_state[STATE_RIGHT].ctrl.speed;
  }
#endif



  // Left motor modulation, if enabled
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


  // Right motor modulation, if enabled
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

  // DEBUG: LED off
  HAL_GPIO_TogglePin(LED_PORT,LED_PIN);

}


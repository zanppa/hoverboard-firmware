// Space vector modulator
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

#include "math.h"
#include "svm.h"

extern volatile motor_state_t motor_state[2];

// RDSon measurement trigger
extern ADC_HandleTypeDef adc_rdson;

// Map sectors to correct timer capture/compare registers to generate the modulation pattern
// Array is [sector][vector] where vector states in which order the switches are turned.
// 000 -> Active 1 -> Active 2 -> 111 -> Active 2 -> Active 1 -> 000
// This works as long as left and right use same channel mapping (CCR1=U etc.)
// CCR is uint32_t, this should be used like (uint_32t *)(TIM8+mod_pattern[0][0]) or something
// Offsets should be CCR1=0x34, CCR2=0x38 and CCR3=0x3C so uint8_t is enough
#if defined(LEFT_MOTOR_SVM) || defined(RIGHT_MOTOR_SVM)
static const uint8_t svm_mod_pattern[6][3] = {
  {offsetof(TIM_TypeDef, LEFT_TIM_U), offsetof(TIM_TypeDef, LEFT_TIM_V), offsetof(TIM_TypeDef, LEFT_TIM_W)},
  {offsetof(TIM_TypeDef, LEFT_TIM_V), offsetof(TIM_TypeDef, LEFT_TIM_U), offsetof(TIM_TypeDef, LEFT_TIM_W)},	// Was 0
  {offsetof(TIM_TypeDef, LEFT_TIM_V), offsetof(TIM_TypeDef, LEFT_TIM_W), offsetof(TIM_TypeDef, LEFT_TIM_U)},	// Was 1
  {offsetof(TIM_TypeDef, LEFT_TIM_W), offsetof(TIM_TypeDef, LEFT_TIM_V), offsetof(TIM_TypeDef, LEFT_TIM_U)},
  {offsetof(TIM_TypeDef, LEFT_TIM_W), offsetof(TIM_TypeDef, LEFT_TIM_U), offsetof(TIM_TypeDef, LEFT_TIM_V)},
  {offsetof(TIM_TypeDef, LEFT_TIM_U), offsetof(TIM_TypeDef, LEFT_TIM_W), offsetof(TIM_TypeDef, LEFT_TIM_V)}
};

// Calculate the timer values given the desired voltage vector
// length (modulation index) and angle in fixed point, return
// vector legths relative to the PWM period in t0, t1 and t2
// (t0 is half of the total zero vector length)
static void calculate_modulator(int16_t midx, uint16_t angle, uint16_t *t0, uint16_t *t1, uint16_t *t2) {
  uint16_t ta1;
  uint16_t ta2;
  uint16_t tz;
  //uint16_t terr = 0;

  // Clamp < 0 modulation index to zero
  if(midx <= 0) midx = 0;
  // Clamp modulation index to 1.0
  else if(midx >= 4096) midx = 4096;

  // Clamp angle to 0...60 degrees
  // angle &= FIXED_MASK;	// 0...360 degrees
  angle = angle % ANGLE_60DEG;

  // Calculate the vector times
  ta1 = fx_mulu(midx,  array_sin(ANGLE_60DEG - angle));
  ta1 = fx_mulu(ta1, PWM_PERIOD);

  ta2 = fx_mulu(midx, array_sin(angle));
  ta2 = fx_mulu(ta2, PWM_PERIOD);

  tz = (PWM_PERIOD - ta1 - ta2) / 2;

  // Minimum pulse limitations
/*  if(tz < SVM_SHORT_ZPULSE) {
    terr = SVM_SHORT_ZPULSE - tz;
    tz = SVM_SHORT_ZPULSE;
  }

  ta1 = CLAMP(ta1 - (terr>>1), SVM_SHORT_PULSE, SVM_LONG_PULSE);
  ta2 = CLAMP(ta2 - (terr>>1), SVM_SHORT_PULSE, SVM_LONG_PULSE);
*/

  // Dead time compensation
  // Downcounting -> Update values for next upcounting part (000 -> 111)
/*
  if(TIM1->CR1 & TIM_CR1_DIR) {
    if(tz > SVM_DEAD_TIME_COMP)
      tz -= SVM_DEAD_TIME_COMP;

    if(ta1 > SVM_DEAD_TIME_COMP)
      ta1 -= SVM_DEAD_TIME_COMP;
  } else {
    // Upcounting -> update for downcounting part (111 -> 000)
    if(tz > SVM_DEAD_TIME_COMP)
      tz -= SVM_DEAD_TIME_COMP;
  }
*/

  *t0 = tz;
  *t1 = ta1;
  *t2 = ta2;
}
#endif


// Convert fixed point angle into sector number
static inline uint8_t angle_to_svm_sector(uint16_t angle) {
  //angle &= FIXED_MASK;	// Changed to full circle = 16 bits
  if(angle < ANGLE_60DEG) return 0;
  else if(angle < ANGLE_120DEG) return 1;
  else if(angle < ANGLE_180DEG) return 2;
  else if(angle < ANGLE_240DEG) return 3;
  else if(angle < ANGLE_300DEG) return 4;
  else return 5;
}

// Timer 1 update handles space vector modulation for both motors
void TIM1_UP_IRQHandler() {
#if defined(LEFT_MOTOR_SVM) || defined(RIGHT_MOTOR_SVM)
  uint16_t t0, t1, t2;
  uint16_t angle;
  uint8_t sector;
  uint16_t angle_min, angle_max;
#endif

  // Clear the update interrupt flag
  TIM1->SR = 0; //&= ~TIM_SR_UIF;

  // DEBUG: LED on
  //HAL_GPIO_TogglePin(LED_PORT,LED_PIN);

// If either motor uses SVM, trigger RDSon measurement here
#if defined(I_MEAS_RDSON) && (defined(LEFT_MOTOR_SVM) || defined(RIGHT_MOTOR_SVM))
  if(!(TIM1->CR1 & TIM_CR1_DIR)) {
    // Trigger ADC rdson measurement when we have 000 zero vector
    // TODO: Use automatic trigger from timer?
    adc_rdson.Instance->CR2 |= ADC_CR2_SWSTART;
  }
#endif

#ifdef LEFT_MOTOR_SVM
  // Interpolate the rotor position
  angle = motor_state[STATE_LEFT].act.angle + motor_state[STATE_LEFT].ctrl.speed;

  angle_min = motor_state[STATE_LEFT].ctrl.angle_min;
  angle_max = motor_state[STATE_LEFT].ctrl.angle_max;

#ifdef LEFT_MOTOR_FOC
  // Check that the new angle is inside the sector limits
  if(angle_min < angle_max) {
    // Normal situation
    angle = CLAMP(angle, angle_min, angle_max);
  } else {
    // Sector 0, angle 0 is inside the sector
      if(angle > angle_max && angle <= ANGLE_180DEG) angle = angle_max;
      else if(angle < angle_min && angle >= ANGLE_180DEG) angle = angle_min;
  }
#endif

  motor_state[STATE_LEFT].act.angle = angle;


  // Get the vector times from the modulator
#ifdef LEFT_MOTOR_FOC
  // Phase advance according to ctrl angle
  angle += motor_state[STATE_LEFT].ctrl.angle;
#else
  // In normal SVM control angle is directly the modulation angle
  angle = motor_state[STATE_LEFT].ctrl.angle;
#endif
  sector = angle_to_svm_sector(angle);
  calculate_modulator(motor_state[STATE_LEFT].ctrl.amplitude, angle, &t0, &t1, &t2);

  // Since the timer compare is wrong way
  *((uint16_t *)(LEFT_TIM_BASE + svm_mod_pattern[sector][0])) = t0;
  if(sector & 0x01) // Every odd sector uses "right" vector first
    *((uint16_t *)(LEFT_TIM_BASE + svm_mod_pattern[sector][1])) = t0 + t2;
  else // Even sectors uses "left" vector first
    *((uint16_t *)(LEFT_TIM_BASE + svm_mod_pattern[sector][1])) = t0 + t1;
  *((uint16_t *)(LEFT_TIM_BASE + svm_mod_pattern[sector][2])) = t0 + t1 + t2;
#endif



#ifdef RIGHT_MOTOR_SVM
  // Interpolate the rotor position
  angle = motor_state[STATE_RIGHT].act.angle + motor_state[STATE_RIGHT].ctrl.speed;

#ifdef RIGHT_MOTOR_FOC
  angle_min = motor_state[STATE_RIGHT].ctrl.angle_min;
  angle_max = motor_state[STATE_RIGHT].ctrl.angle_max;

  // Check that the new angle is inside the sector limits
  if(angle_min < angle_max) {
    // Normal situation
    angle = CLAMP(angle, angle_min, angle_max);
  } else {
    // Sector 0, angle 0 is inside the sector
      if(angle > angle_max && angle <= ANGLE_180DEG) angle = angle_max;
      else if(angle < angle_min && angle >= ANGLE_180DEG) angle = angle_min;
  }
#endif

  motor_state[STATE_RIGHT].act.angle = angle;

  // Get the vector times from the modulator
#ifdef RIGHT_MOTOR_FOC
  // Phase advance according to ctrl angle
  angle += motor_state[STATE_RIGHT].ctrl.angle;
#else
  // In normal SVM control angle is directly the modulation angle
  angle = motor_state[STATE_RIGHT].ctrl.angle;
#endif
  sector = angle_to_svm_sector(angle);
  calculate_modulator(motor_state[STATE_RIGHT].ctrl.amplitude, angle, &t0, &t1, &t2);

  // Since the timer compare is wrong way
  *((uint16_t *)(RIGHT_TIM_BASE + svm_mod_pattern[sector][0])) = t0;
  if(sector & 0x01) // Every odd sector uses "right" vector first
    *((uint16_t *)(RIGHT_TIM_BASE + svm_mod_pattern[sector][1])) = t0 + t2;
  else // Even sectors uses "left" vector first
    *((uint16_t *)(RIGHT_TIM_BASE + svm_mod_pattern[sector][1])) = t0 + t1;
  *((uint16_t *)(RIGHT_TIM_BASE + svm_mod_pattern[sector][2])) = t0 + t1 + t2;
#endif

  // Debug: LED off
  //HAL_GPIO_TogglePin(LED_PORT,LED_PIN);
}

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

//#include "control.h" // Debug
#include "math.h"
#include "svm.h"

// References for left and right motors
//volatile svm_ref_t svm_left = {0, 0};
//volatile svm_ref_t svm_right = {0, 0};

extern volatile motor_state_t motor_state[2];

// Shadow variables for counter values for dead time compensation
// Order is u_up, v_up, w_up, u_down, v_down, w_down
static uint16_t counter_l[6] = {0};
static uint16_t counter_r[6] = {0};

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

// Map to store the counter values
static const uint8_t counter_pattern[6][3] = {
  {1, 0, 2},
  {1, 2, 0},
  {2, 1, 0},
  {2, 0, 1},
  {0, 2, 1},
  {0, 1, 2}
};

// Calculate the timer values given the desired voltage vector
// length (modulation index) and angle in fixed point, return
// vector legths relative to the PWM period in t0, t1 and t2
// (t0 is half of the total zero vector length)
static void calculate_modulator(int16_t midx, uint16_t angle, uint16_t *t0, uint16_t *t1, uint16_t *t2) {
  uint16_t ta1;
  uint16_t ta2;
  uint16_t tz;
  uint16_t terr = 0;

  // Clamp < 0 modulation index to zero
  if(midx <= 0) midx = 0;
  // Clamp modulation index to 1.0
  else if(midx >= 4096) midx = 4096;

  // Clamp angle to 0...60 degrees
  // angle &= FIXED_MASK;	// 0...360 degrees
  angle = angle % ANGLE_60DEG;

  // Calculate the vector times
  ta1 = fx_mulu(midx, array_sin(angle));
  ta1 = fx_mulu(ta1, PWM_PERIOD);

  ta2 = fx_mulu(midx,  array_sin(ANGLE_60DEG - angle));
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


// Convert fixed point angle into sector number
static inline uint8_t angle_to_sector(uint16_t angle) {
  //angle &= FIXED_MASK;	// Changed to full circle = 16 bits
  if(angle < ANGLE_60DEG) return 5;
  else if(angle < ANGLE_120DEG) return 0;
  else if(angle < ANGLE_180DEG) return 1;
  else if(angle < ANGLE_240DEG) return 2;
  else if(angle < ANGLE_300DEG) return 3;
  else return 4;
}

// Timer 1 update handles space vector modulation for both motors
void TIM1_UP_IRQHandler() {
  uint16_t t0, t1, t2;
  uint16_t angle;
  uint8_t sector;
  uint16_t *counter_l_shadow = NULL;
  uint16_t *counter_r_shadow = NULL;

  // Clear the update interrupt flag
  TIM1->SR = 0; //&= ~TIM_SR_UIF;

  // DEBUG: LED on
  HAL_GPIO_TogglePin(LED_PORT,LED_PIN);

  if(TIM1->CR1 & TIM_CR1_DIR) {
    // Counting down currently, references go to up counter shadow
    counter_l_shadow = &counter_l[0];
    counter_r_shadow = &counter_r[0];
  } else {
    // Counting up currently, values go to down counter shadow
    counter_l_shadow = &counter_l[3];
    counter_r_shadow = &counter_r[3];
  }


#ifdef LEFT_MOTOR_SVM

  // Get the vector times from the modulator
  motor_state[STATE_LEFT].ctrl.angle += motor_state[STATE_LEFT].ctrl.speed;

  angle = motor_state[STATE_LEFT].ctrl.angle;
  sector = angle_to_sector(angle);
  calculate_modulator(motor_state[STATE_LEFT].ctrl.amplitude, angle, &t0, &t1, &t2);

  // TODO: Vectors are 111 -> Active 2 -> Active 1 -> 000 and back
  // Since the timer compare is wrong way
  *((uint16_t *)(LEFT_TIM_BASE + svm_mod_pattern[sector][0])) = t0;
  counter_l_shadow[counter_pattern[sector][0]] = t0;
  if(sector & 0x01) { // Every odd sector uses "right" vector first
    *((uint16_t *)(LEFT_TIM_BASE + svm_mod_pattern[sector][1])) = t0 + t2;
    counter_l_shadow[counter_pattern[sector][1]] = t0 + t2;
  } else { // Even sectors uses "left" vector first
    *((uint16_t *)(LEFT_TIM_BASE + svm_mod_pattern[sector][1])) = t0 + t1;
    counter_l_shadow[counter_pattern[sector][1]] = t0 + t1;
  }
  *((uint16_t *)(LEFT_TIM_BASE + svm_mod_pattern[sector][2])) = t0 + t1 + t2;
    counter_l_shadow[counter_pattern[sector][2]] = t0 + t2 + t3;

#endif

#ifdef RIGHT_MOTOR_SVM
  motor_state[STATE_RIGHT].ctrl.angle += motor_state[STATE_RIGHT].ctrl.speed;

  angle = motor_state[STATE_RIGHT].ctrl.angle;
  sector = angle_to_sector(angle);
  calculate_modulator(motor_state[STATE_RIGHT].ctrl.amplitude, angle, &t0, &t1, &t2);

  // TODO: Vectors are 111 -> Active 2 -> Active 1 -> 000 and back
  // Since the timer compare is wrong way
  *((uint16_t *)(RIGHT_TIM_BASE + svm_mod_pattern[sector][0])) = t0;
  counter_r_shadow[counter_pattern[sector][0]] = t0;
  if(sector & 0x01) { // Every odd sector uses "right" vector first
    *((uint16_t *)(RIGHT_TIM_BASE + svm_mod_pattern[sector][1])) = t0 + t2;
    counter_r_shadow[counter_pattern[sector][1]] = t0 + t2;
  } else { // Even sectors uses "left" vector first
    *((uint16_t *)(RIGHT_TIM_BASE + svm_mod_pattern[sector][1])) = t0 + t1;
    counter_r_shadow[counter_pattern[sector][1]] = t0 + t1;
  }
  *((uint16_t *)(RIGHT_TIM_BASE + svm_mod_pattern[sector][2])) = t0 + t1 + t2;
  counter_r_shadow[counter_pattern[sector][2]] = t0 + t1 + t2;
#endif

  // Debug: LED off
  HAL_GPIO_TogglePin(LED_PORT,LED_PIN);
}


// Dead time compensation values
volatile dead_time_t dead_time_l = {0};
volatile dead_time_t dead_time_r = {0};

// IRQ handlers for dead time compensation
// Placed here even though the values could also be used
// with the basic BLDC modulation
void EXTI4_IRQHandler(void) {
  // Left U phase
  // TIM8 CCR1

  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_4);

  int16_t dead;
  uint16_t counter = LEFT_TIM->CNT;

  //if(LEFT_TIM->CR1 & TIM_CR1_DIR) {
  if(!HAL_GPIO_ReadPin(LEFT_U_VOLT_PORT, LEFT_U_VOLT_PIN)) {
    // Counting downwards, going from high to low
    // Or pin is low after trigger
    dead = counter - counter_l[3];
    dead_time_l.u_down = dead;
  } else {
    // Upwards, going from low to high
    dead = counter - counter_l[0];
    dead_time_l.u_up = dead;
  }
}

void EXTI9_5_IRQHandler(void) {
  // Left V phase
  // TIM8 CCR2

  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_5);

  int16_t dead;
  uint16_t counter = LEFT_TIM->CNT;

  //if(LEFT_TIM->CR1 & TIM_CR1_DIR) {
  if(!HAL_GPIO_ReadPin(LEFT_V_VOLT_PORT, LEFT_V_VOLT_PIN)) {
    // Counting downwards, going from high to low
    // Or pin is low after trigger
    dead = counter - counter_l[4];
    dead_time_l.v_down = dead;
  } else {
    // Upwards, going from low to high
    dead = counter - counter_l[1];
    dead_time_l.v_up = dead;
  }
}

void EXTI0_IRQHandler(void) {
  // Right U phase
  // TIM1 CCR1

  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);

  int16_t dead;
  uint16_t counter = RIGHT_TIM->CNT;

  //if(RIGHT_TIM->CR1 & TIM_CR1_DIR) {
  if(!HAL_GPIO_ReadPin(RIGHT_U_VOLT_PORT, RIGHT_U_VOLT_PIN)) {
    // Counting downwards, going from high to low
    // Or pin is low after trigger
    dead = counter - counter_r[3];
    dead_time_r.u_down = dead;
  } else {
    // Upwards, going from low to high
    dead = counter - counter_r[0];
    dead_time_r.u_up = dead;
  }
}

void EXTI3_IRQHandler(void) {
  // Right V phase
  // TIM1 CCR2

  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_3);

  int16_t dead;
  uint16_t counter = RIGHT_TIM->CNT;

  //if(RIGHT_TIM->CR1 & TIM_CR1_DIR) {
  if(!HAL_GPIO_ReadPin(RIGHT_V_VOLT_PORT, RIGHT_V_VOLT_PIN)) {
    // Counting downwards, going from high to low
    // Or pin is low after trigger
    dead = counter - counter_r[4];
    dead_time_r.v_down = dead;
  } else {
    // Upwards, going from low to high
    dead = counter - counter_r[1];
    dead_time_r.v_up = dead;
  }
}

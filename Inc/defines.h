/*
* This file is part of the stmbl project.
*
* Copyright (C) 2013-2018 Rene Hopf <renehopf@mac.com>
* Copyright (C) 2013-2018 Nico Stute <crinq@crinq.de>
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once
#include "stm32f1xx_hal.h"

#define LEFT_HALL_PORT      GPIOB
#define LEFT_HALL_LSB_PIN   5
#define LEFT_HALL_U_PIN GPIO_PIN_5
#define LEFT_HALL_V_PIN GPIO_PIN_6
#define LEFT_HALL_W_PIN GPIO_PIN_7

#define RIGHT_HALL_PORT     GPIOC
#define RIGHT_HALL_LSB_PIN  10
#define RIGHT_HALL_U_PIN GPIO_PIN_10
#define RIGHT_HALL_V_PIN GPIO_PIN_11
#define RIGHT_HALL_W_PIN GPIO_PIN_12

#define CTRL_TIM TIM3

#define LEFT_TIM TIM8
#define LEFT_TIM_BASE (TIM8_BASE)
#define LEFT_TIM_U CCR1
#define LEFT_TIM_UH_PIN GPIO_PIN_6
#define LEFT_TIM_UH_PORT GPIOC
#define LEFT_TIM_UL_PIN GPIO_PIN_7
#define LEFT_TIM_UL_PORT GPIOA
#define LEFT_TIM_V CCR2
#define LEFT_TIM_VH_PIN GPIO_PIN_7
#define LEFT_TIM_VH_PORT GPIOC
#define LEFT_TIM_VL_PIN GPIO_PIN_0
#define LEFT_TIM_VL_PORT GPIOB
#define LEFT_TIM_W CCR3
#define LEFT_TIM_WH_PIN GPIO_PIN_8
#define LEFT_TIM_WH_PORT GPIOC
#define LEFT_TIM_WL_PIN GPIO_PIN_1
#define LEFT_TIM_WL_PORT GPIOB

#define RIGHT_TIM TIM1
#define RIGHT_TIM_BASE (TIM1_BASE)
#define RIGHT_TIM_U CCR1
#define RIGHT_TIM_UH_PIN GPIO_PIN_8
#define RIGHT_TIM_UH_PORT GPIOA
#define RIGHT_TIM_UL_PIN GPIO_PIN_13
#define RIGHT_TIM_UL_PORT GPIOB
#define RIGHT_TIM_V CCR2
#define RIGHT_TIM_VH_PIN GPIO_PIN_9
#define RIGHT_TIM_VH_PORT GPIOA
#define RIGHT_TIM_VL_PIN GPIO_PIN_14
#define RIGHT_TIM_VL_PORT GPIOB
#define RIGHT_TIM_W CCR3
#define RIGHT_TIM_WH_PIN GPIO_PIN_10
#define RIGHT_TIM_WH_PORT GPIOA
#define RIGHT_TIM_WL_PIN GPIO_PIN_15
#define RIGHT_TIM_WL_PORT GPIOB

#define RIGHT_DC_CUR_PIN GPIO_PIN_0
#define RIGHT_U_VOLT_PIN GPIO_PIN_0
#define RIGHT_V_VOLT_PIN GPIO_PIN_3
#define RIGHT_OC_PIN GPIO_PIN_6

#define RIGHT_DC_CUR_PORT GPIOC
#define RIGHT_U_VOLT_PORT GPIOA
#define RIGHT_V_VOLT_PORT GPIOC
#define RIGHT_OC_PORT GPIOA

#define LEFT_DC_CUR_PIN GPIO_PIN_1
#define LEFT_V_VOLT_PIN GPIO_PIN_4
#define LEFT_W_VOLT_PIN GPIO_PIN_5
#define LEFT_OC_PIN GPIO_PIN_12

#define LEFT_DC_CUR_PORT GPIOC
#define LEFT_V_VOLT_PORT GPIOC
#define LEFT_W_VOLT_PORT GPIOC
#define LEFT_OC_PORT GPIOB

#define DCLINK_PIN GPIO_PIN_2
#define DCLINK_PORT GPIOC

#define LED_PIN GPIO_PIN_2
#define LED_PORT GPIOB

#define BUZZER_PIN GPIO_PIN_4
#define BUZZER_PORT GPIOA

#define SWITCH_PIN GPIO_PIN_1
#define SWITCH_PORT GPIOA

#define OFF_PIN GPIO_PIN_5
#define OFF_PORT GPIOA

#define BUTTON_PIN GPIO_PIN_1
#define BUTTON_PORT GPIOA

#define CHARGER_PIN GPIO_PIN_12
#define CHARGER_PORT GPIOA

// Structure that stores analog measurements
typedef struct {
  uint16_t v_battery;
  uint16_t v_switch;
  uint16_t v_ref;
  uint16_t temperature;
  uint16_t analog_ref_1;
  uint16_t analog_ref_2;
} adc_buf_t;


// Struct holding relevant state variables
// (to be shared across modules)
typedef struct {
  // Actual values
  struct {
    int16_t speed;		// Actual rotation speed in p.u.
    int16_t period;		// Period of hall-sensor sector changes in PWM ticks
    uint8_t sector;		// Rotor sector from HALL sensors
    uint16_t angle;		// Accurate rotor position if available (e.g. FOC estimate) (p.u.)
    int16_t current[3];	// Current of phases A, B, C (p.u.)
  } act;

  // Reference(s)
  struct {
    int16_t value;	// Typically speed reference
    uint8_t control_mode;
  } ref;

  // Control outputs to modulator
  struct {
    int16_t amplitude;	// Can also be < 0
    uint16_t angle;		// Angle (or angle advance)
    uint16_t angle_min;
    uint16_t angle_max;
    int8_t speed;		// Speed in angle increments per modulator call for FOC
    uint8_t enable;
  } ctrl;
} motor_state_t;

#define STATE_LEFT 0
#define STATE_RIGHT 1

// Control modes
#define CONTROL_SPEED 		0
#define CONTROL_TORQUE 		1
#define CONTROL_ANGLE 		2

// Stalled speed period limit
#define PERIOD_STOP			0x7F00
#define PERIOD_MAX			0x7E00

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

#define RIGHT_TIM TIM8
#define RIGHT_TIM_BASE (TIM8_BASE)
#define RIGHT_TIM_U CCR1
#define RIGHT_TIM_UH_PIN GPIO_PIN_6
#define RIGHT_TIM_UH_PORT GPIOC
#define RIGHT_TIM_UL_PIN GPIO_PIN_7
#define RIGHT_TIM_UL_PORT GPIOA
#define RIGHT_TIM_V CCR2
#define RIGHT_TIM_VH_PIN GPIO_PIN_7
#define RIGHT_TIM_VH_PORT GPIOC
#define RIGHT_TIM_VL_PIN GPIO_PIN_0
#define RIGHT_TIM_VL_PORT GPIOB
#define RIGHT_TIM_W CCR3
#define RIGHT_TIM_WH_PIN GPIO_PIN_8
#define RIGHT_TIM_WH_PORT GPIOC
#define RIGHT_TIM_WL_PIN GPIO_PIN_1
#define RIGHT_TIM_WL_PORT GPIOB

#define LEFT_TIM TIM1
#define LEFT_TIM_BASE (TIM1_BASE)
#define LEFT_TIM_U CCR1
#define LEFT_TIM_UH_PIN GPIO_PIN_8
#define LEFT_TIM_UH_PORT GPIOA
#define LEFT_TIM_UL_PIN GPIO_PIN_13
#define LEFT_TIM_UL_PORT GPIOB
#define LEFT_TIM_V CCR2
#define LEFT_TIM_VH_PIN GPIO_PIN_9
#define LEFT_TIM_VH_PORT GPIOA
#define LEFT_TIM_VL_PIN GPIO_PIN_14
#define LEFT_TIM_VL_PORT GPIOB
#define LEFT_TIM_W CCR3
#define LEFT_TIM_WH_PIN GPIO_PIN_10
#define LEFT_TIM_WH_PORT GPIOA
#define LEFT_TIM_WL_PIN GPIO_PIN_15
#define LEFT_TIM_WL_PORT GPIOB

#define RIGHT_DC_CUR_PIN GPIO_PIN_0
#define RIGHT_U_VOLT_PIN GPIO_PIN_0
#define RIGHT_V_VOLT_PIN GPIO_PIN_3
#define RIGHT_OC_PIN GPIO_PIN_6

#define RIGHT_DC_CUR_PORT GPIOC
#define RIGHT_U_VOLT_PORT GPIOA
#define RIGHT_V_VOLT_PORT GPIOC
#define RIGHT_OC_PORT GPIOA

#define LEFT_DC_CUR_PIN GPIO_PIN_1
#define LEFT_U_VOLT_PIN GPIO_PIN_4
#define LEFT_V_VOLT_PIN GPIO_PIN_5
#define LEFT_OC_PIN GPIO_PIN_12

#define LEFT_DC_CUR_PORT GPIOC
#define LEFT_U_VOLT_PORT GPIOC
#define LEFT_V_VOLT_PORT GPIOC
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

#define ABS(a) (((a) < 0.0) ? -(a) : (a))
#define LIMIT(x, lowhigh) (((x) > (lowhigh)) ? (lowhigh) : (((x) < (-lowhigh)) ? (-lowhigh) : (x)))
#define SAT(x, lowhigh) (((x) > (lowhigh)) ? (1.0) : (((x) < (-lowhigh)) ? (-1.0) : (0.0)))
#define SAT2(x, low, high) (((x) > (high)) ? (1.0) : (((x) < (low)) ? (-1.0) : (0.0)))
#define STEP(from, to, step) (((from) < (to)) ? (MIN((from) + (step), (to))) : (MAX((from) - (step), (to))))
#define DEG(a) ((a)*M_PI / 180.0)
#define RAD(a) ((a)*180.0 / M_PI)
#define SIGN(a) (((a) < 0.0) ? (-1.0) : (((a) > 0.0) ? (1.0) : (0.0)))
#define CLAMP(x, low, high) (((x) > (high)) ? (high) : (((x) < (low)) ? (low) : (x)))
#define SCALE(value, high, max) MIN(MAX(((max) - (value)) / ((max) - (high)), 0.0), 1.0)
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#define MIN3(a, b, c) MIN(a, MIN(b, c))
#define MAX3(a, b, c) MAX(a, MAX(b, c))

// Structure that stores analog measurements
typedef struct {
  uint16_t v_battery;
  uint16_t v_switch;
  uint16_t temperature;
  uint16_t analog_ref_1;
  uint16_t analog_ref_2;
} adc_buf_t;

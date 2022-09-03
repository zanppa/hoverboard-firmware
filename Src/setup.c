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
#include "defines.h"
#include "config.h"

void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

  // Digital Input pins
  GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;

  // Charge detect pin should have internal pull-up
  GPIO_InitStruct.Pull  = GPIO_PULLUP;
  GPIO_InitStruct.Pin = CHARGER_PIN;
  HAL_GPIO_Init(CHARGER_PORT, &GPIO_InitStruct);

  // Other pins are driven by external pull-up/down or push-pull
  GPIO_InitStruct.Pull  = GPIO_NOPULL;

  GPIO_InitStruct.Pin = LEFT_HALL_U_PIN;
  HAL_GPIO_Init(LEFT_HALL_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LEFT_HALL_V_PIN;
  HAL_GPIO_Init(LEFT_HALL_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LEFT_HALL_W_PIN;
  HAL_GPIO_Init(LEFT_HALL_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = RIGHT_HALL_U_PIN;
  HAL_GPIO_Init(RIGHT_HALL_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = RIGHT_HALL_V_PIN;
  HAL_GPIO_Init(RIGHT_HALL_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = RIGHT_HALL_W_PIN;
  HAL_GPIO_Init(RIGHT_HALL_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = BUTTON_PIN;
  HAL_GPIO_Init(BUTTON_PORT, &GPIO_InitStruct);

  // TODO: Left and right OC pins might be swapped in the schematic/pcb!
  // To be checked
  GPIO_InitStruct.Pin = LEFT_OC_PIN;
  HAL_GPIO_Init(RIGHT_OC_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = RIGHT_OC_PIN;
  HAL_GPIO_Init(RIGHT_OC_PORT, &GPIO_InitStruct);


  // Output push-pull pins
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;

  GPIO_InitStruct.Pin = LED_PIN;
  HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = BUZZER_PIN;
  HAL_GPIO_Init(BUZZER_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = OFF_PIN;
  HAL_GPIO_Init(OFF_PORT, &GPIO_InitStruct);


  // Analog IO pins
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;

  //Current / Phase sense and battery measurements
  GPIO_InitStruct.Pin = LEFT_DC_CUR_PIN;
  HAL_GPIO_Init(LEFT_DC_CUR_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LEFT_V_VOLT_PIN;
  HAL_GPIO_Init(LEFT_V_VOLT_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LEFT_W_VOLT_PIN;
  HAL_GPIO_Init(LEFT_W_VOLT_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = RIGHT_DC_CUR_PIN;
  HAL_GPIO_Init(RIGHT_DC_CUR_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = RIGHT_U_VOLT_PIN;
  HAL_GPIO_Init(RIGHT_U_VOLT_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = RIGHT_V_VOLT_PIN;
  HAL_GPIO_Init(RIGHT_V_VOLT_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = DCLINK_PIN;
  HAL_GPIO_Init(DCLINK_PORT, &GPIO_InitStruct);

  // Left side UART used as analog input references or UART
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);



  //alternate function push-pull
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;

  //left MTR timer HI pins
  GPIO_InitStruct.Pin = LEFT_TIM_UH_PIN;
  HAL_GPIO_Init(LEFT_TIM_UH_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LEFT_TIM_VH_PIN;
  HAL_GPIO_Init(LEFT_TIM_VH_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LEFT_TIM_WH_PIN;
  HAL_GPIO_Init(LEFT_TIM_WH_PORT, &GPIO_InitStruct);

  //left MTR timer LO pins
  GPIO_InitStruct.Pin = LEFT_TIM_UL_PIN;
  HAL_GPIO_Init(LEFT_TIM_UL_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LEFT_TIM_VL_PIN;
  HAL_GPIO_Init(LEFT_TIM_VL_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LEFT_TIM_WL_PIN;
  HAL_GPIO_Init(LEFT_TIM_WL_PORT, &GPIO_InitStruct);

  //right MTR timer HI pins
  GPIO_InitStruct.Pin = RIGHT_TIM_UH_PIN;
  HAL_GPIO_Init(RIGHT_TIM_UH_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = RIGHT_TIM_VH_PIN;
  HAL_GPIO_Init(RIGHT_TIM_VH_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = RIGHT_TIM_WH_PIN;
  HAL_GPIO_Init(RIGHT_TIM_WH_PORT, &GPIO_InitStruct);

  //right MTR timer LO pins
  GPIO_InitStruct.Pin = RIGHT_TIM_UL_PIN;
  HAL_GPIO_Init(RIGHT_TIM_UL_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = RIGHT_TIM_VL_PIN;
  HAL_GPIO_Init(RIGHT_TIM_VL_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = RIGHT_TIM_WL_PIN;
  HAL_GPIO_Init(RIGHT_TIM_WL_PORT, &GPIO_InitStruct);
}


void control_timer_init(void)
{
  TIM_HandleTypeDef htim_control;

  __HAL_RCC_TIM3_CLK_ENABLE();

  htim_control.Instance               = CTRL_TIM;
  htim_control.Init.Prescaler         = 0;
  htim_control.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim_control.Init.Period            = CONTROL_PERIOD;
  htim_control.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim_control.Init.RepetitionCounter = 0;
  htim_control.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  HAL_TIM_PWM_Init(&htim_control);

  // Control task should run at a quite low priority compared
  // to modulator and current measurement
  HAL_NVIC_SetPriority(TIM3_IRQn, 10, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);

  HAL_TIM_Base_Start_IT(&htim_control);
}



/*
 * Initialize timers for left and right side motors
 */

void MX_TIM_Init(void) {
  TIM_HandleTypeDef htim_right;
  TIM_HandleTypeDef htim_left;

  __HAL_RCC_TIM1_CLK_ENABLE();
  __HAL_RCC_TIM8_CLK_ENABLE();

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;
  TIM_SlaveConfigTypeDef sTimConfig;

  // Initialize timer as center-aligned mode
  // Timer counts from 0 to "period"-1, creates overflow event, then counts
  // down to 1 and creates underflow event. Then it restarts.
  // pre-scaler = 0 and divier = 1 i.e.
  // the timer runs at crystal(?) frequency
  htim_right.Instance               = RIGHT_TIM;	// TIM1
  htim_right.Init.Prescaler         = 0;
  htim_right.Init.CounterMode       = TIM_COUNTERMODE_CENTERALIGNED3; // Interrupts at up- and downcounting
  htim_right.Init.Period            = PWM_PERIOD;
  htim_right.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim_right.Init.RepetitionCounter = 0;
  htim_right.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  //htim_right.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_PWM_Init(&htim_right);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_ENABLE;
  sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim_right, &sMasterConfig);

  sConfigOC.OCMode       = TIM_OCMODE_PWM2;
  sConfigOC.Pulse        = 0;
  sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_LOW;
  sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_SET;
  HAL_TIM_PWM_ConfigChannel(&htim_right, &sConfigOC, TIM_CHANNEL_1);
  HAL_TIM_PWM_ConfigChannel(&htim_right, &sConfigOC, TIM_CHANNEL_2);
  HAL_TIM_PWM_ConfigChannel(&htim_right, &sConfigOC, TIM_CHANNEL_3);

  sBreakDeadTimeConfig.OffStateRunMode  = TIM_OSSR_ENABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
  sBreakDeadTimeConfig.LockLevel        = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime         = DEAD_TIME;
  sBreakDeadTimeConfig.BreakState       = TIM_BREAK_ENABLE;		// Overcurrent stops timer
  sBreakDeadTimeConfig.BreakPolarity    = TIM_BREAKPOLARITY_LOW;
  sBreakDeadTimeConfig.AutomaticOutput  = TIM_AUTOMATICOUTPUT_DISABLE;
  HAL_TIMEx_ConfigBreakDeadTime(&htim_right, &sBreakDeadTimeConfig);


  // Left motor timer
  htim_left.Instance               = LEFT_TIM; // TIM8
  htim_left.Init.Prescaler         = 0;
  htim_left.Init.CounterMode       = TIM_COUNTERMODE_CENTERALIGNED3; // Interrupts at up- and downcounting
  htim_left.Init.Period            = PWM_PERIOD;
  htim_left.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim_left.Init.RepetitionCounter = 0;
  htim_left.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  //htim_left.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_PWM_Init(&htim_left);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_ENABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim_left, &sMasterConfig);

  sTimConfig.InputTrigger = TIM_TS_ITR0;	// TIM1_TRGO
  sTimConfig.SlaveMode    = TIM_SLAVEMODE_GATED;
  HAL_TIM_SlaveConfigSynchronization(&htim_left, &sTimConfig);

  sConfigOC.OCMode       = TIM_OCMODE_PWM2;
  sConfigOC.Pulse        = 0;
  sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_LOW;
  sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_SET;
  HAL_TIM_PWM_ConfigChannel(&htim_left, &sConfigOC, TIM_CHANNEL_1);
  HAL_TIM_PWM_ConfigChannel(&htim_left, &sConfigOC, TIM_CHANNEL_2);
  HAL_TIM_PWM_ConfigChannel(&htim_left, &sConfigOC, TIM_CHANNEL_3);

  sBreakDeadTimeConfig.OffStateRunMode  = TIM_OSSR_ENABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
  sBreakDeadTimeConfig.LockLevel        = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime         = DEAD_TIME;
  sBreakDeadTimeConfig.BreakState       = TIM_BREAK_ENABLE;		// Overcurrent stops timer
  sBreakDeadTimeConfig.BreakPolarity    = TIM_BREAKPOLARITY_LOW;
  sBreakDeadTimeConfig.AutomaticOutput  = TIM_AUTOMATICOUTPUT_DISABLE;
  HAL_TIMEx_ConfigBreakDeadTime(&htim_left, &sBreakDeadTimeConfig);

  // Enable update interrupts
  //LEFT_TIM->DIER |= TIM_DIER_UIE;
  //LEFT_TIM->CR1 &= ~(TIM_CR1_URS | TIM_CR1_UDIS); // Clear update disable
  //RIGHT_TIM->DIER |= TIM_DIER_UIE;
  //RIGHT_TIM->CR1 &= ~(TIM_CR1_URS | TIM_CR1_UDIS); // Clear update disable

  // Timer 1 interrupt is used for SVM if one motor uses it
  TIM1->DIER |= TIM_DIER_UIE;
  TIM1->CR1 &= ~(TIM_CR1_URS | TIM_CR1_UDIS); // Clear update disable

  // Modulator runs at a rather high priority, but still lower
  // than current measurement
  HAL_NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);

  // Timer 8 interrupt is used for BLDC
  TIM8->DIER |= TIM_DIER_UIE;
  TIM8->CR1 &= ~(TIM_CR1_URS | TIM_CR1_UDIS); // Clear update disable

  HAL_NVIC_SetPriority(TIM8_UP_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(TIM8_UP_IRQn);

  // Disable outputs
  LEFT_TIM->BDTR &= ~TIM_BDTR_MOE;
  RIGHT_TIM->BDTR &= ~TIM_BDTR_MOE;

  // Start the timers
  HAL_TIM_PWM_Start_IT(&htim_left, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start_IT(&htim_left, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start_IT(&htim_left, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim_left, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim_left, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim_left, TIM_CHANNEL_3);

  HAL_TIM_PWM_Start_IT(&htim_right, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start_IT(&htim_right, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start_IT(&htim_right, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim_right, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim_right, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim_right, TIM_CHANNEL_3);

  htim_left.Instance->RCR = 1;

  __HAL_TIM_ENABLE(&htim_right);
}


// Initialize timer for triggering shunt current measurement
void ishunt_timer_init(void)
{
  TIM_HandleTypeDef htim_ishunt;
  TIM_SlaveConfigTypeDef sTimConfig;

  __HAL_RCC_TIM2_CLK_ENABLE();

  htim_ishunt.Instance               = TIM2;
  htim_ishunt.Init.Prescaler         = 0;
  htim_ishunt.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim_ishunt.Init.Period            = PWM_PERIOD;
  htim_ishunt.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim_ishunt.Init.RepetitionCounter = 0;
  htim_ishunt.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  HAL_TIM_PWM_Init(&htim_ishunt);

  sTimConfig.InputTrigger = TIM_TS_ITR1;	// TIM8 triggers
  sTimConfig.SlaveMode    = TIM_SLAVEMODE_RESET;
  HAL_TIM_SlaveConfigSynchronization(&htim_ishunt, &sTimConfig);

  TIM2->DIER |= TIM_DIER_CC2IE;	// Enable capture-compare 2 interrupt (launches ADC2)

  // Low priority but higher than control task
  HAL_NVIC_SetPriority(TIM2_IRQn, 8, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);

  HAL_TIM_PWM_Start_IT(&htim_ishunt, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim_ishunt, TIM_CHANNEL_2);
}

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

#include "stm32f1xx_hal.h"
#include "defines.h"
#include "setup.h"
#include "config.h"
#include "uart.h"
#include "cfgbus.h"
#include "modbus.h"
#include "control.h"
#include "eeprom.h"
#include "adc.h"
#include "svm.h"

void SystemClock_Config(void);

extern ADC_HandleTypeDef hadc1;
//extern ADC_HandleTypeDef hadc2;

extern volatile motor_state_t motor_state[2];
extern volatile adc_buf_t analog_meas;

//extern volatile dead_time_t dead_time_l;
extern volatile dead_time_t dead_time_r;


int main(void) {

  HAL_Init();
  __HAL_RCC_AFIO_CLK_ENABLE();

  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
  /* System interrupt init*/
  /* MemoryManagement_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
  /* BusFault_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
  /* UsageFault_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);
  /* SVCall_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SVCall_IRQn, 0, 0);
  /* DebugMonitor_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DebugMonitor_IRQn, 0, 0);
  /* PendSV_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(PendSV_IRQn, 0, 0);

  SystemClock_Config();

  __HAL_RCC_DMA1_CLK_DISABLE();

  MX_GPIO_Init();

  initialize_control_state();

  MX_TIM_Init();

  ADC1_init();
  HAL_ADC_Start(&hadc1);
  ADC1_calibrate();

  UART_Init(0, 1);	// Use only UART3 for modbus

  ee_init();
  CfgInit();

  // Enable power latch
  HAL_GPIO_WritePin(OFF_PORT, OFF_PIN, 1);

  //HAL_ADC_Start(&hadc2);

  // Enable both motor drivers
  motor_state[STATE_LEFT].ctrl.enable = 1;
  motor_state[STATE_RIGHT].ctrl.enable = 1;

  //UARTRxEnable(UARTCh2, 1);
  UARTRxEnable(UARTCh3, 1);

  control_timer_init();


  while(1)
  {
    //show user board is alive
    //led_update();

    //update cfg_bus communication
    mb_update();


    // Do all "slow" calculations here, on background
    // These will be pre-empted by everything more important
    // And all variables most likely will change so copy them locally first

    float v_battery = analog_meas.v_battery;
    v_battery = (v_battery - ADC_BATTERY_OFFSET) * ADC_BATTERY_VOLTS;
    //cfg.vars.v_battery = v_battery;

    float act_speed = motor_state[STATE_LEFT].act.period;
    act_speed = MOTOR_PERIOD_TO_MS / act_speed;
    //cfg.vars.speed_l = act_speed;

    act_speed = motor_state[STATE_RIGHT].act.period;
    act_speed = MOTOR_PERIOD_TO_MS / act_speed;
    //cfg.vars.speed_r = act_speed;

    // DEBUG: Dead time values to config bus
    cfg.vars.dead_ruu = dead_time_r.u_up;
    cfg.vars.dead_rud = dead_time_r.u_down;
    cfg.vars.dead_rvu = dead_time_r.v_up;
    cfg.vars.dead_rvd = dead_time_r.v_down;
  }
}

/** System Clock Configuration
*/
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  //Initializes the CPU, AHB and APB bus oscillator
  //HSI/2 * 16 = 8/2*16 = 64MHz
  // Use the 8 MHz high speed internal oscillator with PLL set to 16/2 ==>  64 MHz
  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL          = RCC_PLL_MUL16;

  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  //Initializes the CPU, AHB and APB clocks
  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;  // APB1 @ 32 Mhz (max 36 MHz)
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  // PB2 @ 64 MHz (e.g. ADC) (max 72 MHz)

  // APB1: USART23 etc
  // APB2: ADC123, USART1, GPIO etc
  // TIM 3,4,5,6,7 are APB1 x1 if APB1 prescaler is 1, otherwise APB1 x2 ==> 64 MHz
  // TIM 1,8 are APB2 x1 if APB2 prescaler is 1, otherwise APB2 x2 ==> 64 MHz

  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  // Configure ADC clock as 1/8th of PCLK2
  // PCLK2 = APB2; APB2 / 8 = 64MHz / 8 ==> ADC @ 8MHz
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection    = RCC_ADCPCLK2_DIV8;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);


  // Configure the Systick interrupt time and interrupt
  // This already sets the interrupt priority to lowest value
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
}

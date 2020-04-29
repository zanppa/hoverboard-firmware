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
#include "math.h"
#include "imeas.h"
#include "powersw.h"

void SystemClock_Config(void);

extern ADC_HandleTypeDef adc_rdson;
extern ADC_HandleTypeDef hadc3;

extern volatile motor_state_t motor_state[2];
extern volatile adc_buf_t analog_meas;
extern volatile i_meas_t i_meas;
extern volatile uint16_t rdson_offset[4];

extern volatile uint8_t generic_adc_conv_done;


int main(void) {
  //uint16_t button_time = 0;

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

  // Initialize control modes
  motor_state[STATE_LEFT].ref.control_mode = CONTROL_TORQUE;
  motor_state[STATE_RIGHT].ref.control_mode = CONTROL_TORQUE;

  __HAL_RCC_DMA1_CLK_DISABLE();
  __HAL_RCC_DMA2_CLK_DISABLE();

  MX_GPIO_Init();

  // Enable power latch here so that the board keeps
  // powered on during initialization
  HAL_GPIO_WritePin(OFF_PORT, OFF_PIN, 1);

  // Initialize control state, e.g. rotor position
  // according to hall sensors and so forth
  initialize_control_state();
  motor_state[STATE_LEFT].ref.control_mode = CONTROL_TORQUE;
  motor_state[STATE_RIGHT].ref.control_mode = CONTROL_TORQUE;

  // Initialize generic measurements with ADC3
  ADC3_init();
  HAL_ADC_Start(&hadc3);
  ADC3_calibrate();

#ifdef I_MEAS_RDSON
  // Initialize Rds,on measurements with ADC1
  ADC1_init();
  HAL_ADC_Start(&adc_rdson);
#endif

  // Initialize control timer
  control_timer_init();

  // Initialize PWM timers
  MX_TIM_Init();

#ifdef POWER_BUTTON_NORMAL
  // Check the power-up sequence
  powersw_on_sequence();
#endif

  // Enable both motor drivers
  enable_motors(0x01 | 0x02);

#ifdef I_MEAS_RDSON
  // Rds,on measurement must be calibrated when modulator is running
  // without load (0 reference)
  ADC1_calibrate();
#endif


  // Initialize UART for modbus
  // do this last so that we don't accidentally update the reference...
#if defined(LEFT_SENSOR_MODBUS) || defined(RIGHT_SENSOR_MODBUS)

#ifdef LEFT_SENSOR_MODBUS
  UART_Init(0, 1);	// Use UART3 for modbus
#else
  UART_Init(1, 0);	// Use UART2
#endif

  // Initialize EEPROM and config bus
  ee_init();
  CfgInit();

  UARTRxEnable(CFG_BUS_UART, 1);
#endif


  while(1)
  {
    //show user board is alive
    //led_update();
    //HAL_GPIO_WritePin(LED_PORT,LED_PIN, 1);

    // Check for short circuit status
    check_sc();

#if defined(LEFT_SENSOR_MODBUS) || defined(RIGHT_SENSOR_MODBUS)
    //update cfg_bus communication
    mb_update();
#endif

#if 0
    // Check if user requested power off
    powersw_off_sequence();
#endif

    // Check if power button was pressed long for fault reset
    if(powersw_fault_reset()) clear_fault(0x01 | 0x02);		// Reset all faults

    // Clear the ADC flag for next loop
    generic_adc_conv_done = 0;

    // Update references
    // TODO: Select reference source (e.g. analog or modbus)
    // motor_state[STATE_LEFT].ref.value = cfg.vars.setpoint_l;
    // motor_state[STATE_RIGHT].ref.value = cfg.vars.setpoint_r;

    // Do all "slow" calculations here, on background
    // These will be pre-empted by everything more important
    // And all variables most likely will change so copy them locally first

    float v_battery = analog_meas.v_battery;
    v_battery = (v_battery - ADC_BATTERY_OFFSET) * ADC_BATTERY_VOLTS;
    //cfg.vars.v_battery = v_battery;

    float act_speed = motor_state[STATE_LEFT].act.speed;
    act_speed = act_speed * MOTOR_SPEED / FIXED_ONE;
    //cfg.vars.speed_l = act_speed;

    act_speed = motor_state[STATE_RIGHT].act.speed;
    act_speed = act_speed * MOTOR_SPEED / FIXED_ONE;
    //cfg.vars.speed_r = act_speed;

    // Copy rdson measurement values to configbus
    cfg.vars.rdsonla = i_meas.i_lA;
    cfg.vars.rdsonlb = i_meas.i_lB;
    cfg.vars.rdsonrb = i_meas.i_rB;
    cfg.vars.rdsonrc = i_meas.i_rC;
    cfg.vars.lboff = rdson_offset[0];
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

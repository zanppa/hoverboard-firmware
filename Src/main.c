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

#if defined(DATALOGGER_ENABLE)
extern volatile DATALOGGER_TYPE datalogger[DATALOGGER_MAX+1][DATALOGGER_CHANNELS];
extern volatile uint8_t datalogger_trigger;
extern void *datalogger_var[DATALOGGER_CHANNELS];
#endif

void SystemClock_Config(void);

extern ADC_HandleTypeDef adc_rdson;
extern ADC_HandleTypeDef hadc2; // In DUAL ADC mode
extern ADC_HandleTypeDef hadc3;

extern volatile motor_state_t motor_state[2];
extern volatile adc_buf_t analog_meas;
extern volatile i_meas_t i_meas;
extern volatile uint16_t rdson_offset[4];

extern volatile uint8_t generic_adc_conv_done;
extern volatile uint8_t status_bits;
extern volatile uint8_t fault_bits;

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
  //motor_state[STATE_LEFT].ref.control_mode = CONTROL_SPEED;  // BLDC
  //motor_state[STATE_RIGHT].ref.control_mode = CONTROL_TORQUE; // SVM
  motor_state[STATE_LEFT].ref.control_mode = CONTROL_TORQUE;  // BLDC
  motor_state[STATE_RIGHT].ref.control_mode = CONTROL_TORQUE; // BLDC
  //motor_state[STATE_LEFT].ref.control_mode = CONTROL_SPEED;  // FOC
  //motor_state[STATE_RIGHT].ref.control_mode = CONTROL_SPEED; // FOC

  __HAL_RCC_DMA1_CLK_DISABLE();
  __HAL_RCC_DMA2_CLK_DISABLE();

  MX_GPIO_Init();

  // Enable power latch here so that the board keeps
  // powered on during initialization
  HAL_GPIO_WritePin(OFF_PORT, OFF_PIN, 1);

  // Initialize control state, e.g. rotor position
  // according to hall sensors and so forth
  initialize_control_state();

  // Initialize generic measurements with ADC3
  ADC3_init();
  HAL_ADC_Start(&hadc3);
  ADC3_calibrate();

#ifdef I_MEAS_RDSON
  // Initialize Rds,on measurements with ADC1
  ADC1_init();
#if defined(DUAL_ADC_MODE)
  ADC2_init();
  HAL_ADC_Start(&hadc2);
#endif
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

  // Wait until we're ready to start (continue forward)
  while(!(status_bits && STATUS_READY));


  // Enable both motor drivers
  enable_motors(0x01 | 0x02);

#ifdef I_MEAS_RDSON
  // Rds,on measurement must be calibrated when modulator is running
  // without load (0 reference)
  ADC12_calibrate();
#endif


  // Initialize UARTs
#if defined(LEFT_SENSOR_MODBUS) || defined(LEFT_SENSOR_SCOPE)
  UART_Init(0, 1);	// Use UART3 for modbus
#endif
#if defined(RIGHT_SENSOR_MODBUS) || defined(RIGHT_SENSOR_SCOPE)
  UART_Init(1, 0);	// Use UART2
#endif

  // Initialize modbus
  // Initialize EEPROM and config bus
  ee_init();
  CfgInit();
#if defined(LEFT_SENSOR_MODBUS) || defined(RIGHT_SENSOR_MODBUS)
  UARTRxEnable(CFG_BUS_UART, 1);
#endif


  // Configure datalogger variables
  // for debugging purposes
#if defined(DATALOGGER_ENABLE)
  // Sample rotor position
  datalogger_var[0] = (void *)&motor_state[STATE_RIGHT].act.angle;
  datalogger_var[1] = (void *)&motor_state[STATE_RIGHT].act.sector;

  // Sample current measurements
  datalogger_var[2] = (void *)&i_meas.i_rA;
  datalogger_var[3] = (void *)&i_meas.i_rB;

  // Sample modulator output --> PWM references
  datalogger_var[4] = (void *)&RIGHT_TIM->RIGHT_TIM_U;
  datalogger_var[5] = (void *)&RIGHT_TIM->RIGHT_TIM_V;
  datalogger_var[6] = (void *)&RIGHT_TIM->RIGHT_TIM_W;

  // Sample battery voltage
  datalogger_var[7] = (void *)&analog_meas.v_battery;
#endif


  // Play a turn-on tune when we're ready to start
  // Play after config bus enable so buzzer enable status is known
  power_tune(1);


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

    // Check if user requested power off
    powersw_off_sequence();

    // Check if power button was pressed long for fault reset, but only if faults are active
#if 0
    if(powersw_fault_reset() && fault_bits) {
      initialize_control_state();
      clear_fault(0x01 | 0x02);		// Reset all faults
      enable_motors(0x01 | 0x02);
    }
#endif

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
    v_battery = (v_battery + ADC_BATTERY_OFFSET) * ADC_BATTERY_VOLTS;
    cfg.vars.v_battery = v_battery;

    //float act_speed = motor_state[STATE_LEFT].act.speed;
    //act_speed = act_speed * MOTOR_SPEED / FIXED_ONE;
    //cfg.vars.speed_l = act_speed;
    //cfg.vars.speed_l = 15 ;//motor_state[STATE_LEFT].act.speed;

    //act_speed = motor_state[STATE_RIGHT].act.speed;
    //act_speed = act_speed * MOTOR_SPEED / FIXED_ONE;
    //cfg.vars.speed_r = act_speed;
    //cfg.vars.speed_r = motor_state[STATE_RIGHT].act.speed;


    // Update rotor positions (sector)
    cfg.vars.pos_l = motor_state[STATE_LEFT].act.sector;
    cfg.vars.pos_r = motor_state[STATE_RIGHT].act.sector;


    // Store analog measurement values to config bus
    cfg.vars.temperature = analog_meas.temperature;
    cfg.vars.aref1 = analog_meas.analog_ref_1;
    cfg.vars.aref2 = analog_meas.analog_ref_2;
    cfg.vars.pwm_l = motor_state[STATE_LEFT].ctrl.amplitude;
    cfg.vars.pwm_r = motor_state[STATE_RIGHT].ctrl.amplitude;
    //cfg.vars.l_angle_adv = motor_state[STATE_LEFT].ctrl.angle;
    //cfg.vars.r_angle_adv = motor_state[STATE_RIGHT].ctrl.angle;
    cfg.vars.fault_code = fault_bits;
    cfg.vars.status_code = status_bits;


    // Copy raw rdson measurement values to configbus
    //cfg.vars.rdsonla = i_meas.i_lA;
    //cfg.vars.rdsonlb = i_meas.i_lB;
    //cfg.vars.rdsonrb = i_meas.i_rB;
    //cfg.vars.rdsonrc = i_meas.i_rC;

#if defined(DATALOGGER_ENABLE)
  // Handle datalogger control
  if(cfg.vars.dlog_ctrl & 1) { // Trigger manually
    if(!datalogger_trigger) {
      datalogger_trigger = 1;
      cfg.vars.dlog_ctrl &= ~3; // Clear trigger & ready bits from cfgbus register
    } else {
      cfg.vars.dlog_ctrl &= ~1; // Clear trigger since datalogger is running
    }
  } else {
    if(!datalogger_trigger)
      cfg.vars.dlog_ctrl |= 2;	// Raise ready flag
  }

  // Transfer data through config bus
  uint16_t addr = cfg.vars.dlog_ctrl >> 2; // Remove the lowest two bits
  if(addr > (DATALOGGER_MAX * (DATALOGGER_CHANNELS/4) + (DATALOGGER_CHANNELS-1))) addr = 0; // Simplified clamping

  char *dlog_ptr = ((char *)datalogger) + 8*addr;
  for(uint8_t i=0;i<8;i++) { // Move channels in 4 channel chunks
    cfg.vars.dlog_data[i] = *dlog_ptr;
    dlog_ptr++;
  }
#endif

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

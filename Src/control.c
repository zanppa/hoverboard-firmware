/*
 * control.c
 *
 *  Created on: May 6, 2018
 *      Author: tomvoc
 */

#include "control.h"
#include "config.h"
#include "defines.h"
#include "cfgbus.h"
#include "setup.h"
#include "svm.h"
#include "math.h"
#include "adc.h"

#include <stm32f1xx_hal_gpio.h>

// From adc.c
extern ADC_HandleTypeDef hadc1;
extern volatile adc_buf_t analog_meas;

#define LED_PERIOD (300)  //ms

// TODO: Move to setup.c and calculate there
const uint16_t motor_nominal_counts = (3<<12);		// timer ticks/sector change at rated speed

volatile uint16_t dc_voltage = 4096;		// Fixed point in p.u. TODO: Use ADC and convert to p.u.
volatile motor_state_t motor_state[2] = {0};

// Buzzer tone control
volatile uint16_t buzzer_tone = 0x0; // No tone. This defines the tone(s) to play. 1 bit is 1 ms.
volatile uint16_t buzzer_pattern = 0xFF00;	// Beep pattern, 1 bit is 64 ms (8b on 8b off is about 1 Hz beep)

// LED blinking pattern control
volatile uint16_t led_pattern = 0xFF00;		// Default blinking pattern

// Non-volatile variables that are ONLY used in the control timer interrupt
static uint16_t control_tick = 0;
static uint16_t speed_tick[2] = {0};

static uint8_t buzzer_tone_tick = 0;
static uint8_t pattern_tick = 0;

// Array to convert HALL sensor readings (order ABC) to sector number
// Note that index 0 and 7 are "guards" and should never happen when sensors work properly
static const uint8_t hall_to_sector[8] = { 0, 5, 1, 0, 3, 4, 2, 0 };



void init_controls(void)
{

}



void update_controls(void)
{

}


//called 64000000/64000 = 1000 times per second
void TIM3_IRQHandler(void)
{
  uint8_t sector_l, sector_r;
  uint8_t prev_sector_l, prev_sector_r;
  int16_t speed_l, speed_r;

#if defined(LEFT_MOTOR_BLDC) || defined(RIGHT_MOTOR_BLDC)
  int16_t pwm_diff;
#endif

  CTRL_TIM->SR = 0;

  // Debug: rotate the SVM reference
#ifdef LEFT_MOTOR_SVM
  if(motor_state[STATE_LEFT].ctrl.angle >= 4095) motor_state[STATE_LEFT].ctrl.angle = 0;
  else motor_state[STATE_LEFT].ctrl.angle += 1;
  motor_state[STATE_LEFT].ctrl.amplitude = 3000;
#endif

#ifdef RIGHT_MOTOR_SVM
  if(motor_state[STATE_RIGHT].ctrl.angle >= 4095) motor_state[STATE_RIGHT].ctrl.angle = 0;
  else motor_state[STATE_RIGHT].ctrl.angle += 1;
  motor_state[STATE_RIGHT].ctrl.amplitude = 3000;
#endif

  // Read HALL sensors
  // Determine rotor position (sector) based on HALL sensors
  sector_l =  (LEFT_HALL_PORT->IDR >> LEFT_HALL_LSB_PIN) & 0b111;
  sector_r =  (RIGHT_HALL_PORT->IDR >> RIGHT_HALL_LSB_PIN) & 0b111;
  sector_l = hall_to_sector[sector_l];
  sector_r = hall_to_sector[sector_r];

  prev_sector_l = motor_state[STATE_LEFT].act.sector;
  prev_sector_r = motor_state[STATE_RIGHT].act.sector;

  // Left motor speed
  if(sector_l != prev_sector_l) {
    speed_l = motor_nominal_counts / speed_tick[0];

    if(sector_l != ((prev_sector_l + 1) % 6)) speed_l = -speed_l;

    motor_state[STATE_LEFT].act.period = speed_tick[0];
    speed_tick[0] = 0;
  } else {
    speed_l = motor_state[STATE_LEFT].act.speed;
    if(speed_tick[0] < 4095) speed_tick[0]++;
    else {
      speed_l = 0;	// Easy way but response time is long
      motor_state[STATE_LEFT].act.period = 0xFFFF;
    }
  }

  // Right motor speed
  if(sector_r != prev_sector_r) {
    speed_r = motor_nominal_counts / speed_tick[1];

    if(sector_r != ((prev_sector_r + 1) % 6)) speed_r = -speed_r;

    motor_state[STATE_RIGHT].act.period = speed_tick[1];
    speed_tick[1] = 0;
  } else {
    speed_r = motor_state[STATE_RIGHT].act.speed;
    if(speed_tick[1] < 4095) speed_tick[1]++;
    else {
      speed_r = 0;	// Easy way but response time is long
      motor_state[STATE_RIGHT].act.period = 0xFFFF;
    }
  }


  // TODO: Move volatile(?) setpoints to local variables
#ifdef LEFT_MOTOR_BLDC
  // Torque (voltage) control of left motor in BLDC mode
  cfg.vars.setpoint_l = CLAMP(cfg.vars.setpoint_l, -cfg.vars.max_pwm_l, cfg.vars.max_pwm_l);

  pwm_diff = cfg.vars.setpoint_l - motor_state[STATE_LEFT].ctrl.amplitude;
  pwm_diff = CLAMP(pwm_diff, -cfg.vars.rate_limit, cfg.vars.rate_limit);

  motor_state[STATE_LEFT].ctrl.amplitude += pwm_diff;
#endif

#ifdef RIGHT_MOTOR_BLDC
  // Torque (voltage) control of left motor in BLDC mode
  cfg.vars.setpoint_r = CLAMP(cfg.vars.setpoint_r, -cfg.vars.max_pwm_r, cfg.vars.max_pwm_r);

  pwm_diff = cfg.vars.setpoint_r - motor_state[STATE_RIGHT].ctrl.amplitude;
  pwm_diff = CLAMP(pwm_diff, -cfg.vars.rate_limit, cfg.vars.rate_limit);

  motor_state[STATE_RIGHT].ctrl.amplitude += pwm_diff;
#endif


  // Update buzzer
  if(!cfg.vars.buzzer)
  {
	  HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, 0);
  }
  else
  {
    if(((buzzer_pattern >> pattern_tick) & 1) && ((buzzer_tone >> buzzer_tone_tick) & 1))
      HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, 1);
    else
      HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, 0);

    // Update tone
    buzzer_tone_tick = (buzzer_tone_tick + 1) & 0xF;
  }


#if 0 // Disabled while LED is used for interrupt timing etc
  // Update LED
  if((led_pattern >> pattern_tick) & 1)
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, 1);
  else
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, 0);
#endif

  // Update pattern
  if(!(control_tick ^ 0x40))	// Every 64 ms
    pattern_tick = (pattern_tick + 1) & 0xF;


  // Update motor state variables
  motor_state[STATE_LEFT].act.sector = sector_l;
  motor_state[STATE_RIGHT].act.sector = sector_r;
  motor_state[STATE_LEFT].act.speed = speed_l;
  motor_state[STATE_RIGHT].act.speed = speed_r;


  // Update config array
  cfg.vars.pos_l = sector_l;
  cfg.vars.pos_r = sector_r;
  cfg.vars.speed_l = speed_l;
  cfg.vars.speed_r = speed_r;


  // Copy ADC values to cfg array
  cfg.vars.vbat = analog_meas.v_battery;
  cfg.vars.vsw = analog_meas.v_switch;
  cfg.vars.temperature = analog_meas.temperature;
  cfg.vars.aref1 = analog_meas.analog_ref_1;
  cfg.vars.aref2 = analog_meas.analog_ref_2;
  cfg.vars.pwm_l = motor_state[STATE_LEFT].ctrl.amplitude;
  cfg.vars.pwm_r = motor_state[STATE_RIGHT].ctrl.amplitude;

  control_tick++;

  // Launch ADC1 so that at next call we
  // have fresh analog measurements
  hadc1.Instance->CR2 |= ADC_CR2_SWSTART;
}

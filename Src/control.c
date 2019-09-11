/*
 * control.c
 *
 *  Created on: May 6, 2018
 *      Author: tomvoc
 */

#include "control.h"
#include "defines.h"
#include "cfgbus.h"
#include "setup.h"
#include "svm.h"
#include "math.h"
#include "adc.h"

// From adc.c
extern ADC_HandleTypeDef hadc1;
extern volatile adc_buf_t analog_meas;

extern volatile uint16_t adc_raw_data[16];

#define LED_PERIOD (300)  //ms

// TODO: Move to setup.c and calculate there
const uint16_t motor_nominal_counts = (3<<12);		// timer ticks/sector change at rated speed

volatile uint16_t dc_voltage = 4096;		// Fixed point in p.u. TODO: Use ADC and convert to p.u.
volatile motor_state_t motor_state[2] = {0};
static volatile uint16_t speed_tick[2] = {0};

volatile uint32_t _ledTick=0;
volatile uint32_t _ctrlTick=0;

// Array to convert HALL sensor readings (order ABC) to sector number
// Note that index 0 and 7 are "guards" and should never happen when sensors work properly
static const uint8_t hall_to_sector[8] = { 0, 5, 1, 0, 3, 4, 2, 0 };


//call this from main thread. Responsible for turning off the LED
//LED is turned on in interrupt, so if LED flashes, we know main-loop
//and control interrupt are running properly.
void led_update(void)
{
  //turn off status LED if on for LED_PERIOD
  if(_ledTick >= LED_PERIOD)
  {
    HAL_GPIO_TogglePin(LED_PORT,LED_PIN);
   _ledTick = 0;
  }
}


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
  int32_t pwmDiff;

  CTRL_TIM->SR = 0;

  // Debug: rotate the SVM reference
  if(motor_state[STATE_LEFT].ctrl.angle >= 4095) motor_state[STATE_LEFT].ctrl.angle = 0;
  else motor_state[STATE_LEFT].ctrl.angle += 1;
  motor_state[STATE_LEFT].ctrl.amplitude = 3000;

  if(motor_state[STATE_RIGHT].ctrl.angle >= 4095) motor_state[STATE_RIGHT].ctrl.angle = 0;
  else motor_state[STATE_RIGHT].ctrl.angle += 1;
  motor_state[STATE_RIGHT].ctrl.amplitude = 3000;


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

    speed_tick[0] = 0;
  } else {
    speed_l = motor_state[STATE_LEFT].act.speed;
    if(speed_tick[0] < 4095) speed_tick[0]++;
  }

  // Right motor speed
  if(sector_r != prev_sector_r) {
    speed_r = motor_nominal_counts / speed_tick[1];

    if(sector_r != ((prev_sector_r + 1) % 6)) speed_r = -speed_r;

    speed_tick[1] = 0;
  } else {
    speed_r = motor_state[STATE_RIGHT].act.speed;
    if(speed_tick[1] < 4095) speed_tick[1]++;
  }


#ifdef LEFT_MOTOR_BLDC
  // Torque (voltage) control of left motor in BLDC mode
  cfg.vars.setpoint_l = CLAMP(cfg.vars.setpoint_l, -cfg.vars.max_pwm_l, cfg.vars.max_pwm_l);

  pwmDiff = (int32_t)cfg.vars.setpoint_l - motor_state[STATE_LEFT].ctrl.amplitude;
  pwmDiff = CLAMP(pwmDiff, -cfg.vars.rate_limit, cfg.vars.rate_limit);

  motor_state[STATE_LEFT].ctrl.amplitude += pwmDiff;
#endif

#ifdef RIGHT_MOTOR_BLDC
  // Torque (voltage) control of left motor in BLDC mode
  cfg.vars.setpoint_r = CLAMP(cfg.vars.setpoint_r, -cfg.vars.max_pwm_r, cfg.vars.max_pwm_r);

  pwmDiff = (int32_t)cfg.vars.setpoint_r - motor_state[STATE_RIGHT].ctrl.amplitude;
  pwmDiff = CLAMP(pwmDiff, -cfg.vars.rate_limit, cfg.vars.rate_limit);

  motor_state[STATE_RIGHT].ctrl.amplitude += pwmDiff;
#endif


  // Update buzzer
  if(cfg.vars.buzzer == 0)
  {
	  HAL_GPIO_WritePin(BUZZER_PORT,BUZZER_PIN,0);
  }
  else if( (_ctrlTick%cfg.vars.buzzer) == 0 && _ctrlTick%200 > 100  && _ctrlTick%3000 > 2000)
  {
	  HAL_GPIO_TogglePin(BUZZER_PORT, BUZZER_PIN);
  }

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
  //_old_tachoL = cfg.vars.tacho_l;
  //_old_tachoR = cfg.vars.tacho_r;

  // Copy ADC values to cfg array
  cfg.vars.vbat = analog_meas.v_battery;
  cfg.vars.vsw = analog_meas.v_switch;
  cfg.vars.temperature = analog_meas.temperature;
  cfg.vars.aref1 = analog_meas.analog_ref_1;
  cfg.vars.aref2 = analog_meas.analog_ref_2;

  //update state variables
  _ledTick++;
  _ctrlTick++;

  // Launch ADC1 so that at next call we
  // have fresh analog measurements
  hadc1.Instance->CR2 |= ADC_CR2_SWSTART;
}

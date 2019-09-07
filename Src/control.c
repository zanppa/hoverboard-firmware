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

#define LED_PERIOD (300)  //ms
volatile uint32_t _ledTick=0;
volatile uint32_t _ctrlTick=0;

volatile uint16_t _tachoL=0;
volatile uint16_t _tachoR=0;
volatile uint16_t _cntL;
volatile uint16_t _cntR;
volatile uint16_t _lastSpeedL = 0;
volatile uint16_t _lastSpeedR = 0;
volatile uint16_t _lastPosL = 0;
volatile uint16_t _lastPosR = 0;


// Debug: SVM references
extern volatile svm_ref_t svm_left;
extern volatile svm_ref_t svm_right;

// Array to convert HALL sensor readings (order ABC) to sector number
// Note that index 0 and 7 are "guards" and should never happen when sensors work properly
static const uint8_t hall_to_sector[8] = { 2, 5, 1, 0, 3, 4, 2, 2 };


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
  CTRL_TIM->SR = 0;

  // Debug: rotate the SVM reference
//  if(svm_debug_angle >= 4054) svm_debug_angle = 0;
//  else svm_debug_angle += 41;
  if(svm_left.angle >= 4095) svm_left.angle = 0;
  else svm_left.angle += 1;

  // Read HALL sensors
  //determine next position based on hall sensors
  uint8_t hall_l =  (LEFT_HALL_PORT->IDR >> LEFT_HALL_LSB_PIN) & 0b111;
  uint8_t hall_r =  (RIGHT_HALL_PORT->IDR >> RIGHT_HALL_LSB_PIN) & 0b111;

  cfg.vars.pos_r = hall_to_sector[hall_r];
  cfg.vars.pos_l = hall_to_sector[hall_l];

  //keep track of wheel movement
  if(_lastPosL != cfg.vars.pos_l)
    cfg.vars.tacho_l += (_lastPosL == (cfg.vars.pos_l + 1)%6) ? 1 : -1;

  if(_lastPosR != cfg.vars.pos_r)
    cfg.vars.tacho_r += (_lastPosR == (cfg.vars.pos_r + 1)%6) ? 1 : -1;

  _lastPosL = cfg.vars.pos_l;
  _lastPosR = cfg.vars.pos_r;


  //calculate motor speeds
  _cntL ++;

  if(cfg.vars.tacho_l != _tachoL)
  {
    cfg.vars.speed_l = (((int32_t)cfg.vars.tacho_l - (int32_t)_tachoL) * 4096) / _cntL;
    _cntL = 0;
  }
  else if(_cntL >= 100)
  {
    cfg.vars.speed_l = 0;
    _cntL = 0;
  }

  _cntR ++;

  if(cfg.vars.tacho_r != _tachoR)
  {
    cfg.vars.speed_r = (((int32_t)cfg.vars.tacho_r - (int32_t)_tachoR) * 4096) / _cntR;
    _cntR = 0;
  }
  else if(_cntR >= 100)
  {
    cfg.vars.speed_r = 0;
    _cntR = 0;
  }


  //update left motor PWM
  if(_lastSpeedL != cfg.vars.setpoint_l)
  {
    cfg.vars.setpoint_l = CLAMP(cfg.vars.setpoint_l, -1000, 1000);
    cfg.vars.setpoint_l = CLAMP(cfg.vars.setpoint_l, -cfg.vars.max_pwm_l, cfg.vars.max_pwm_l);

    int32_t pwmDiff = (int32_t)cfg.vars.setpoint_l - cfg.vars.pwm_l;
    pwmDiff = CLAMP(pwmDiff,-cfg.vars.rate_limit,cfg.vars.rate_limit);

    cfg.vars.pwm_l += pwmDiff;

    _lastSpeedL = cfg.vars.pwm_l;
  }


  //update right motor PWM
  if(_lastSpeedR != cfg.vars.setpoint_r)
  {
    cfg.vars.setpoint_r = CLAMP(cfg.vars.setpoint_r, -1000, 1000);
    cfg.vars.setpoint_r = CLAMP(cfg.vars.setpoint_r, -cfg.vars.max_pwm_r, cfg.vars.max_pwm_r);

    int32_t pwmDiff = (int32_t)cfg.vars.setpoint_r - cfg.vars.pwm_r;
    pwmDiff = CLAMP(pwmDiff,-cfg.vars.rate_limit,cfg.vars.rate_limit);

    cfg.vars.pwm_r += pwmDiff;

    _lastSpeedR = cfg.vars.pwm_r;
  }


  if(cfg.vars.buzzer == 0)
  {
	  HAL_GPIO_WritePin(BUZZER_PORT,BUZZER_PIN,0);
  }
  else if( (_ctrlTick%cfg.vars.buzzer) == 0 && _ctrlTick%200 > 100  && _ctrlTick%3000 > 2000)
  {
	  HAL_GPIO_TogglePin(BUZZER_PORT, BUZZER_PIN);
  }

  //update state variables
  _ledTick++;
  _ctrlTick++;
  _tachoL = cfg.vars.tacho_l;
  _tachoR = cfg.vars.tacho_r;
}

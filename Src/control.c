/*
 * control.c
 *
 *  Created on: May 6, 2018
 *      Original author: tomvoc
 *
 * Copyright (C) 2019 Lauri Peltonen
 */

#include "control.h"
#include "config.h"
#include "defines.h"
#include "cfgbus.h"
#include "setup.h"
#include "svm.h"
#include "math.h"
#include "adc.h"
#include "math.h"
#include "imeas.h"

#include <stm32f1xx_hal_gpio.h>

// From adc.c
extern ADC_HandleTypeDef hadc3;
extern volatile adc_buf_t analog_meas;

#define LED_PERIOD (300)  //ms

// TODO: Move to setup.c and calculate there
const uint16_t motor_nominal_counts = MOTOR_NOMINAL_PERIOD * (CONTROL_FREQ/1000.0);		// timer ticks/sector change at rated speed
const uint16_t sector_counts_to_svm = ANGLE_60DEG / (2*PWM_FREQ/CONTROL_FREQ);	// Control cycle runs at 1000 Hz while modulator twice in PWM_FREQ
const uint16_t motor_voltage_scale = MOTOR_VOLTS / (MOTOR_POLEPAIRS * MOTOR_SPEED);

const uint32_t adc_battery_to_pu = (FIXED_ONE / (2.45*MOTOR_VOLTS)) * (FIXED_ONE * ADC_BATTERY_VOLTS); // 2.45=sqrt(2)*sqrt(3)=phase RMS to main peak
const uint16_t adc_battery_filt_gain = FIXED_ONE / 10;		// Low-pass filter gain in fixed point for battery voltage

// -----------
// Torque control (FOC D and Q axis currents) parameters
#if defined(LEFT_MOTOR_FOC) || defined(RIGHT_MOTOR_FOC)
const uint16_t idq_filt_gain = FIXED_ONE / 300;	// Low pass filter id and iq

// P and I terms for d and q axis current regulators for FOC
// TODO: Ifdefs
const uint16_t kp_id = 0.6 * FIXED_ONE; //2000;
const uint16_t ki_id = 0.04 * FIXED_ONE; //2000; //1200;

const uint16_t kp_iq = 0.6 * FIXED_ONE;
const uint16_t ki_iq = 0.04 * FIXED_ONE; //1200;

// Id and Iq error integrals
#ifdef LEFT_MOTOR_FOC
static int16_t id_error_int_l = 0;
static int16_t iq_error_int_l = 0;
#endif
#ifdef RIGHT_MOTOR_FOC
static int16_t id_error_int_r = 0;
static int16_t iq_error_int_r = 0;
#endif
const int16_t idq_int_max = 30000;	// TODO: What is a sane value...?
const uint8_t int_divisor = 10;
#endif

// ---------
// Speed control parameters
static uint16_t kp_speed = 1.2 * FIXED_ONE;
static uint16_t ki_speed = 0.1 * FIXED_ONE;
static int16_t speed_error_int_l = 0;
static int16_t speed_error_int_r = 0;
static const uint16_t speed_int_max = 20000;	// TODO: What is a sane value?
static const uint16_t speed_int_divisor = 30;

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

// Array to convert HALL sensor readings (order CBA, MSB first) to sector number
// Note that index 0 and 7 are "guards" and should never happen when sensors work properly
//static const uint8_t hall_to_sector[8] = { 0, 5, 1, 0, 3, 4, 2, 0 };
//static const uint8_t hall_to_sector[8] = { 0, 0, 2, 1, 4, 5, 3, 0 }; // Mod 1
static const uint8_t hall_to_sector[8] = { 0, 2, 4, 3, 0, 1, 5, 0 };

// HALL sensors			Sector
// 		CBA	decimal		number (old)	new
// A 	001	1			0				2
// AB	011	3			1				3
// B	010	2			2				4
// BC	110	6			3				5
// C	100	4			4				0
// CA	101	5			5				1

// Current measurement
extern volatile i_meas_t i_meas;


// Read left hall sensors and return corresponding sector
uint8_t read_left_hall(void) {
  uint8_t sector;
  sector =  (LEFT_HALL_PORT->IDR >> LEFT_HALL_LSB_PIN) & 0b111;
  sector = hall_to_sector[sector];
  return sector;
}

// Read right hall sensors and return corresponding sector
uint8_t read_right_hall(void) {
  uint8_t sector;
  sector =  (RIGHT_HALL_PORT->IDR >> RIGHT_HALL_LSB_PIN) & 0b111;
  sector = hall_to_sector[sector];
  return sector;
}

// Initialize the control state, i.e. set starting positions
// for motors, set control variables, do precalculations and so one
void initialize_control_state(void) {
  uint8_t sector;

  // Initial rotor positions
  sector = read_left_hall();
  motor_state[STATE_LEFT].act.sector = sector;
  __disable_irq();
  motor_state[STATE_LEFT].act.angle = sector * ANGLE_60DEG;// + ANGLE_30DEG;	// Assume we're in the middle of a sector
  __enable_irq();

  sector = read_right_hall();
  motor_state[STATE_RIGHT].act.sector = sector;
  __disable_irq();
  motor_state[STATE_RIGHT].act.angle = sector * ANGLE_60DEG;// + ANGLE_30DEG;	// Assume middle of a sector
  __enable_irq();

}


// Controller internal variables, e.g. limited and ramped references
static int16_t setpoint_l_limit = 0;
static int16_t setpoint_r_limit = 0;
static int16_t pwm_l_ramp = 0;
static int16_t pwm_r_ramp = 0;
static uint16_t battery_voltage_filt = 0;	// Multiplied by 16 to increase filter accuracy, otherwise the error is something like 0.5 volts...

//called 64000000/64000 = 1000 times per second
// TODO: At 20 km/h HALL sector changes every 1 ms so this
// is definitely too slow / called too seldom
void TIM3_IRQHandler(void)
{
  uint8_t sector_l, sector_r;
  uint8_t prev_sector_l, prev_sector_r;
  int16_t speed_l, speed_r;
  uint16_t voltage_scale;
  int16_t speed_error;
  int16_t torque_ref;
  uint8_t ctrl_mode;


#if defined(LEFT_MOTOR_BLDC) || defined(RIGHT_MOTOR_BLDC)
  int16_t pwm_diff;
#endif

  CTRL_TIM->SR = 0;

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
    // Sector has changed
    speed_l = (FIXED_ONE * motor_nominal_counts) / speed_tick[0];
    uint16_t angle = sector_l * ANGLE_60DEG - ANGLE_30DEG;	// Edge of a sector

    // Calculate min and max angles in this sector
    // To prevent the modulator angle estimation from exceeding the sector
    motor_state[STATE_LEFT].ctrl.angle_min = angle;
    motor_state[STATE_LEFT].ctrl.angle_max = angle + ANGLE_60DEG;

    if(sector_l != ((prev_sector_l + 1) % 6)) {
      speed_l = -speed_l;
      angle += ANGLE_60DEG;
    }

#ifdef SVM_HALL_UPDATE
    __disable_irq();	// Angle is also updated by modulator
    motor_state[STATE_LEFT].act.angle = angle;
    __enable_irq();
    motor_state[STATE_LEFT].ctrl.speed = sector_counts_to_svm / speed_tick[0];
#endif

    motor_state[STATE_LEFT].act.period = speed_tick[0];
    speed_tick[0] = 0;
  } else {
    // Still inside the current sector
    speed_l = motor_state[STATE_LEFT].act.speed;

    if(speed_tick[0] > motor_state[STATE_LEFT].act.period) {
      // We should have passed the sector change, going slower than expected
      // so update speed
      speed_l = ((speed_l < 0) ? -1 : 1) * (FIXED_ONE * motor_nominal_counts) / speed_tick[0];
    }

    if(speed_tick[0] < 1000) speed_tick[0]++;	// If no sector change in 1 s assume stall
    else {
      speed_l = 0;	// Easy way but response time is long
      motor_state[STATE_LEFT].act.period = 0xFFFF;
    }
  }


  // Right motor speed
  if(sector_r != prev_sector_r) {
    // Sector has changed
    speed_r = (FIXED_ONE * motor_nominal_counts) / speed_tick[1];
    uint16_t angle = sector_r * ANGLE_60DEG - ANGLE_30DEG;	// Edge of a sector

    motor_state[STATE_RIGHT].ctrl.angle_min = angle;
    motor_state[STATE_RIGHT].ctrl.angle_max = angle + ANGLE_60DEG;

    if(sector_r != ((prev_sector_r + 1) % 6)) {
      speed_r = -speed_r;
      angle += ANGLE_60DEG;
    }

#ifdef SVM_HALL_UPDATE
    __disable_irq();	// Angle is also updated by modulator
    motor_state[STATE_RIGHT].act.angle = angle;
    __enable_irq();
    motor_state[STATE_RIGHT].ctrl.speed = sector_counts_to_svm / speed_tick[1];
#endif

    motor_state[STATE_RIGHT].act.period = speed_tick[1];
    speed_tick[1] = 0;
  } else {
    // Still inside the current sector
    speed_r = motor_state[STATE_RIGHT].act.speed;

    if(speed_tick[1] > motor_state[STATE_RIGHT].act.period) {
      // We should have passed the sector change, going slower than expected
      // so update speed
      speed_r = ((speed_r < 0) ? -1 : 1) * (FIXED_ONE * motor_nominal_counts) / speed_tick[1];
    }

    if(speed_tick[1] < 1000) speed_tick[1]++; // if no sector change in 1 s assume stall
    else {
      speed_r = 0;	// Easy way but response time is long
      motor_state[STATE_RIGHT].act.period = 0xFFFF;
    }
  }


  // Analog measurements (battery voltage, to be used in modulator)
  analog_meas.v_battery += ADC_BATTERY_OFFSET;
  //battery_voltage_filt = fx_mulu(analog_meas.v_battery << 4, adc_battery_filt_gain) + fx_mulu(battery_voltage_filt, FIXED_ONE-adc_battery_filt);
  battery_voltage_filt = FILTERU(analog_meas.v_battery << 4, battery_voltage_filt, adc_battery_filt_gain);
  voltage_scale = fx_mulu((battery_voltage_filt >> 4), adc_battery_to_pu);

  // Reference scaling so that 1 (4096) results in 1 (motor nominal voltage) always
  // So we scale all references by battery_voltage / nominal voltage
  voltage_scale = fx_divu(FIXED_ONE, voltage_scale);


  // --------------
  // Left motor

  // Speed control loop for left motor
  ctrl_mode = motor_state[STATE_LEFT].ref.control_mode ;

  if(ctrl_mode == CONTROL_SPEED) {
    speed_error = motor_state[STATE_LEFT].ref.value - speed_l;
    speed_error_int_l = LIMIT(speed_error_int_l + (speed_error / speed_int_divisor), speed_int_max);
    torque_ref = speed_error + fx_mul(speed_error_int_l, ki_speed);
    torque_ref = fx_mul(torque_ref, kp_speed);
  } else if(ctrl_mode == CONTROL_TORQUE) {
    torque_ref = motor_state[STATE_LEFT].ref.value;
  } else {
    torque_ref = 0;
  }


  // Debug: rotate the SVM reference
#ifdef LEFT_MOTOR_SVM

#ifdef LEFT_MOTOR_FOC
  // Get the interpolated position and measured currents from exactly same time instants
  __disable_irq();
  int16_t ia = i_meas.i_lA;
  int16_t ib = i_meas.i_lB;
  uint16_t angle = motor_state[STATE_LEFT].act.angle;	// Estimated rotor position
  __enable_irq();

  // Transform to rotor coordinate frame
  int16_t ialpha, ibeta;
  int16_t id, iq;
  clarke(ia, ib, &ialpha, &ibeta);
  park(ialpha, ibeta, angle, &id, &iq);

  int16_t id_error = 0 - id;	// TODO: Add id reference (from field weakening)
  //int16_t iq_error = cfg.vars.setpoint_l - iq;
  int16_t iq_error = torque_ref - iq;

  // Run the PI controllers
  // First for D axis current which sets the angle advance
  id_error_int_l = LIMIT(id_error_int_l + (id_error / int_divisor), idq_int_max);
  int16_t angle_advance = id_error + fx_mul(id_error_int_l, ki_id);
  angle_advance = fx_mul(angle_advance, kp_id) * 8;	// From 12-bit fixed point to 16-bit angle => 1 = 4096 = one full rotation

  // Then for Q axis current which sets the reference amplitude
  iq_error_int_l = LIMIT(iq_error_int_l + iq_error, idq_int_max);
  int16_t ref_amplitude = iq_error + fx_mul(iq_error_int_l, ki_iq);
  ref_amplitude = fx_mul(ref_amplitude, kp_iq);
  ref_amplitude = CLAMP(ref_amplitude, 0, cfg.vars.max_pwm_l);

  // Apply DC voltage scaling
  ref_amplitude = fx_mul(ref_amplitude, voltage_scale);

  // Apply references
  motor_state[STATE_LEFT].ctrl.amplitude = (uint16_t)ref_amplitude;
  motor_state[STATE_LEFT].ctrl.angle = (uint16_t)angle_advance + ANGLE_90DEG;	// Should start with 90 degree phase shift
  // TODO: Apply 90 degrees positive or negative depending on reference polarity

#else // LEFT_MOTOR_FOC
  // TODO: U/f control for SVM without FOC?
#endif // !LEFT_MOTOR_FOC

#endif // LEFT_MOTOR_SVM


  // TODO: Move volatile(?) setpoints to local variables
#ifdef LEFT_MOTOR_BLDC
  // Torque (voltage) control of left motor in BLDC mode
  setpoint_l_limit = CLAMP(torque_ref, -cfg.vars.max_pwm_l, cfg.vars.max_pwm_l);

  pwm_diff = setpoint_l_limit - pwm_l_ramp;
  pwm_diff = CLAMP(pwm_diff, -cfg.vars.rate_limit, cfg.vars.rate_limit);

  pwm_l_ramp += pwm_diff;

  //motor_state[STATE_LEFT].ctrl.amplitude = fx_mul(pwm_l_ramp, voltage_scale);
  motor_state[STATE_LEFT].ctrl.amplitude = pwm_l_ramp;
#endif // LEFT_MOTOR_BLDC



  // ------------
  // Right motor

  ctrl_mode = motor_state[STATE_RIGHT].ref.control_mode ;

  if(ctrl_mode == CONTROL_SPEED) {
    // Speed control loop for right motor
    speed_error = motor_state[STATE_RIGHT].ref.value - speed_r;
    speed_error_int_r = LIMIT(speed_error_int_r + (speed_error / speed_int_divisor), speed_int_max);
    torque_ref = speed_error + fx_mul(speed_error_int_r, ki_speed);
    torque_ref = fx_mul(torque_ref, kp_speed);

    cfg.vars.t_req_r = torque_ref;

  } else if(ctrl_mode == CONTROL_TORQUE) {
    torque_ref = motor_state[STATE_RIGHT].ref.value;
  } else {
    torque_ref = 0;
  }

#ifdef RIGHT_MOTOR_SVM

#ifdef RIGHT_MOTOR_FOC
  // Get the interpolated position and measured currents from exactly same time instants
  __disable_irq();
  int16_t ib = i_meas.i_rB;
  int16_t ic = i_meas.i_rC;
  uint16_t angle = motor_state[STATE_RIGHT].act.angle;	// Estimated rotor position
  __enable_irq();

  int16_t ia = -ib - ic;		// For simplicity, we use ia and ib in the calculation

  // Transform to rotor coordinate frame
  int16_t ialpha, ibeta;
  int16_t id, iq;
  clarke(ia, ib, &ialpha, &ibeta);
  park(ialpha, ibeta, angle, &id, &iq);

  id = FILTER(id, cfg.vars.r_id, idq_filt_gain);
  iq = FILTER(iq, cfg.vars.r_iq, idq_filt_gain);

  // Debug: Store id and iq to config bus
  cfg.vars.r_id = id;
  cfg.vars.r_iq = iq;

  int16_t id_error = id;    // TODO: Add id reference (from field weakening)
  //int16_t iq_error = cfg.vars.setpoint_r - iq;
  int16_t iq_error = torque_ref - iq;

  // Run the PI controllers
  // First for D axis current which sets the angle advance
  id_error_int_r = LIMIT(id_error_int_r + (id_error/int_divisor), idq_int_max);
  int16_t angle_advance = id_error + fx_mul(id_error_int_r, ki_id);
  angle_advance = fx_mul(angle_advance, kp_id) * 8;

  // Invert phase advance if speed is reverse
  //if(speed_r < 0) angle_advance = -angle_advance;
  // TODO: Should probably be the reference, not actual...

  // Then for Q axis current which sets the reference amplitude
  iq_error_int_r = LIMIT(iq_error_int_r + iq_error, idq_int_max);
  int16_t ref_amplitude = iq_error + fx_mul(iq_error_int_r, ki_iq);
  ref_amplitude = fx_mul(ref_amplitude, kp_iq);
  ref_amplitude = CLAMP(ref_amplitude, 0, cfg.vars.max_pwm_r);

  // Apply DC voltage scaling
  ref_amplitude = fx_mul(ref_amplitude, voltage_scale);

  // Apply references
  motor_state[STATE_RIGHT].ctrl.amplitude = (uint16_t)ref_amplitude;
  motor_state[STATE_RIGHT].ctrl.angle = (uint16_t)angle_advance + ANGLE_90DEG;

#else // RIGHT_MOTOR_FOC
  // TODO: U/f control for SVM without FOC?
#endif // !RIGHT_MOTOR_FOC

#endif // RIGHT_MOTOR_SVM


#ifdef RIGHT_MOTOR_BLDC
  // Torque (voltage) control of left motor in BLDC mode
  setpoint_r_limit = CLAMP(torque_ref, -cfg.vars.max_pwm_r, cfg.vars.max_pwm_r);

  pwm_diff = setpoint_r_limit - pwm_r_ramp;
  pwm_diff = CLAMP(pwm_diff, -cfg.vars.rate_limit, cfg.vars.rate_limit);

  pwm_r_ramp += pwm_diff;

  //motor_state[STATE_RIGHT].ctrl.amplitude = fx_mul(pwm_r_ramp, voltage_scale);
  motor_state[STATE_RIGHT].ctrl.amplitude = pwm_r_ramp;
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
  motor_state[STATE_LEFT].act.current[0] = i_meas.i_lA;
  motor_state[STATE_LEFT].act.current[1] = i_meas.i_lB;
  motor_state[STATE_LEFT].act.current[2] = -i_meas.i_lA - i_meas.i_lB;
  motor_state[STATE_RIGHT].act.current[0] = -i_meas.i_lB - i_meas.i_rC;
  motor_state[STATE_RIGHT].act.current[1] = i_meas.i_rB;
  motor_state[STATE_RIGHT].act.current[2] = i_meas.i_rC;

  // Update config array
  cfg.vars.pos_l = sector_l;
  cfg.vars.pos_r = sector_r;
  cfg.vars.speed_l = speed_l;
  cfg.vars.speed_r = speed_r;
  cfg.vars.r_angle = motor_state[STATE_RIGHT].act.angle;


  // Copy ADC values to cfg array
  // TODO: Move to main?
  cfg.vars.vbat = analog_meas.v_battery;
  cfg.vars.vsw = analog_meas.v_switch;
  cfg.vars.temperature = analog_meas.temperature;
  cfg.vars.aref1 = analog_meas.analog_ref_1;
  cfg.vars.aref2 = analog_meas.analog_ref_2;
  cfg.vars.pwm_l = motor_state[STATE_LEFT].ctrl.amplitude;
  cfg.vars.pwm_r = motor_state[STATE_RIGHT].ctrl.amplitude;
  cfg.vars.l_angle_adv = motor_state[STATE_LEFT].ctrl.angle;
  cfg.vars.r_angle_adv = motor_state[STATE_RIGHT].ctrl.angle;
  cfg.vars.ref_scale = voltage_scale;

  control_tick++;

  // Launch ADC3 so that at next call we
  // have fresh analog measurements
  hadc3.Instance->CR2 |= ADC_CR2_SWSTART;
}

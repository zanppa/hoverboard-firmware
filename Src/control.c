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
#include "uartscope.h"
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
const uint16_t ki_id = 0.01 * FIXED_ONE; //2000; //1200;

const uint16_t kp_iq = 0.6 * FIXED_ONE;
const uint16_t ki_iq = 0.08 * FIXED_ONE; //1200;

// Id and Iq error integrals
#ifdef LEFT_MOTOR_FOC
static int16_t id_error_int_l = 0;
static int16_t iq_error_int_l = 0;
#endif // LEFT_MOTOR_FOC
#ifdef RIGHT_MOTOR_FOC
static int16_t id_error_int_r = 0;
static int16_t iq_error_int_r = 0;
#endif // RIGHT_MOTOR_FOC
const int16_t idq_int_max = 30000;	// TODO: What is a sane value...?
const uint8_t int_divisor = 10;
#endif // LEFT_MOTOR_FOC || RIGHT_MOTOR_FOC

// ---------
// Speed control parameters
static uint16_t kp_speed = 1.2 * FIXED_ONE;
static uint16_t ki_speed = 0.1 * FIXED_ONE;
static int16_t speed_error_int_l = 0;
static int16_t speed_error_int_r = 0;
static const uint16_t speed_int_max = 20000;	// TODO: What is a sane value?
static const uint16_t speed_int_divisor = 30;

uint16_t battery_volt_pu = 0;

// Warning/trip values in fixed point per-unit values
volatile uint8_t fault_bits = 0;		// Fault type, 0=no fault
const uint16_t ov_warn_pu = FIXED_ONE * OVERVOLTAGE_WARN / (2.45*MOTOR_VOLTS);
const uint16_t ov_trip_pu = FIXED_ONE * OVERVOLTAGE_TRIP / (2.45*MOTOR_VOLTS);
const uint16_t uv_warn_pu = FIXED_ONE * UNDERVOLTAGE_WARN / (2.45*MOTOR_VOLTS);
const uint16_t uv_trip_pu = FIXED_ONE * UNDERVOLTAGE_TRIP / (2.45*MOTOR_VOLTS);


volatile uint8_t status_bits = 0;	// Status bits

volatile motor_state_t motor_state[2] = {0};

// Current measurement
extern volatile i_meas_t i_meas;

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


// Fault (generate break) to selected motors
// bit 0 = left (TIM8) bit 1 = right (TIM1)
// This can be triggered by e.g. overcurrent event
void do_fault(uint8_t sides) {
  if(sides & 0x01) {
    LEFT_TIM->EGR |= TIM_EGR_BG;
  }

  if(sides & 0x02) {
    RIGHT_TIM->EGR |= TIM_EGR_BG;
  }
}

// Clear fault (break) and enable motors
// bit 0 = left (TIM8) bit 1 = right (TIM1)
// TODO: Does not make sense to not reset both...
void clear_fault(uint8_t sides) {
  fault_bits = 0;

  if(sides & 0x01) {
    LEFT_TIM->EGR &= ~TIM_EGR_BG;	// Clear break
    LEFT_TIM->BDTR |= TIM_BDTR_MOE;	// Enable motor
  }

  if(sides & 0x02) {
    RIGHT_TIM->EGR &= ~TIM_EGR_BG;	// Clear break
    RIGHT_TIM->BDTR |= TIM_BDTR_MOE;	// Enable motor
  }
}

// Check if there has been a short circuit
// This is indicated by the break interrupt flag in
// timers
// TODO: Does this go up also if BG bit is set in EGR?
void check_sc() {
  if((LEFT_TIM->SR & TIM_SR_BIF) || (RIGHT_TIM->SR & TIM_SR_BIF)) {
    fault_bits |= FAULT_SHORT;
    // TODO: Fault the other side as well?
  }
}


// Disable one or both motor PWM outputs
// bit 0 = left, bit 1 = right
void disable_motors(uint8_t sides) {
  if(sides & 0x01) {
    LEFT_TIM->BDTR &= ~TIM_BDTR_MOE;	// Disable output
    motor_state[STATE_LEFT].ctrl.enable = 0;
  }

  if(sides & 0x02) {
    RIGHT_TIM->BDTR &= ~TIM_BDTR_MOE;	// Enable output
    motor_state[STATE_RIGHT].ctrl.enable = 0;
  }
}

// Enable one or both motor PWM outputs
// bit 0 = left, bit 1 = right
void enable_motors(uint8_t sides) {
  if(sides & 0x01) {
    LEFT_TIM->BDTR |= TIM_BDTR_MOE;	// Disable output
    motor_state[STATE_LEFT].ctrl.enable = 1;
  }

  if(sides & 0x02) {
    RIGHT_TIM->BDTR |= TIM_BDTR_MOE;	// Enable output
    motor_state[STATE_RIGHT].ctrl.enable = 1;
  }
}


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
#ifdef LEFT_MOTOR_BLDC
static int16_t setpoint_l_limit = 0;
static int16_t pwm_l_ramp = 0;
#endif
#ifdef RIGHT_MOTOR_BLDC
static int16_t setpoint_r_limit = 0;
static int16_t pwm_r_ramp = 0;
#endif

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
  int16_t ia_l, ib_l, ic_l;
  int16_t ia_r, ib_r, ic_r;
  uint16_t angle_l, angle_r;
  int16_t ref_l, ref_r;
  int16_t speed_error;
  int16_t torque_ref;
  uint8_t ctrl_mode;

#if defined(LEFT_MOTOR_FOC) || defined(RIGHT_MOTOR_FOC)
  int16_t ia, ib;
  uint16_t angle;
  int16_t ialpha, ibeta;
  int16_t id, iq;
  int16_t id_error, iq_error;
  int16_t angle_advance, ref_amplitude;
  int8_t ref_sign;
#endif

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

  // Current measurement and overcurrent trips
  // Left motor phase currents and position
  __disable_irq();
  ia_l = i_meas.i_lA;
  ib_l = i_meas.i_lB;
  angle_l = motor_state[STATE_LEFT].act.angle;

  // Right motor phase currents and position
  ib_r = i_meas.i_rB;
  ic_r = i_meas.i_rC;
  angle_r = motor_state[STATE_RIGHT].act.angle;
  __enable_irq();

  ia_r = -ib_r - ic_r;
  ic_l = -ia_l - ib_l;

  // Check if currents exceed overcurrent limits
  // and trip one (TODO: or both?) motors
  if(ia_l > OVERCURRENT_TRIP || -ia_l < -OVERCURRENT_TRIP ||
     ib_l > OVERCURRENT_TRIP || -ib_l < -OVERCURRENT_TRIP ||
     ic_l > OVERCURRENT_TRIP || -ic_l < -OVERCURRENT_TRIP) {
    do_fault(0x01);	// Trip left motor
    fault_bits |= FAULT_OVERCURRENT;
    // TODO: Buzzer + led
  }

  if(ia_r > OVERCURRENT_TRIP || -ia_r < -OVERCURRENT_TRIP ||
     ib_r > OVERCURRENT_TRIP || -ib_r < -OVERCURRENT_TRIP ||
     ic_r > OVERCURRENT_TRIP || -ic_r < -OVERCURRENT_TRIP) {
    do_fault(0x02);	// Trip right motor
    fault_bits |= FAULT_OVERCURRENT;
    // TODO: Buzzer + led
  }


  // Analog measurements (battery voltage, to be used in modulator)
  analog_meas.v_battery += ADC_BATTERY_OFFSET;
  //battery_voltage_filt = fx_mulu(analog_meas.v_battery << 4, adc_battery_filt_gain) + fx_mulu(battery_voltage_filt, FIXED_ONE-adc_battery_filt);
  battery_voltage_filt = FILTERU(analog_meas.v_battery << 4, battery_voltage_filt, adc_battery_filt_gain);
  battery_volt_pu = fx_mulu((battery_voltage_filt >> 4), adc_battery_to_pu);

  // Reference scaling so that 1 (4096) results in 1 (motor nominal voltage) always
  // So we scale all references by battery_voltage / nominal voltage
  voltage_scale = fx_divu(FIXED_ONE, battery_volt_pu);


  // Check voltage limits
  // Only trip if everything is ready, e.g. voltage has been filtered long enough etc.
  if(battery_volt_pu > ov_trip_pu && (status_bits & STATUS_READY)) {
    do_fault(0x01 | 0x02);	// Trip both motors
    fault_bits |= FAULT_OVERVOLTAGE;
    // TODO: Buzzer tone & blink led
  } else if(battery_volt_pu > ov_warn_pu) {
    status_bits |= STATUS_OVERVOLTAGE_WARN;
    // TODO: Buzzer & blink
  } else if(battery_volt_pu < uv_trip_pu && (status_bits & STATUS_READY)) {
    do_fault(0x01 | 0x02);
    fault_bits |= FAULT_UNDERVOLTAGE;
  } else if(battery_volt_pu < uv_warn_pu) {
    status_bits |= STATUS_UNDERVOLTAGE_WARN;
    // TODO: Buzzer & blink
  } else { // Remove alarm bits
    status_bits &= ~(STATUS_OVERVOLTAGE_WARN | STATUS_UNDERVOLTAGE_WARN);

    // Check that filtered DC link voltage is high enough and indicate ready state
    if(!(status_bits & STATUS_READY) && battery_volt_pu > uv_warn_pu)
      status_bits |= STATUS_READY;
  }


#if defined(REFERENCE_MODBUS)
  motor_state[STATE_LEFT].ref.value = cfg.vars.setpoint_l;
  motor_state[STATE_RIGHT].ref.value = cfg.vars.setpoint_r;
  //  ref_l = cfg.vars.spdref_l;
  // ref_r = cfg.vars.spdref_r;
#elif defined(REFERENCE_ADC)
#if defined(REFERENCE_ADC_DIFF)
  // This is simple and results in -8192 ... 8191 range
  // TODO: Add scaling and offset and deadband?
  ref_l = analog_meas.analog_ref_1 + analog_meas.analog_ref_2;
  ref_r = analog_meas.analog_ref_2 - analog_meas.analog_ref_2;
  ref_l = (ref_l - 4096) * 2;
  ref_r = (ref_r - 4096) * 2;
  motor_state[STATE_LEFT].ref.value = ref_l;
  motor_state[STATE_RIGHT].ref.value = ref_r;
#else // REFERENCE_ADC_DIFF
  // ADC output is 0...4095, scale it to -4096 ... 4095
  // TODO: Add some configuration (offset, gain, deadband) to these?
  ref_l = (analog_meas.analog_ref_1 - 2048) * 2;
  ref_r = (analog_meas.analog_ref_2 - 2048) * 2;
  motor_state[STATE_LEFT].ref.value = ref_l;
  motor_state[STATE_RIGHT].ref.value = ref_r;
#endif // REFERENCE_ADC_DIFF
#else // REFERENCE_ADC
  motor_state[STATE_LEFT].ref.value = 0;
  motor_state[STATE_RIGHT].ref.value = 0;
#endif // REFERENCE_ADC


  // --------------
  // Left motor

  // Speed control loop for left motor
  ctrl_mode = motor_state[STATE_LEFT].ref.control_mode;

  if(ctrl_mode == CONTROL_SPEED) {
#if defined(LEFT_MOTOR_SVM) && !defined(LEFT_MOTOR_FOC)
    // SVM --> F/U control so reference is the speed reference
#else
    // FOC and BLCD --> run PI controller
    speed_error = motor_state[STATE_LEFT].ref.value - speed_l;
    speed_error_int_l = LIMIT(speed_error_int_l + (speed_error / speed_int_divisor), speed_int_max);
    torque_ref = speed_error + fx_mul(speed_error_int_l, ki_speed);
    torque_ref = fx_mul(torque_ref, kp_speed);
#endif
  } else if(ctrl_mode == CONTROL_TORQUE) {
    torque_ref = motor_state[STATE_LEFT].ref.value;
  } else {
    torque_ref = 0;
  }


#ifdef LEFT_MOTOR_SVM
#ifdef LEFT_MOTOR_FOC
  // Get the interpolated position and measured currents from exactly same time instants
  ia = ia_l;
  ib = ib_l;
  angle = angle_l;

  // Transform to rotor coordinate frame
  clarke(ia, ib, &ialpha, &ibeta);
  park(ialpha, ibeta, angle, &id, &iq);

  id_error = id;	// TODO: Add id reference (from field weakening)
  //int16_t iq_error = cfg.vars.setpoint_l - iq;
  iq_error = torque_ref - iq;

  // Run the PI controllers
  // First for D axis current which sets the angle advance
  id_error_int_l = LIMIT(id_error_int_l + (id_error / int_divisor), idq_int_max);
  angle_advance = id_error + fx_mul(id_error_int_l, ki_id);
  angle_advance = fx_mul(angle_advance, kp_id) * 8;	// From 12-bit fixed point to 16-bit angle => 1 = 4096 = one full rotation

  // Then for Q axis current which sets the reference amplitude
  iq_error_int_l = LIMIT(iq_error_int_l + iq_error, idq_int_max);
  ref_amplitude = iq_error + fx_mul(iq_error_int_l, ki_iq);
  ref_amplitude = fx_mul(ref_amplitude, kp_iq);

  ref_sign = SIGN(ref_amplitude);

  // Apply DC voltage scaling
  ref_amplitude = fx_mul(ref_amplitude, voltage_scale);

  // Apply limiter
  ref_amplitude = CLAMP(ref_amplitude, 0, cfg.vars.max_pwm_l);

  // Apply references
  motor_state[STATE_LEFT].ctrl.amplitude = (uint16_t)ref_amplitude;
  motor_state[STATE_LEFT].ctrl.angle = (uint16_t)angle_advance + (ref_sign * ANGLE_90DEG);	// Should start with 90 degree phase shift
  // TODO: Angle advance polarity should change depending on speed direction

#else // LEFT_MOTOR_FOC
  // TODO: U/f control for SVM without FOC?
  if(motor_state[STATE_LEFT].ref.control_mode == CONTROL_SPEED) {
#if defined(IR_MINIMUM_VOLTAGE)
    motor_state[STATE_LEFT].ctrl.amplitude = MAX(ABS(motor_state[STATE_LEFT].ref.value), IR_MINIMUM_VOLTAGE);
#else
    motor_state[STATE_LEFT].ctrl.amplitude = ABS(motor_state[STATE_LEFT].ref.value);
#endif // IR_MINIMUM_VOLTAGE
    motor_state[STATE_LEFT].ctrl.speed = motor_state[STATE_LEFT].ref.value;
  } else if(motor_state[STATE_LEFT].ref.control_mode == CONTROL_ANGLE) {
    motor_state[STATE_LEFT].ctrl.speed = 0;
    motor_state[STATE_LEFT].ctrl.angle = motor_state[STATE_LEFT].ref.value;  // DEBUG
  }
#endif // !LEFT_MOTOR_FOC

#endif // LEFT_MOTOR_SVM

  // TODO: Move volatile(?) setpoints to local variables
#ifdef LEFT_MOTOR_BLDC
  // Torque (voltage) control of left motor in BLDC mode

  // Apply ramp
  pwm_diff = torque_ref - pwm_l_ramp;
  pwm_diff = LIMIT(pwm_diff, cfg.vars.rate_limit);
  pwm_l_ramp += pwm_diff;

  // Scale ramped reference according to PWM period and DC voltage
  torque_ref = fx_mul(pwm_l_ramp, PWM_PERIOD);
  torque_ref = fx_mul(torque_ref, voltage_scale);

  // Limit pwm value
  setpoint_l_limit = LIMIT(torque_ref, cfg.vars.max_pwm_l);

  motor_state[STATE_LEFT].ctrl.amplitude = setpoint_l_limit;
#endif // LEFT_MOTOR_BLDC



  // ------------
  // Right motor

  ctrl_mode = motor_state[STATE_RIGHT].ref.control_mode;

  if(ctrl_mode == CONTROL_SPEED) {
    // Speed control loop for right motor
#if defined(RIGHT_MOTOR_SVM) && !defined(RIGHT_MOTOR_FOC)
    // SVM --> F/U control so reference is the speed reference
#else
    // FOC and BLCD --> run PI controller
    speed_error = motor_state[STATE_RIGHT].ref.value - speed_r;
    speed_error_int_r = LIMIT(speed_error_int_r + (speed_error / speed_int_divisor), speed_int_max);
    torque_ref = speed_error + fx_mul(speed_error_int_r, ki_speed);
    torque_ref = fx_mul(torque_ref, kp_speed);

    cfg.vars.t_req_r = torque_ref;
#endif
  } else if(ctrl_mode == CONTROL_TORQUE) {
    torque_ref = motor_state[STATE_RIGHT].ref.value;
  } else {
    torque_ref = 0;
  }

#ifdef RIGHT_MOTOR_SVM
#ifdef RIGHT_MOTOR_FOC
  // Get the interpolated position and measured currents from exactly same time instants
  ia = ia_r;		// For simplicity, we use ia and ib in the calculation
  ib = ib_r;
  angle = angle_r;	// Estimated rotor position

  // Transform to rotor coordinate frame
  clarke(ia, ib, &ialpha, &ibeta);
  park(ialpha, ibeta, angle, &id, &iq);

  id = FILTER(id, cfg.vars.r_id, idq_filt_gain);
  iq = FILTER(iq, cfg.vars.r_iq, idq_filt_gain);

  // Debug: Store id and iq to config bus
  cfg.vars.r_id = id;
  cfg.vars.r_iq = iq;

  id_error = id;    // TODO: Add id reference (from field weakening)
  //int16_t iq_error = cfg.vars.setpoint_r - iq;
  iq_error = torque_ref - iq;

  // Run the PI controllers
  // First for D axis current which sets the angle advance
  id_error_int_r = LIMIT(id_error_int_r + (id_error/int_divisor), idq_int_max);
  angle_advance = id_error + fx_mul(id_error_int_r, ki_id);
  angle_advance = fx_mul(angle_advance, kp_id) * 8;

  // Invert phase advance if speed is reverse
  //if(speed_r < 0) angle_advance = -angle_advance;
  // TODO: Should probably be the reference, not actual...

  // Then for Q axis current which sets the reference amplitude
  iq_error_int_r = LIMIT(iq_error_int_r + iq_error, idq_int_max);
  ref_amplitude = iq_error + fx_mul(iq_error_int_r, ki_iq);
  ref_amplitude = fx_mul(ref_amplitude, kp_iq);

  ref_sign = SIGN(ref_amplitude);

  // Apply DC voltage scaling
  ref_amplitude = fx_mul(ref_amplitude, voltage_scale);

  // Apply limiter
  ref_amplitude = CLAMP(ref_amplitude, 0, cfg.vars.max_pwm_r);

  // Apply references
  motor_state[STATE_RIGHT].ctrl.amplitude = (uint16_t)ref_amplitude;
  motor_state[STATE_RIGHT].ctrl.angle = (uint16_t)angle_advance + (ref_sign * ANGLE_90DEG);
  // TODO: Angle advance polarity should change depending on speed direction

#else // RIGHT_MOTOR_FOC
  // TODO: U/f control for SVM without FOC?

  if(motor_state[STATE_RIGHT].ref.control_mode == CONTROL_SPEED) {
#if defined(IR_MINIMUM_VOLTAGE)
    motor_state[STATE_RIGHT].ctrl.amplitude = MAX(ABS(motor_state[STATE_RIGHT].ref.value), IR_MINIMUM_VOLTAGE);
#else
    motor_state[STATE_RIGHT].ctrl.amplitude = ABS(motor_state[STATE_RIGHT].ref.value);
#endif // IR_MNINMUM_VOLTAGE
    motor_state[STATE_RIGHT].ctrl.speed = motor_state[STATE_RIGHT].ref.value;
  } else if(motor_state[STATE_RIGHT].ref.control_mode == CONTROL_ANGLE) {
    motor_state[STATE_RIGHT].ctrl.speed = 0;
    motor_state[STATE_RIGHT].ctrl.angle = motor_state[STATE_RIGHT].ref.value;  // DEBUG
  }
#endif // !RIGHT_MOTOR_FOC

#endif // RIGHT_MOTOR_SVM


#ifdef RIGHT_MOTOR_BLDC
  // Torque (voltage) control of left motor in BLDC mode

  // Apply ramp
  pwm_diff = torque_ref - pwm_r_ramp;
  pwm_diff = LIMIT(pwm_diff, cfg.vars.rate_limit);
  pwm_r_ramp += pwm_diff;

  // Scale ramped reference according to PWM period and DC voltage
  torque_ref = fx_mul(pwm_r_ramp, PWM_PERIOD);
  torque_ref = fx_mul(torque_ref, voltage_scale);

  // Limit pwm reference to maximum limits
  setpoint_r_limit = LIMIT(torque_ref, cfg.vars.max_pwm_r);

  // Apply the reference
  motor_state[STATE_RIGHT].ctrl.amplitude = setpoint_r_limit;
#endif // RIGHT_MOTOR_BLDC


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

#if defined(LEFT_SENSOR_SCOPE) || defined(RIGHT_SENSOR_SCOPE)
  // Scope, send every other tick (start bits & 2 stop bits included)
  // To send start, 8 data, stop = (2+8) x 16 bits every 2 ms, we need (2+8) * (16+3) / 2e-3 = 95000 bps
  // so 115200 bps should be enough to send this data every 2 ms
  if(control_tick & 1) {
    scope_set_data(0, ia_l);
    scope_set_data(1, ib_l);
    scope_set_data(2, ia_r);
    scope_set_data(3, ib_r);
    scope_set_data(4, analog_meas.v_battery);
    scope_set_data(5, speed_l);
    scope_set_data(6, speed_r);
    scope_set_data(7, 0);
  }
#endif

  control_tick++;

  // Launch ADC3 so that at next call we
  // have fresh analog measurements
  hadc3.Instance->CR2 |= ADC_CR2_SWSTART;
}

/*
 * control.c
 *
 *  Created on: May 6, 2018
 *      Author: tomvoc
 *
 * Modified by Lauri Peltonen, 2019
 */

#include "control.h"
#include "config.h"
#include "defines.h"
#include "cfgbus.h"
#include "setup.h"
#include "svm.h"
#include "math.h"
#include "adc.h"
#include "imeas.h"
#include "uartscope.h"

#include <stm32f1xx_hal_gpio.h>

// From adc.c
extern ADC_HandleTypeDef hadc3;
extern volatile adc_buf_t analog_meas;

#define LED_PERIOD (300)  //ms

// TODO: Move to setup.c and calculate there
const uint16_t motor_nominal_counts = (3<<12);		// timer ticks/sector change at rated speed
const uint16_t sector_counts_to_svm = ANGLE_60DEG / (2*PWM_FREQ/1000);	// Control cycle runs at 1000 Hz
const uint16_t motor_voltage_scale = MOTOR_VOLTS / (MOTOR_POLEPAIRS * MOTOR_SPEED);

volatile uint16_t dc_voltage = 4096;		// Fixed point in p.u. TODO: Use ADC and convert to p.u.
const uint32_t adc_battery_to_pu = (FIXED_ONE / (2.45*MOTOR_VOLTS)) * (FIXED_ONE * ADC_BATTERY_VOLTS); // 2.45=sqrt(2)*sqrt(3)=phase RMS to main peak
const uint16_t adc_battery_filt = FIXED_ONE / 10;		// Low-pass filter gain in fixed point

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

// Array to convert HALL sensor readings (order ABC) to sector number
// Note that index 0 and 7 are "guards" and should never happen when sensors work properly
static const uint8_t hall_to_sector[8] = { 0, 5, 1, 0, 3, 4, 2, 0 };



void init_controls(void)
{

}



void update_controls(void)
{

}

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
  motor_state[STATE_LEFT].ctrl.angle = sector * ANGLE_60DEG;
  __enable_irq();

  sector = read_right_hall();
  motor_state[STATE_RIGHT].act.sector = sector;
  __disable_irq();
  motor_state[STATE_RIGHT].ctrl.angle = sector * ANGLE_60DEG;
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
  int16_t ia_l, ib_l, ic_l;
  int16_t ia_r, ib_r, ic_r;
  int16_t ref_l, ref_r;

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
    speed_l = motor_nominal_counts / speed_tick[0];
    uint16_t angle = sector_l * ANGLE_60DEG;

    if(sector_l != ((prev_sector_l + 1) % 6)) {
      speed_l = -speed_l;
      angle += ANGLE_60DEG;
    }


#ifdef SVM_HALL_UPDATE
    __disable_irq();	// Angle is also updated by modulator
    motor_state[STATE_LEFT].ctrl.angle = angle;
    __enable_irq();
    motor_state[STATE_LEFT].ctrl.speed = sector_counts_to_svm / speed_tick[0];
#endif

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
    uint16_t angle = sector_r * ANGLE_60DEG;

    if(sector_r != ((prev_sector_r + 1) % 6)) {
      speed_r = -speed_r;
      angle += ANGLE_60DEG;
    }

#ifdef SVM_HALL_UPDATE
    __disable_irq();	// Angle is also updated by modulator
    motor_state[STATE_RIGHT].ctrl.angle = angle;
    __enable_irq();
    motor_state[STATE_RIGHT].ctrl.speed = sector_counts_to_svm / speed_tick[1];
#endif

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

  // Current measurement and overcurrent trips
  // Left motor phase currents
  ia_l = i_meas.i_lA;
  ib_l = i_meas.i_lB;
  ic_l = -ia_l - ib_l;

  // Right motor phase currents
  ib_r = i_meas.i_rB;
  ic_r = i_meas.i_rC;
  ia_r = -ib_r - ic_r;

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
  battery_voltage_filt = fx_mulu(analog_meas.v_battery << 4, adc_battery_filt) + fx_mulu(battery_voltage_filt, FIXED_ONE-adc_battery_filt);
  battery_volt_pu = fx_mulu((battery_voltage_filt >> 4), adc_battery_to_pu);

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

  // Reference scaling so that 1 (4096) results in 1 (motor nominal voltage) always
  // So we scale all references by battery_voltage / nominal voltage
  voltage_scale = fx_divu(FIXED_ONE, battery_volt_pu);


#if defined(REFERENCE_MODBUS)
  ref_l = cfg.vars.spdref_l;
  ref_r = cfg.vars.spdref_r;
#elif defined(REFERENCE_ADC)
  // ADC output is 0...4095, scale it to -4096 ... 4095
  // TODO: Add some configuration (offset, gain) to these?
  ref_l = (analog_meas.analog_ref_1 - 2048) * 2;
  ref_r = (analog_meas.analog_ref_2 - 2048) * 2;
#else
  ref_l = 0
  ref_r = 0
#endif


  // Debug: rotate the SVM reference
#ifdef LEFT_MOTOR_SVM
  if(motor_state[STATE_LEFT].ref.control_mode == CONTROL_SPEED) {
    motor_state[STATE_LEFT].ctrl.speed = ref_l;
  } else if(motor_state[STATE_LEFT].ref.control_mode == CONTROL_ANGLE) {
    motor_state[STATE_LEFT].ctrl.angle = ref_l;  // DEBUG
  }
#endif

#ifdef RIGHT_MOTOR_SVM
  if(motor_state[STATE_RIGHT].ref.control_mode == CONTROL_SPEED) {
    motor_state[STATE_RIGHT].ctrl.speed = ref_r;
  } else if(motor_state[STATE_RIGHT].ref.control_mode == CONTROL_ANGLE) {
    motor_state[STATE_RIGHT].ctrl.angle = ref_r;  // DEBUG
  }
#endif


  // TODO: Move volatile(?) setpoints to local variables
//#ifdef LEFT_MOTOR_BLDC
  // Torque (voltage) control of left motor in BLDC mode
  setpoint_l_limit = CLAMP(cfg.vars.setpoint_l, -cfg.vars.max_pwm_l, cfg.vars.max_pwm_l);
//  setpoint_l_limit = CLAMP(ref_l, -cfg.vars.max_pwm_l, cfg.vars.max_pwm_l);

  pwm_diff = setpoint_l_limit - pwm_l_ramp;
  pwm_diff = CLAMP(pwm_diff, -cfg.vars.rate_limit, cfg.vars.rate_limit);

  pwm_l_ramp += pwm_diff;

  //motor_state[STATE_LEFT].ctrl.amplitude = fx_mul(pwm_l_ramp, voltage_scale); // TODO: Check and fix
  motor_state[STATE_LEFT].ctrl.amplitude = pwm_l_ramp;
//#endif

//#ifdef RIGHT_MOTOR_BLDC
  // Torque (voltage) control of left motor in BLDC mode
  setpoint_r_limit = CLAMP(cfg.vars.setpoint_r, -cfg.vars.max_pwm_r, cfg.vars.max_pwm_r);
  //setpoint_r_limit = CLAMP(ref_r, -cfg.vars.max_pwm_r, cfg.vars.max_pwm_r);

  pwm_diff = setpoint_r_limit - pwm_r_ramp;
  pwm_diff = CLAMP(pwm_diff, -cfg.vars.rate_limit, cfg.vars.rate_limit);

  pwm_r_ramp += pwm_diff;

  //motor_state[STATE_RIGHT].ctrl.amplitude = fx_mul(pwm_r_ramp, voltage_scale); // TODO: Check and fix
  motor_state[STATE_RIGHT].ctrl.amplitude = pwm_r_ramp;
//#endif


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
  cfg.vars.speed_l = motor_state[STATE_LEFT].act.period;//speed_l; // DEBUG
  cfg.vars.speed_r = motor_state[STATE_RIGHT].act.period; //speed_r;  // DEBUG


  // Copy ADC values to cfg array
  // TODO: Move to main?
  cfg.vars.vbat = analog_meas.v_battery;
  cfg.vars.vsw = analog_meas.v_switch;
  cfg.vars.temperature = analog_meas.temperature;
  cfg.vars.aref1 = analog_meas.analog_ref_1;
  cfg.vars.aref2 = analog_meas.analog_ref_2;
  cfg.vars.pwm_l = motor_state[STATE_LEFT].ctrl.amplitude;
  cfg.vars.pwm_r = motor_state[STATE_RIGHT].ctrl.amplitude;
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

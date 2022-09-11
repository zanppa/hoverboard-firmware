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
extern uint8_t imeas_calibration_done;

#define LED_PERIOD (300)  //ms

// TODO: Move to setup.c and calculate there
const uint16_t motor_nominal_counts = MOTOR_NOMINAL_PERIOD * (PWM_FREQ/1000.0);		// PWM ticks/sector change at rated speed (per unit)
//const uint16_t sector_counts_to_svm = ANGLE_60DEG / (2*PWM_FREQ/CONTROL_FREQ);	// Control cycle runs at 1000 Hz while modulator twice in PWM_FREQ
const uint16_t motor_voltage_scale = MOTOR_VOLTS / (MOTOR_POLEPAIRS * MOTOR_SPEED);


const uint32_t adc_battery_to_pu = (FIXED_ONE / (2.45*MOTOR_VOLTS)) * (FIXED_ONE * ADC_BATTERY_VOLTS); // 2.45=sqrt(2)*sqrt(3)=phase RMS to main peak
const uint16_t adc_battery_filt_gain = FIXED_ONE / 10;		// Low-pass filter gain in fixed point for battery voltage


// -----------
// Torque control (FOC D and Q axis currents) parameters
#if defined(LEFT_MOTOR_FOC) || defined(RIGHT_MOTOR_FOC)

// P and I terms for d and q axis current regulators for FOC
// TODO: Ifdefs
uint16_t kp_id = 0.6 * FIXED_ONE; //2000;
uint16_t ki_id = 0.01 * FIXED_ONE; //2000; //1200;

uint16_t kp_iq = 0.6 * FIXED_ONE;
uint16_t ki_iq = 0.08 * FIXED_ONE; //1200;

#if defined(RIGHT_MOTOR_FOC)
int16_t id_old_r = 0;
int16_t iq_old_r = 0;
#endif
#if defined(LEFT_MOTOR_FOC)
int16_t id_old_l = 0;
int16_t iq_old_l = 0;
#endif

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
static uint16_t kp_speed = 0.6 * FIXED_ONE;
static uint16_t ki_speed = 0.12 * FIXED_ONE;
static int16_t speed_error_int_l = 0;
static int16_t speed_error_int_r = 0;
static const uint16_t speed_int_max = 15000;	// TODO: What is a sane value?
static const uint16_t speed_int_divisor = 30;

uint16_t battery_volt_pu = 0;

// Warning/trip values in fixed point per-unit values
volatile uint8_t fault_bits = 0;		// Fault type, 0=no fault
const uint16_t ov_warn_pu = FIXED_ONE * OVERVOLTAGE_WARN / (2.45*MOTOR_VOLTS);
const uint16_t ov_trip_pu = FIXED_ONE * OVERVOLTAGE_TRIP / (2.45*MOTOR_VOLTS);
const uint16_t uv_warn_pu = FIXED_ONE * UNDERVOLTAGE_WARN / (2.45*MOTOR_VOLTS);
const uint16_t uv_trip_pu = FIXED_ONE * UNDERVOLTAGE_TRIP / (2.45*MOTOR_VOLTS);


volatile uint8_t status_bits = 0;	// Status bits

static uint8_t reset_ctrl = 0; // Force reseting the control state (integrators etc.)

volatile motor_state_t motor_state[2] = {0};

// Current measurement
extern volatile i_meas_t i_meas;

// Buzzer tone control
uint16_t buzzer_tone = 0x0; // No tone. This defines the tone(s) to play. 1 bit is 1 ms.
uint16_t buzzer_pattern = 0xFF00;	// Beep pattern, 1 bit is 64 ms (8b on 8b off is about 1 Hz beep)
uint8_t buzzer_custom = 0;

// LED blinking pattern control
volatile uint16_t led_pattern = 0xFF00;		// Default blinking pattern

// Non-volatile variables that are ONLY used in the control timer interrupt
volatile uint16_t control_tick = 0;
static int16_t speed_tick[2] = {0};

static uint8_t buzzer_tone_tick = 0;
static uint8_t pattern_tick = 0;

// Array to convert HALL sensor readings (order CBA, MSB first) to sector number
// Note that index 0 and 7 are "guards" and should never happen when sensors work properly
//static const uint8_t hall_to_sector[8] = { 0, 5, 1, 0, 3, 4, 2, 0 };
#if defined(HALL_GBYGBY)
static const uint8_t hall_to_sector[8] = { 0, 2, 0, 1, 4, 3, 5, 0 };
#elif defined(HALL_GBYBGY)
static const uint8_t hall_to_sector[8] = { 0, 2, 4, 3, 0, 1, 5, 0 };
#endif

// HALL mapping (original sector order)
// When board wire colors and motor wire colors match
// 			Phases			HALLs
//	Sector	U/G	V/B	W/Y		G/2	B/1	Y/0		Bin	Dec
//	0		1	0	0		0	1	0		010	2
//	1		1	1	0		0	1	1		011	3
//	2		0	1	0		0	0	1		001	1
//	3		0	1	1		1	0	1		101	5
//	4		0	0	1		1	0	0		100	4
//	5		1	0	1		1	1	0		110	6

// When board side has green and blue wires switched
// 			Phases			HALLs
//	Sector	U/G	V/B	W/Y		G/2	B/1	Y/0		Bin	Dec
//	0		1	0	0		1	0	0		100	4
//	1		1	1	0		1	0	1		101	5
//	2		0	1	0		0	0	1		001	1
//	3		0	1	1		0	1	1		011	3
//	4		0	0	1		0	1	0		010	2
//	5		1	0	1		1	1	0		110	6



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
    RIGHT_TIM->BDTR &= ~TIM_BDTR_MOE;	// Disable output
    motor_state[STATE_RIGHT].ctrl.enable = 0;
  }
}

// Enable one or both motor PWM outputs
// bit 0 = left, bit 1 = right
void enable_motors(uint8_t sides) {
  if(sides & 0x01) {
    LEFT_TIM->BDTR |= TIM_BDTR_MOE;	// Enable output
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
// for motors, set control variables, do precalculations and so on
void initialize_control_state(void) {
  uint8_t sector;

  // Initial rotor positions
  sector = read_left_hall();
  __disable_irq();
  motor_state[STATE_LEFT].act.sector = sector;
  motor_state[STATE_LEFT].act.angle = sector * ANGLE_60DEG;// + ANGLE_30DEG;	// Assume we're in the middle of a sector
  motor_state[STATE_LEFT].ctrl.amplitude = 0;
  motor_state[STATE_LEFT].ctrl.angle = sector * ANGLE_60DEG;;
  __enable_irq();

  sector = read_right_hall();
  __disable_irq();
  motor_state[STATE_RIGHT].act.sector = sector;
  motor_state[STATE_RIGHT].act.angle = sector * ANGLE_60DEG;// + ANGLE_30DEG;	// Assume middle of a sector
  motor_state[STATE_RIGHT].ctrl.amplitude = 0;
  motor_state[STATE_RIGHT].ctrl.angle = sector * ANGLE_60DEG;
  __enable_irq();


  // Reset integrators
  speed_error_int_l = 0;
  speed_error_int_r = 0;
#ifdef LEFT_MOTOR_FOC
  id_error_int_l = 0;
  iq_error_int_l = 0;
#endif // LEFT_MOTOR_FOC
#ifdef RIGHT_MOTOR_FOC
  id_error_int_r = 0;
  iq_error_int_r = 0;
#endif // RIGHT_MOTOR_FOC

  reset_ctrl = 1; // Force control loop to re-do this
}

// Helper to set buzzer from outside functions
void set_buzzer(uint16_t tone, uint16_t pattern, uint8_t enable)
{
  buzzer_tone = tone;
  buzzer_pattern = pattern;
  buzzer_custom = enable;
}


// Controller internal variables, i.e. ramped references
static int16_t ref_l_ramp = 0;
static int16_t ref_r_ramp = 0;
static uint16_t rate_lim_remainder = 0;

static uint16_t battery_voltage_filt = 0;	// Multiplied by 16 to increase filter accuracy, otherwise the error is something like 0.5 volts...

//called 64000000/64000 = 1000 times per second
void TIM3_IRQHandler(void)
{
  int16_t speed_l, speed_r;
  uint16_t voltage_scale;
  int16_t ia_l, ib_l, ic_l;
  int16_t ia_r, ib_r, ic_r;
#if defined(LEFT_MOTOR_FOC)
  uint16_t angle_l;
#endif
#if defined(RIGHT_MOTOR_FOC)
  uint16_t angle_r;
#endif
  int16_t ref_l, ref_r;
  int16_t ref_ramp_diff;
  int16_t speed_error;
  int16_t torque_ref;
  int16_t torque_lim_speed = INT16_MAX; // Limits motoring (speeding up) torque only, not braking
  int16_t torque_lim_volt = INT16_MAX; // Positive --> motoring (speeding up) limit, negative --> generating (braking) limit
  uint8_t ctrl_mode;
  uint16_t v_bat;

#if defined(LEFT_MOTOR_FOC) || defined(RIGHT_MOTOR_FOC)
  int16_t ia, ib;
  uint16_t angle;
  int16_t ialpha, ibeta;
  int16_t id, iq;
  int16_t id_error, iq_error;
  int16_t angle_advance, ref_amplitude;
  int8_t ref_sign;
#endif

  CTRL_TIM->SR = 0;

  if(reset_ctrl) {
    initialize_control_state();
    reset_ctrl = 0;
  }

  // Update motor speed from latest period information
  // Left motor
  speed_tick[0] = motor_state[STATE_LEFT].act.period;
  if(speed_tick[0] != PERIOD_STOP && speed_tick[0] != -PERIOD_STOP)
    speed_l = fx_div(motor_nominal_counts, speed_tick[0]);
  else
    speed_l = 0;

  // Right motor
  speed_tick[1] = motor_state[STATE_RIGHT].act.period;
  if(speed_tick[1] != PERIOD_STOP && speed_tick[1] != -PERIOD_STOP)
    speed_r = fx_div(motor_nominal_counts, speed_tick[1]);
  else
    speed_r = 0;


  // Check overspeed trip
  if(speed_l > OVERSPEED_TRIP || speed_l < -OVERSPEED_TRIP ||
     speed_r > OVERSPEED_TRIP || speed_r < -OVERSPEED_TRIP) {
    do_fault(0x01 | 0x02);	// Trip both motors
    fault_bits |= FAULT_OVERSPEED;

    buzzer_pattern = 0xA8A8;
    buzzer_tone = 0xAA88;
  }


  // Current measurement and overcurrent trips
  // Left motor phase currents and position
  __disable_irq();
  ia_l = i_meas.i_lA;
  ib_l = i_meas.i_lB;
#if defined(LEFT_MOTOR_FOC)
  angle_l = motor_state[STATE_LEFT].act.angle;
#endif

  // Right motor phase currents and position
  ib_r = i_meas.i_rB;
  ic_r = i_meas.i_rC;
#if defined(RIGHT_MOTOR_FOC)
  angle_r = motor_state[STATE_RIGHT].act.angle;
#endif
  __enable_irq();

  ia_r = -ib_r - ic_r;
  ic_l = -ia_l - ib_l;

  // Check if currents exceed overcurrent limits
  // and trip one both motors
  // But only check when we're ready to run, i.e. measurements are stable
  if(status_bits & STATUS_READY) {
    if(ia_l > OVERCURRENT_TRIP || ia_l < -OVERCURRENT_TRIP ||
       ib_l > OVERCURRENT_TRIP || ib_l < -OVERCURRENT_TRIP ||
       ic_l > OVERCURRENT_TRIP || ic_l < -OVERCURRENT_TRIP) {
      do_fault(0x01 | 0x02);	// Trip both motors
      fault_bits |= FAULT_OVERCURRENT;

      buzzer_tone = 0x92A4;
      buzzer_pattern = 0xC30C;
    }

    if(ia_r > OVERCURRENT_TRIP || ia_r < -OVERCURRENT_TRIP ||
       ib_r > OVERCURRENT_TRIP || ib_r < -OVERCURRENT_TRIP ||
       ic_r > OVERCURRENT_TRIP || ic_r < -OVERCURRENT_TRIP) {
      do_fault(0x01 | 0x02);	// Trip both motors
      fault_bits |= FAULT_OVERCURRENT;

      buzzer_tone = 0x92A4;
      buzzer_pattern = 0xC30C;
    }
  }


  // Analog measurements (battery voltage, to be used in modulator)
  v_bat = analog_meas.v_battery + ADC_BATTERY_OFFSET;
  //battery_voltage_filt = fx_mulu(analog_meas.v_battery << 4, adc_battery_filt_gain) + fx_mulu(battery_voltage_filt, FIXED_ONE-adc_battery_filt);
  battery_voltage_filt = FILTERU(v_bat << 4, battery_voltage_filt, adc_battery_filt_gain);
  battery_volt_pu = fx_mulu((battery_voltage_filt >> 4), adc_battery_to_pu);

  // Reference scaling so that 1 (4096) results in 1 (motor nominal voltage) always
  // So we scale all references by battery_voltage / nominal voltage
  voltage_scale = fx_divu(FIXED_ONE, battery_volt_pu);
  //voltage_scale = fx_div(FIXED_ONE, battery_volt_pu);


  // Check voltage limits
  // Only trip if everything is ready, e.g. voltage has been filtered long enough etc.
  if(battery_volt_pu > ov_trip_pu && (status_bits & STATUS_READY)) {
    do_fault(0x01 | 0x02);	// Trip both motors
    fault_bits |= FAULT_OVERVOLTAGE;
    // TODO: Blink led

    buzzer_tone = 0xCCCC;
    buzzer_pattern = 0xFFFF;

  } else if(battery_volt_pu > ov_warn_pu) {
    status_bits |= STATUS_OVERVOLTAGE_WARN;

    // Torque limitation
    // At low voltage this results in INT16_MIN (negative), then goes to zero when voltage is increased
    // i.e. allow braking current (that charges battery) only when voltage is low enough
#if defined(OVERVOLTAGE_LIM_GAIN) && OVERVOLTAGE_LIM_GAIN > 0
    torque_lim_volt = CLAMP(battery_volt_pu - ov_warn_pu, (INT16_MIN + OVERVOLTAGE_LIM_OFFSET) / OVERVOLTAGE_LIM_GAIN,
                            (INT16_MAX - OVERVOLTAGE_LIM_OFFSET) / OVERVOLTAGE_LIM_GAIN) * OVERVOLTAGE_LIM_GAIN;
    torque_lim_volt = CLAMP(torque_lim_volt, INT16_MIN + OVERVOLTAGE_LIM_OFFSET, OVERVOLTAGE_LIM_OFFSET) - OVERVOLTAGE_LIM_OFFSET;
#endif

    // TODO: LED Blink?

    buzzer_tone = 0xCCCC;
    buzzer_pattern = 0xAAAA;

  } else if(battery_volt_pu < uv_trip_pu && (status_bits & STATUS_READY)) {
    do_fault(0x01 | 0x02); // Trip both motors
    fault_bits |= FAULT_UNDERVOLTAGE;

    buzzer_tone = 0xF0F0;
    buzzer_pattern = 0xFFFF;

  } else if(battery_volt_pu < uv_warn_pu) {
    status_bits |= STATUS_UNDERVOLTAGE_WARN;

    // Torque limitation
    // At high voltage this results in INT16_MAX (positive), then goes to zero when voltage is increased
    // i.e. allows motoring (battery discharging) current only at high enough voltage
#if defined(UNDERVOLTAGE_LIM_GAIN) && UNDERVOLTAGE_LIM_GAIN > 0
    torque_lim_volt = CLAMP(ov_warn_pu - battery_volt_pu, (INT16_MIN + UNDERVOLTAGE_LIM_OFFSET) / UNDERVOLTAGE_LIM_GAIN,
                            (INT16_MAX - UNDERVOLTAGE_LIM_OFFSET) / UNDERVOLTAGE_LIM_GAIN) * UNDERVOLTAGE_LIM_GAIN;
    torque_lim_volt = UNDERVOLTAGE_LIM_OFFSET - CLAMP(torque_lim_volt, INT16_MIN + UNDERVOLTAGE_LIM_OFFSET+1, UNDERVOLTAGE_LIM_OFFSET);
#endif

    // TODO: LED blink

    buzzer_tone = 0xF0F0;
    buzzer_pattern = 0xAAAA;

  } else { // Remove alarm bits
    status_bits &= ~(STATUS_OVERVOLTAGE_WARN | STATUS_UNDERVOLTAGE_WARN);

    // Clear buzzer only if no faults are active
    if(!fault_bits && !buzzer_custom) {
      buzzer_tone = 0;
      buzzer_pattern = 0;
    }

    // Check that filtered DC link voltage is high enough and indicate ready state
    if(!(status_bits & STATUS_READY) && battery_volt_pu > uv_warn_pu && imeas_calibration_done)
      status_bits |= STATUS_READY;
  }



  // Reference generation

#if defined(REFERENCE_MODBUS)
  ref_l = cfg.vars.setpoint_l;
  ref_r = cfg.vars.setpoint_r;
#elif defined(REFERENCE_ADC)
#if defined(REFERENCE_ADC_DIFF)
  // This is simple and results in -8192 ... 8191 range
  // TODO: Add scaling and offset and deadband?
  ref_l = analog_meas.analog_ref_1 + analog_meas.analog_ref_2;
  ref_r = analog_meas.analog_ref_2 - analog_meas.analog_ref_2;
  ref_l = (ref_l - 4096) * 2;
  ref_r = (ref_r - 4096) * 2;
#elif defined(REFERENCE_ADC_SINGLE)
  ref_l = ((int16_t)analog_meas.analog_ref_1 - 2048) * 2;
  ref_r = ref_l;
#else // REFERENCE_ADC_SINGLE
  // ADC output is 0...4095, scale it to -4096 ... 4095
  // TODO: Add some configuration (offset, gain, deadband) to these?
  ref_l = ((int16_t)analog_meas.analog_ref_1 - 2048) * 2;
  ref_r = ((int16_t)analog_meas.analog_ref_2 - 2048) * 2;
#endif // REFERENCE_ADC_DIFF
#else // REFERENCE_ADC
  ref_l = 0;
  ref_r = 0;
#endif // REFERENCE_ADC


  // Apply ramps to references
  rate_lim_remainder += cfg.vars.rate_limit;
  uint16_t rate_limited = rate_lim_remainder / 1000;
  rate_lim_remainder -= (rate_limited * 1000);

  ref_ramp_diff = ref_l - ref_l_ramp;
  ref_ramp_diff = LIMIT(ref_ramp_diff, rate_limited);
  ref_l_ramp += ref_ramp_diff;
  motor_state[STATE_LEFT].ref.value = ref_l_ramp;

  ref_ramp_diff = ref_r - ref_r_ramp;
  ref_ramp_diff = LIMIT(ref_ramp_diff, rate_limited);
  ref_r_ramp += ref_ramp_diff;
  motor_state[STATE_RIGHT].ref.value = ref_r_ramp;



  // --------------
  // Left motor

  // Speed control loop for left motor
  ctrl_mode = motor_state[STATE_LEFT].ref.control_mode;

  if(ctrl_mode == CONTROL_SPEED) {
    // FOC and BLCD in speed control mode --> run PI controller
    speed_error = motor_state[STATE_LEFT].ref.value - speed_l;
    //speed_error_int_l = LIMIT(speed_error_int_l + (speed_error / speed_int_divisor), speed_int_max);
    speed_error_int_l += speed_error / speed_int_divisor;
    speed_error_int_l = LIMIT(speed_error_int_l, speed_int_max);
    //torque_ref = speed_error + fx_mul(speed_error_int_l, ki_speed);
    //torque_ref = fx_mul(torque_ref, kp_speed);
    torque_ref = fx_mul(speed_error, kp_speed) + fx_mul(speed_error_int_l, ki_speed);
  } else if(ctrl_mode == CONTROL_TORQUE) {
    torque_ref = motor_state[STATE_LEFT].ref.value;
  } else {
    torque_ref = 0;
  }

  // Limit torque reference
  torque_ref = LIMIT(torque_ref, cfg.vars.max_tref_l);


  // Torque reference limitation above overspeed
  // Note! This only limits torque if torque is in the same direction as speed
  // During braking, limitation is not active but will brake until overspeed trip
  if(speed_l > OVERSPEED_LIMIT || speed_l < -OVERSPEED_LIMIT) {
#if defined(OVERSPEED_LIM_GAIN) && OVERSPEED_LIM_GAIN > 0
    torque_lim_speed = CLAMP(ABS(speed_l) - OVERSPEED_LIMIT, (INT16_MIN + OVERSPEED_LIM_OFFSET) / OVERSPEED_LIM_GAIN,
                       (INT16_MAX - OVERSPEED_LIM_OFFSET) / OVERSPEED_LIM_GAIN) * OVERSPEED_LIM_GAIN;
    torque_lim_speed = OVERSPEED_LIM_OFFSET - CLAMP(torque_lim_speed, INT16_MIN + OVERSPEED_LIM_OFFSET + 1, OVERSPEED_LIM_OFFSET);
#endif

    status_bits |= STATUS_OVERSPEED_WARN_L;
    buzzer_pattern = 0x9120;
    buzzer_tone = 0xAA88;
  } else status_bits &= ~STATUS_OVERSPEED_WARN_L;


  // Apply torque limitations
  if(speed_l > 0 && torque_ref > 0) {
    // Positive motoring direction --> undervoltage & overspeed limitations (both positive)
    torque_ref = MIN(torque_ref, torque_lim_speed);
    if(torque_lim_volt > 0)
      torque_ref = MIN(torque_ref, MAX(0, torque_lim_volt));
  } else if(speed_l < 0 && torque_ref < 0) {
    // Negative motoring direction --> undervoltage & overspeed limitations (both positive)
    torque_ref = MAX(torque_ref, -torque_lim_speed);
    if(torque_lim_volt > 0)
      torque_ref = MAX(torque_ref, -MAX(0, torque_lim_volt));
  } else if(torque_lim_volt < 0) {
    // Braking --> overvoltage only (negative)
    if(torque_ref > 0)
      torque_ref = MIN(torque_ref, -MIN(0, torque_lim_volt));
    else
      torque_ref = MAX(torque_ref, MIN(0, torque_lim_volt));
  }

  // Back-apply the now limited reference to ramped reference
  if(ctrl_mode == CONTROL_TORQUE)
    ref_l_ramp = torque_ref;
  // TODO: How to do similar thing in speed control...?



#if defined(LEFT_MOTOR_FOC)
  // Get the interpolated position and measured currents from exactly same time instants
  ia = ia_l;
  ib = ib_l;
  angle = angle_l;

  // Transform to rotor coordinate frame
  clarke(ia, ib, &ialpha, &ibeta);
  park(ialpha, ibeta, angle, &id, &iq);

  // Filter id and iq
  id = FILTER(id, id_old_l, cfg.vars.i_filter);
  iq = FILTER(iq, iq_old_l, cfg.vars.i_filter);
  id_old_l = id;
  iq_old_l = iq;

  id_error = id;	// TODO: Add id reference (from field weakening)
  //int16_t iq_error = cfg.vars.setpoint_l - iq;
  iq_error = torque_ref - iq;

  // Run the PI controllers
  // First for D axis current which sets the angle advance
  //id_error_int_l = LIMIT(id_error_int_l + (id_error / int_divisor), idq_int_max);
  id_error_int_l += id_error / int_divisor;
  id_error_int_l = LIMIT(id_error_int_l, idq_int_max);
  angle_advance = fx_mul(id_error, kp_id) + fx_mul(id_error_int_l, ki_id);
  angle_advance *= 8;	// From 12-bit fixed point to 16-bit angle => 1 = 4096 = one full rotation

  // Then for Q axis current which sets the reference amplitude
  //iq_error_int_l = LIMIT(iq_error_int_l + iq_error, idq_int_max);
  iq_error_int_l += iq_error;
  iq_error_int_l = LIMIT(iq_error_int_l, idq_int_max);
  ref_amplitude = fx_mul(iq_error, kp_iq) + fx_mul(iq_error_int_l, ki_iq);

  ref_sign = SIGN(ref_amplitude);

  // Apply DC voltage scaling
  ref_amplitude = fx_mul(ref_amplitude, voltage_scale);

  // Apply limiter
  ref_amplitude = ABS(ref_amplitude);
  ref_amplitude = MIN(ref_amplitude, cfg.vars.max_pwm_l);

  // Apply references
  __disable_irq();
  motor_state[STATE_LEFT].ctrl.amplitude = (uint16_t)ref_amplitude;
  motor_state[STATE_LEFT].ctrl.angle = (uint16_t)angle_advance + (ref_sign * ANGLE_120DEG);	// Should start with 90 degree phase shift
  // TODO: Angle advance polarity should change depending on speed direction
  __enable_irq();

  //torque_ref = ref_amplitude; // TODO: For debugging purposes only

#elif defined(LEFT_MOTOR_SVM) && !defined(LEFT_MOTOR_FOC)
  // TODO: U/f control for SVM without FOC?
  if(ctrl_mode == CONTROL_SPEED) {
    torque_ref = fx_mul(ABS(motor_state[STATE_LEFT].ref.value), voltage_scale);
    __disable_irq();
    motor_state[STATE_LEFT].ctrl.amplitude = MAX(torque_ref, IR_MINIMUM_VOLTAGE);
    //motor_state[STATE_LEFT].ctrl.speed = motor_state[STATE_LEFT].ref.value / SPEED_SCALE;
    // TODO: Clean this up...
    motor_state[STATE_LEFT].ctrl.speed = (ANGLE_60DEG * motor_state[STATE_LEFT].ref.value) / (FIXED_ONE * motor_nominal_counts * 2);
    // Ref: speed_l = (FIXED_ONE * motor_nominal_counts) / speed_tick[0];
    __enable_irq();
  } else if(ctrl_mode == CONTROL_ANGLE) {
    __disable_irq();
    motor_state[STATE_LEFT].ctrl.speed = 0;
    motor_state[STATE_LEFT].ctrl.angle = motor_state[STATE_LEFT].ref.value;
    motor_state[STATE_LEFT].ctrl.amplitude = IR_MINIMUM_VOLTAGE;
    __enable_irq();
  }



#elif defined(LEFT_MOTOR_BLDC)
  // Torque (voltage) control of left motor in BLDC mode

  // Scale ramped reference according to DC voltage
  torque_ref = fx_mul(torque_ref, voltage_scale);

  // Limit pwm value
  torque_ref = LIMIT(torque_ref, cfg.vars.max_pwm_l);

#if defined(BLDC_FIELD_WEAKENING) && 0 // TODO: Debug remove
  // Apply field weakening if necessary
  uint16_t tref_abs = ABS(torque_ref);
  if(tref_abs > FIXED_ONE) {
    // "Excess" torque reference
    tref_abs = tref_abs - FIXED_ONE;

    // Limit the torque reference to one (full modulation amplitude)
    torque_ref = LIMIT(torque_ref, FIXED_ONE);

    __disable_irq();
    // Excess is directly applied as the field weakening ref
    motor_state[STATE_LEFT].ctrl.angle = tref_abs;
    motor_state[STATE_LEFT].ctrl.amplitude = torque_ref;
    __enable_irq();

  }
#else

  // Apply the reference
  __disable_irq();
  motor_state[STATE_LEFT].ctrl.amplitude = torque_ref;
  __enable_irq();

#endif

#endif // LEFT_MOTOR_BLDC

  cfg.vars.t_req_l = torque_ref;




  // ------------
  // Right motor

  ctrl_mode = motor_state[STATE_RIGHT].ref.control_mode;

  if(ctrl_mode == CONTROL_SPEED) {
    // Speed control loop for right motor
    // FOC and BLCD in speed mode --> run PI controller
    speed_error = motor_state[STATE_RIGHT].ref.value - speed_r;
    //speed_error_int_r = LIMIT(speed_error_int_r + (speed_error / speed_int_divisor), speed_int_max);
    speed_error_int_r += speed_error / speed_int_divisor;
    speed_error_int_r = LIMIT(speed_error_int_r, speed_int_max);
    //torque_ref = speed_error + fx_mul(speed_error_int_r, ki_speed);
    //torque_ref = fx_mul(torque_ref, kp_speed);
    torque_ref = fx_mul(speed_error, kp_speed) + fx_mul(speed_error_int_r, ki_speed);
  } else if(ctrl_mode == CONTROL_TORQUE) {
    torque_ref = motor_state[STATE_RIGHT].ref.value;
  } else {
    torque_ref = 0;
  }

  // Limit torque reference
  torque_ref = LIMIT(torque_ref, cfg.vars.max_tref_r);

  // Torque reference limitation above overspeed
  // Note! This only limits torque if torque is in the same direction as speed
  // During braking, limitation is not active but will brake until overspeed trip
  torque_lim_speed = INT16_MAX;
  if(speed_r > OVERSPEED_LIMIT || speed_r < -OVERSPEED_LIMIT) {
#if defined(OVERSPEED_LIM_GAIN) && OVERSPEED_LIM_GAIN > 0
    torque_lim_speed = CLAMP(ABS(speed_r) - OVERSPEED_LIMIT, (INT16_MIN + OVERSPEED_LIM_OFFSET) / OVERSPEED_LIM_GAIN,
                       (INT16_MAX - OVERSPEED_LIM_OFFSET) / OVERSPEED_LIM_GAIN) * OVERSPEED_LIM_GAIN;
    torque_lim_speed = OVERSPEED_LIM_OFFSET - CLAMP(torque_lim_speed, INT16_MIN + OVERSPEED_LIM_OFFSET + 1, OVERSPEED_LIM_OFFSET);
#endif

    status_bits |= STATUS_OVERSPEED_WARN_R;
    buzzer_pattern = 0x9120;
    buzzer_tone = 0xAA88;
  } else status_bits &= ~STATUS_OVERSPEED_WARN_R;


  // Apply torque limitations
  if(speed_r > 0 && torque_ref > 0) {
    // Positive motoring direction --> undervoltage & overspeed limitations (both positive)
    torque_ref = MIN(torque_ref, torque_lim_speed);
    if(torque_lim_volt > 0)
      torque_ref = MIN(torque_ref, MAX(0, torque_lim_volt));
  } else if(speed_r < 0 && torque_ref < 0) {
    // Negative motoring direction --> undervoltage & overspeed limitations (both positive)
    torque_ref = MAX(torque_ref, -torque_lim_speed);
    if(torque_lim_volt > 0)
      torque_ref = MAX(torque_ref, -MAX(0, torque_lim_volt));
  } else if(torque_lim_volt < 0) {
    // Braking --> overvoltage only (negative)
    if(torque_ref > 0)
      torque_ref = MIN(torque_ref, -MIN(0, torque_lim_volt));
    else
      torque_ref = MAX(torque_ref, MIN(0, torque_lim_volt));
  }

  // Back-apply the now limited reference to ramped reference
  if(ctrl_mode == CONTROL_TORQUE)
    ref_r_ramp = torque_ref;
  // TODO: How to do similar thing in speed control...?



#if defined(RIGHT_MOTOR_FOC)
  // Get the interpolated position and measured currents from exactly same time instants
  ia = ia_r;		// For simplicity, we use ia and ib in the calculation
  ib = ib_r;
  angle = angle_r;	// Estimated rotor position

  // Transform to rotor coordinate frame
  clarke(ia, ib, &ialpha, &ibeta);
  park(ialpha, ibeta, angle, &id, &iq);

  // Filter id and iq
  id = FILTER(id, id_old_r, cfg.vars.i_filter);
  iq = FILTER(iq, iq_old_r, cfg.vars.i_filter);
  id_old_r = id;
  iq_old_r = iq;


  id_error = id;    // TODO: Add id reference (from field weakening)
  //int16_t iq_error = cfg.vars.setpoint_r - iq;
  iq_error = torque_ref - iq;

  // Run the PI controllers
  // First for D axis current which sets the angle advance
  //id_error_int_r = LIMIT(id_error_int_r + (id_error/int_divisor), idq_int_max);
  id_error_int_r += id_error/int_divisor;
  id_error_int_r = LIMIT(id_error_int_r, idq_int_max);
  angle_advance = fx_mul(id_error, kp_id) + fx_mul(id_error_int_r, ki_id);
  angle_advance *= 8;

  // Invert phase advance if speed is reverse
  //if(speed_r < 0) angle_advance = -angle_advance;
  // TODO: Should probably be the reference, not actual...

  // Then for Q axis current which sets the reference amplitude
  //iq_error_int_r = LIMIT(iq_error_int_r + iq_error, idq_int_max);
  iq_error_int_r += iq_error;
  iq_error_int_r = LIMIT(iq_error_int_r, idq_int_max);
  ref_amplitude = fx_mul(iq_error, kp_iq) + fx_mul(iq_error_int_r, ki_iq);

  ref_sign = SIGN(ref_amplitude);

  // Apply DC voltage scaling
  ref_amplitude = fx_mul(ref_amplitude, voltage_scale);

  // Apply limiter
  ref_amplitude = ABS(ref_amplitude);
  ref_amplitude = MIN(ref_amplitude,cfg.vars.max_pwm_r);

  // Apply references
  __disable_irq();
  motor_state[STATE_RIGHT].ctrl.amplitude = (uint16_t)ref_amplitude;
  motor_state[STATE_RIGHT].ctrl.angle = (uint16_t)angle_advance + (ref_sign * ANGLE_120DEG);
  // TODO: Angle advance polarity should change depending on speed direction
  __enable_irq();

  //torque_ref = ref_amplitude; // TODO: Debug


#elif defined(RIGHT_MOTOR_SVM)  && !defined(RIGHT_MOTOR_FOC)
  // TODO: U/f control for SVM without FOC?

  if(ctrl_mode == CONTROL_SPEED) {
    torque_ref = fx_mul(ABS(motor_state[STATE_RIGHT].ref.value), voltage_scale);
    __disable_irq();
    motor_state[STATE_RIGHT].ctrl.amplitude = MAX(torque_ref, IR_MINIMUM_VOLTAGE);
    //motor_state[STATE_RIGHT].ctrl.amplitude = MAX(ABS(motor_state[STATE_RIGHT].ref.value), IR_MINIMUM_VOLTAGE);
    //motor_state[STATE_RIGHT].ctrl.speed = motor_state[STATE_RIGHT].ref.value / SPEED_SCALE;
    // TODO: Clean this
    motor_state[STATE_RIGHT].ctrl.speed = (ANGLE_60DEG * motor_state[STATE_RIGHT].ref.value) / (FIXED_ONE * motor_nominal_counts * 2);
    __enable_irq();
  } else if(ctrl_mode == CONTROL_ANGLE) {
    __disable_irq();
    motor_state[STATE_RIGHT].ctrl.speed = 0;
    motor_state[STATE_RIGHT].ctrl.angle = motor_state[STATE_RIGHT].ref.value;
    motor_state[STATE_RIGHT].ctrl.amplitude = IR_MINIMUM_VOLTAGE;
    __enable_irq();
  }

#elif defined(RIGHT_MOTOR_BLDC)
  // Torque (voltage) control of left motor in BLDC mode

  // Scale ramped reference according to DC voltage
  torque_ref = fx_mul(torque_ref, voltage_scale);

  // Limit pwm reference to maximum limits
  torque_ref = LIMIT(torque_ref, cfg.vars.max_pwm_r);

#if defined(BLDC_FIELD_WEAKENING) && 0 // TODO: Debug remove
  // Apply field weakening if necessary
  uint16_t tref_abs = ABS(torque_ref);
  if(tref_abs > FIXED_ONE) {
    // "Excess" torque reference
    tref_abs = tref_abs - FIXED_ONE;

    // Limit the torque reference to one (full modulation amplitude)
    torque_ref = LIMIT(torque_ref, FIXED_ONE);

    // Excess is directly applied as the field weakening ref
    __disable_irq();
    motor_state[STATE_RIGHT].ctrl.angle = tref_abs;
    motor_state[STATE_RIGHT].ctrl.amplitude = torque_ref;
    __enable_irq();
  }
#else
  // Apply the reference
  __disable_irq();
  motor_state[STATE_RIGHT].ctrl.amplitude = torque_ref;
  __enable_irq();
#endif

#endif // RIGHT_MOTOR_BLDC

  cfg.vars.t_req_r = torque_ref; // TODO: DEBUG



  // Update buzzer
  if(!cfg.vars.buzzer)
  {
	  HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, 0);
  }
  else
  {
    // Modbus can be used to override buzzer to test notes & patterns
    if(cfg.vars.buzzer_tone != 0 && cfg.vars.buzzer_pattern != 0) {
      buzzer_pattern = cfg.vars.buzzer_pattern;
      buzzer_tone = cfg.vars.buzzer_tone;
    }

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
  if((control_tick & 0x40) != ((control_tick - 1) & 0x40))	// Every 64 ms
    pattern_tick = (pattern_tick + 1) & 0xF;


  motor_state[STATE_LEFT].act.current[0] = i_meas.i_lA;
  motor_state[STATE_LEFT].act.current[1] = i_meas.i_lB;
  motor_state[STATE_LEFT].act.current[2] = -i_meas.i_lA - i_meas.i_lB;
  motor_state[STATE_RIGHT].act.current[0] = -i_meas.i_lB - i_meas.i_rC;
  motor_state[STATE_RIGHT].act.current[1] = i_meas.i_rB;
  motor_state[STATE_RIGHT].act.current[2] = i_meas.i_rC;


  // Update controller tuning parameters
#if defined(LEFT_MOTOR_FOC) || defined(RIGHT_MOTOR_FOC)
  kp_iq = cfg.vars.kp_iq;
  ki_iq = cfg.vars.ki_iq;
  kp_id = cfg.vars.kp_id;
  ki_id = cfg.vars.ki_id;
#endif


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
    scope_send();
  }
#endif

  control_tick++;

  // Launch ADC3 so that at next call we
  // have fresh analog measurements
  hadc3.Instance->CR2 |= ADC_CR2_SWSTART;
}

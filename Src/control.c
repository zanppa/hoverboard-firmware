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
extern volatile uint8_t imeas_calibration_done;

#define LED_PERIOD (300)  //ms

// TODO: Move to setup.c and calculate there
const uint16_t motor_nominal_counts = MOTOR_NOMINAL_PERIOD * (PWM_FREQ/1000.0);		// PWM ticks/sector change at rated speed (per unit)
//const uint16_t sector_counts_to_svm = ANGLE_60DEG / (2*PWM_FREQ/CONTROL_FREQ);	// Control cycle runs at 1000 Hz while modulator twice in PWM_FREQ
const uint16_t motor_voltage_scale = MOTOR_VOLTS / (MOTOR_POLEPAIRS * MOTOR_SPEED);


const uint32_t adc_battery_to_pu = (FIXED_ONE / (2.45*MOTOR_VOLTS)) * (FIXED_ONE * ADC_BATTERY_VOLTS); // 2.45=sqrt(2)*sqrt(3)=phase RMS to main peak
const uint16_t adc_battery_filt_gain = FIXED_ONE / 10;		// Low-pass filter gain in fixed point for battery voltage

const uint16_t speed_filt_gain = FIXED_ONE * 0.1;

#if defined(LEFT_CURRENT_TFORM)
int16_t id_old_l = 0;
int16_t iq_old_l = 0;
#endif
#if defined(RIGHT_CURRENT_TFORM)
int16_t id_old_r = 0;
int16_t iq_old_r = 0;
#endif


// -----------
// Torque control (FOC D and Q axis currents) parameters
#if defined(LEFT_MOTOR_FOC) || defined(RIGHT_MOTOR_FOC)

// P and I terms for d and q axis current regulators for FOC
// TODO: Ifdefs
uint16_t kp_id = 0.6 * FIXED_ONE; //2000;
uint16_t ki_id = 0.01 * FIXED_ONE; //2000; //1200;

uint16_t kp_iq = 0.6 * FIXED_ONE;
uint16_t ki_iq = 0.08 * FIXED_ONE; //1200;

// Id and Iq error integrals for FOC
#if defined(LEFT_MOTOR_FOC)
static int16_t id_error_int_l = 0;
static int16_t iq_error_int_l = 0;
#endif // LEFT_MOTOR_FOC
#if defined(RIGHT_MOTOR_FOC)
static int16_t id_error_int_r = 0;
static int16_t iq_error_int_r = 0;
#endif // RIGHT_MOTOR_FOC
const int16_t idq_int_max = 12000;	// TODO: What is a sane value...?
const uint8_t int_divisor = 10;
#endif // LEFT_MOTOR_FOC || RIGHT_MOTOR_FOC

// ---------
// Speed control parameters
static uint16_t kp_speed = 0.6 * FIXED_ONE;
static uint16_t ki_speed = 0.12 * FIXED_ONE;
static int16_t speed_error_int_l = 0;
static int16_t speed_error_int_r = 0;
static const uint16_t speed_int_max = 8000;	// TODO: What is a sane value?
static const uint16_t speed_int_divisor = 30;

uint16_t battery_volt_pu = 0;

// Warning/trip values in fixed point per-unit values
volatile uint8_t fault_bits = 0;		// Fault type, 0=no fault
const uint16_t ov_warn_pu = FIXED_ONE * OVERVOLTAGE_WARN / (2.45*MOTOR_VOLTS);
const uint16_t ov_trip_pu = FIXED_ONE * OVERVOLTAGE_TRIP / (2.45*MOTOR_VOLTS);
const uint16_t uv_warn_pu = FIXED_ONE * UNDERVOLTAGE_WARN / (2.45*MOTOR_VOLTS);
const uint16_t uv_trip_pu = FIXED_ONE * UNDERVOLTAGE_TRIP / (2.45*MOTOR_VOLTS);

// Limitation constants to prevent overflow
#if defined(OVERVOLTAGE_LIM_GAIN) && OVERVOLTAGE_LIM_GAIN > 0
const int16_t ov_lim_min = (INT16_MIN + OVERVOLTAGE_LIM_OFFSET) / OVERVOLTAGE_LIM_GAIN;
const int16_t ov_lim_max = (INT16_MAX - OVERVOLTAGE_LIM_OFFSET) / OVERVOLTAGE_LIM_GAIN;
#endif
#if defined(UNDERVOLTAGE_LIM_GAIN) && UNDERVOLTAGE_LIM_GAIN > 0
const int16_t uv_lim_min = (INT16_MIN + UNDERVOLTAGE_LIM_OFFSET) / UNDERVOLTAGE_LIM_GAIN;
const int16_t uv_lim_max = (INT16_MAX - UNDERVOLTAGE_LIM_OFFSET) / UNDERVOLTAGE_LIM_GAIN;
#endif
#if defined(OVERSPEED_LIM_GAIN) && OVERSPEED_LIM_GAIN > 0
const int16_t overspeed_lim_min = (INT16_MIN + OVERSPEED_LIM_OFFSET) / OVERSPEED_LIM_GAIN;
const int16_t overspeed_lim_max = (INT16_MAX - OVERSPEED_LIM_OFFSET) / OVERSPEED_LIM_GAIN;
#endif


volatile uint8_t status_bits = 0;	// Status bits

static volatile uint8_t reset_ctrl = 0; // Force reseting the control state (integrators etc.)

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

#if defined(DATALOGGER_ENABLE) && defined(DATALOGGER_TRIG_TRIP)
extern volatile uint8_t datalogger_trigger;
#endif

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

#if defined(DATALOGGER_ENABLE) && defined(DATALOGGER_TRIG_TRIP)
    datalogger_trigger = 1; // Trigger datalogger on short circuit trip
#endif
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
  static int16_t speed_l, speed_r;
  int16_t speed_new;
  uint16_t voltage_scale;
  int16_t ia_l, ib_l, ic_l;
  int16_t ia_r, ib_r, ic_r;
  int16_t ref_l, ref_r;
  int16_t ref_ramp_diff;
  int16_t speed_error;
  int16_t torque_ref;
  int16_t torque_lim_speed = INT16_MAX; // Limits motoring (speeding up) torque only, not braking
  int16_t torque_lim_volt = INT16_MAX; // Positive --> motoring (speeding up) limit, negative --> generating (braking) limit
  uint8_t ctrl_mode;
  uint16_t v_bat;
#if defined(BLDC_FIELD_WEAKENING) || defined(SVM_FIELD_WEAKENING) || defined(LEFT_MOTOR_FOC) || defined(RIGHT_MOTOR_FOC)
  uint16_t tref_abs;
#endif

#if defined(LEFT_MOTOR_SVM) || defined(RIGHT_MOTOR_SVM)
  int8_t ref_sign;
#endif

#if defined(LEFT_CURRENT_TFORM) || defined(RIGHT_CURRENT_TFORM)
  uint16_t angle_l = 0;
  uint16_t angle_r = 0;
  int16_t ialpha, ibeta;
  int16_t id = 0;
  int16_t iq = 0;
#endif

#if defined(LEFT_MOTOR_FOC) || defined(RIGHT_MOTOR_FOC)
  int16_t id_error, iq_error;
  int16_t ref_amplitude;
#endif
  int16_t angle_advance;

#if (defined(LEFT_MOTOR_SVM) && !defined(LEFT_MOTOR_FOC)) || (defined(RIGHT_MOTOR_SVM) && !defined(RIGHT_MOTOR_FOC))
  int16_t speed_ctrl;
#endif

  CTRL_TIM->SR = 0;

  //HAL_GPIO_TogglePin(LED_PORT,LED_PIN); // TODO: Debug LED

  if(reset_ctrl) {
    initialize_control_state();
    reset_ctrl = 0;
  }

  // Clear run status bit which is then set if one of the motor is rotating
  status_bits &= ~STATUS_RUN;

  // Update motor speed from latest period information
  // Left motor
  speed_tick[0] = motor_state[STATE_LEFT].act.period;
  if(speed_tick[0] != PERIOD_STOP && speed_tick[0] != -PERIOD_STOP) {
    speed_new = fx_div(motor_nominal_counts, speed_tick[0]);
    status_bits |= STATUS_RUN;
  } else {
    speed_new = 0;
  }
  speed_l = FILTER(speed_new, speed_l, speed_filt_gain);
  motor_state[STATE_LEFT].act.speed = speed_l;
  cfg.vars.speed_l = speed_l; // TODO: Debug

  // Right motor
  speed_tick[1] = motor_state[STATE_RIGHT].act.period;
  if(speed_tick[1] != PERIOD_STOP && speed_tick[1] != -PERIOD_STOP) {
    speed_new = fx_div(motor_nominal_counts, speed_tick[1]);
    status_bits |= STATUS_RUN;
  } else {
    speed_new = 0;
  }
  speed_r = FILTER(speed_new, speed_r, speed_filt_gain);
  motor_state[STATE_RIGHT].act.speed = speed_r;
  cfg.vars.speed_r = speed_r; // TODO: Debug


  // Check overspeed trip
  if(speed_l > OVERSPEED_TRIP || speed_l < -OVERSPEED_TRIP ||
     speed_r > OVERSPEED_TRIP || speed_r < -OVERSPEED_TRIP) {
    do_fault(0x01 | 0x02);	// Trip both motors
    fault_bits |= FAULT_OVERSPEED;

    buzzer_pattern = 0xA8A8;
    buzzer_tone = 0xAA88;
  }


  // Current measurement and overcurrent trips
  if(imeas_calibration_done) {
    __disable_irq();

    // Left motor phase currents and position
    ia_l = i_meas.i_lA;
    ib_l = i_meas.i_lB;
    ic_l = i_meas.i_lC;
#if defined(LEFT_CURRENT_TFORM)
    angle_l = motor_state[STATE_LEFT].act.angle;
#endif

    // Right motor phase currents and position
    ia_r = i_meas.i_rA;
    ib_r = i_meas.i_rB;
    ic_r = i_meas.i_rC;
#if defined(RIGHT_CURRENT_TFORM)
    angle_r = motor_state[STATE_RIGHT].act.angle;
#endif
    __enable_irq();

  } else {
    ia_l = ib_l = ic_l = 0;
    ia_r = ib_r = ic_r = 0;
  }

  // Check if currents exceed overcurrent limits
  // and trip one both motors
  // But only check when we're ready to run, i.e. measurements are stable
  if((status_bits & STATUS_READY) && imeas_calibration_done) {
    if(ia_l > OVERCURRENT_TRIP || ia_l < -OVERCURRENT_TRIP ||
       ib_l > OVERCURRENT_TRIP || ib_l < -OVERCURRENT_TRIP ||
       ic_l > OVERCURRENT_TRIP || ic_l < -OVERCURRENT_TRIP) {
      do_fault(0x01 | 0x02);	// Trip both motors
      fault_bits |= FAULT_OVERCURRENT;

#if defined(DATALOGGER_ENABLE) && defined(DATALOGGER_TRIG_TRIP)
    datalogger_trigger = 1; // Trigger datalogger on overcurrent trip
#endif

      buzzer_tone = 0x92A4;
      buzzer_pattern = 0xC30C;
    }

    if(ia_r > OVERCURRENT_TRIP || ia_r < -OVERCURRENT_TRIP ||
       ib_r > OVERCURRENT_TRIP || ib_r < -OVERCURRENT_TRIP ||
       ic_r > OVERCURRENT_TRIP || ic_r < -OVERCURRENT_TRIP) {
      do_fault(0x01 | 0x02);	// Trip both motors
      fault_bits |= FAULT_OVERCURRENT;

#if defined(DATALOGGER_ENABLE) && defined(DATALOGGER_TRIG_TRIP)
    datalogger_trigger = 1; // Trigger datalogger on overcurrent trip
#endif

      buzzer_tone = 0x92A4;
      buzzer_pattern = 0xC30C;
    }
  }

  // Analog measurements (battery voltage, to be used in modulator)
  v_bat = analog_meas.v_battery + ADC_BATTERY_OFFSET;
  battery_voltage_filt = FILTERU(v_bat << 4, battery_voltage_filt, adc_battery_filt_gain);
  battery_volt_pu = fx_mulu((battery_voltage_filt >> 4), adc_battery_to_pu);

  // Reference scaling so that 1 (4096) results in 1 (motor nominal voltage) always
  // So we scale all references by battery_voltage / nominal voltage
  voltage_scale = fx_divu(FIXED_ONE, battery_volt_pu);


  // Check voltage limits
  // Only trip if everything is ready, e.g. voltage has been filtered long enough etc.
  if(battery_volt_pu > ov_trip_pu && (status_bits & STATUS_READY)) {
    do_fault(0x01 | 0x02);	// Trip both motors
    fault_bits |= FAULT_OVERVOLTAGE;

    buzzer_tone = 0xCCCC;
    buzzer_pattern = 0xFFFF;

#if defined(DATALOGGER_ENABLE) && defined(DATALOGGER_TRIG_TRIP)
    datalogger_trigger = 1; // Trigger datalogger on overvoltage trip
#endif

  } else if(battery_volt_pu > ov_warn_pu) {
    status_bits |= STATUS_OVERVOLTAGE_WARN;

    // Torque limitation
    // At low voltage this results in INT16_MIN (negative), then goes to zero when voltage is increased
    // i.e. allow braking current (that charges battery) only when voltage is low enough
#if defined(OVERVOLTAGE_LIM_GAIN) && OVERVOLTAGE_LIM_GAIN > 0
    torque_lim_volt = CLAMP(battery_volt_pu - ov_warn_pu, ov_lim_min, ov_lim_max) * OVERVOLTAGE_LIM_GAIN;
    torque_lim_volt = CLAMP(torque_lim_volt, INT16_MIN + OVERVOLTAGE_LIM_OFFSET, OVERVOLTAGE_LIM_OFFSET) - OVERVOLTAGE_LIM_OFFSET;
#endif

    buzzer_tone = 0xCCCC;
    buzzer_pattern = 0xAAAA;

  } else if(battery_volt_pu < uv_trip_pu && (status_bits & STATUS_READY)) {
    do_fault(0x01 | 0x02); // Trip both motors
    fault_bits |= FAULT_UNDERVOLTAGE;

    buzzer_tone = 0xF0F0;
    buzzer_pattern = 0xFFFF;

#if defined(DATALOGGER_ENABLE) && defined(DATALOGGER_TRIG_TRIP)
    datalogger_trigger = 1; // Trigger datalogger on undervoltage trip
#endif

  } else if(battery_volt_pu < uv_warn_pu) {
    status_bits |= STATUS_UNDERVOLTAGE_WARN;

    // Torque limitation
    // At high voltage this results in INT16_MAX (positive), then goes to zero when voltage is increased
    // i.e. allows motoring (battery discharging) current only at high enough voltage
#if defined(UNDERVOLTAGE_LIM_GAIN) && UNDERVOLTAGE_LIM_GAIN > 0
    torque_lim_volt = CLAMP(ov_warn_pu - battery_volt_pu, uv_lim_min, uv_lim_max) * UNDERVOLTAGE_LIM_GAIN;
    torque_lim_volt = UNDERVOLTAGE_LIM_OFFSET - CLAMP(torque_lim_volt, INT16_MIN + UNDERVOLTAGE_LIM_OFFSET+1, UNDERVOLTAGE_LIM_OFFSET);
#endif

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
    if(!(status_bits & STATUS_READY) && battery_volt_pu > uv_warn_pu)
      status_bits |= STATUS_READY;
  }


  // Handle reference & speed & torque control ONLY if we're ready
  // and current measurement has been calibrated
  if((status_bits & STATUS_READY) && imeas_calibration_done)
  {

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
#elif defined(REFERENCE_ADC_EBIKE)
    // Ebike style reference which only allows reference to one
    // direction, and re-generation (braking) only, not reverse
    ref_l = ((int16_t)analog_meas.analog_ref_1 - 512);		// Below about 10% ref. use regenerative braking
    ref_r = ref_l;

    // Only allow regeneration, not reversing
    // Currently enabled only above roughly 5 % of rated motor speed
    if(speed_l <= SPEED_SLOW && ref_l < 0) {
      ref_l = 0;
      ref_l_ramp = 0;	// Also force the ramped reference immediately to zero, otherwise it may take long to ramp down...
    }
    if(speed_r <= SPEED_SLOW && ref_r < 0) {
      ref_r = 0;
      ref_r_ramp = 0; // Also force ramped to zero
    }

#else // Different analog references for both motors
    // ADC output is 0...4095, scale it to -4096 ... 4095
    // TODO: Add some configuration (offset, gain, deadband) to these?
    ref_l = ((int16_t)analog_meas.analog_ref_1 - 2048) * 2;
    ref_r = ((int16_t)analog_meas.analog_ref_2 - 2048) * 2;
#endif // REFERENCE_ADC_X
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


    // Clear field weakening status bit, which is then set if one of
    // the motors is operating in field weakening region
    status_bits &= ~STATUS_FIELD_WEAK;


    // --------------
    // Left motor

    // Speed control loop for left motor
    ctrl_mode = motor_state[STATE_LEFT].ref.control_mode;

    if(ctrl_mode == CONTROL_SPEED) {
      // FOC and BLCD in speed control mode --> run PI controller
      speed_error = motor_state[STATE_LEFT].ref.value - speed_l;
      speed_error_int_l += speed_error / speed_int_divisor;
      speed_error_int_l = LIMIT(speed_error_int_l, speed_int_max);
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
      torque_lim_speed = CLAMP(ABS(speed_l) - OVERSPEED_LIMIT, overspeed_lim_min, overspeed_lim_max) * OVERSPEED_LIM_GAIN;
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

    cfg.vars.t_req_l = torque_ref;

#if defined(LEFT_CURRENT_TFORM)
    if(imeas_calibration_done) {
      // Transform to rotor coordinate frame
      clarke(ia_l, ib_l, &ialpha, &ibeta);
      park(ialpha, ibeta, angle_l, &id, &iq);

      // Filter id and iq
      id = FILTER(id, id_old_l, cfg.vars.i_filter);
      iq = FILTER(iq, iq_old_l, cfg.vars.i_filter);
      id_old_l = id;
      iq_old_l = iq;

      cfg.vars.rdsonla = ia_l;
      cfg.vars.rdsonlb = ib_l;
    }
#endif


#if defined(LEFT_MOTOR_FOC)

    // Run the PI controllers...
    // ... for Q axis current which sets the reference amplitude
    iq_error = torque_ref - iq;

    iq_error_int_l += iq_error / int_divisor;	// Slow down integrator rise time
    iq_error_int_l = LIMIT(iq_error_int_l, idq_int_max);
    ref_amplitude = fx_mul(iq_error, kp_iq) + fx_mul(iq_error_int_l, ki_iq);

    ref_sign = ISIGN(ref_amplitude);

    // Apply DC voltage scaling
    ref_amplitude = fx_mul(ref_amplitude, voltage_scale);

    // Apply limiter
    tref_abs = ABS(ref_amplitude);
    ref_amplitude = MIN(tref_abs, cfg.vars.max_pwm_l);


    // ... for D axis current which sets the angle advance
    id_error = id;		// Try to set d axis current to zero

#if defined(FOC_FIELD_WEAKENING)
    if(tref_abs > cfg.vars.max_pwm_l) {
      tref_abs -= cfg.vars.max_pwm_l;
      tref_abs = CLAMP(tref_abs, 0, MOTOR_MAX_ID);
      id_error = id + tref_abs;		// Excess amplitude request will be directly d axis current
    }
#endif

    id_error_int_l += id_error / int_divisor;
    id_error_int_l = LIMIT(id_error_int_l, idq_int_max);
    angle_advance = fx_mul(id_error, kp_id) + fx_mul(id_error_int_l, ki_id);
    angle_advance *= 8;	// From 12-bit fixed point to 16-bit angle => 1 = 4096 = one full rotation

    cfg.vars.pwm_l = ref_amplitude;
    cfg.vars.l_angle_adv = angle_advance;


    // Apply references
    __disable_irq();
    motor_state[STATE_LEFT].ctrl.amplitude = (uint16_t)ref_amplitude;
    motor_state[STATE_LEFT].ctrl.angle = ref_sign * ((uint16_t)angle_advance + ANGLE_100DEG);	// Start with 100 degree phase shift to guarantee starting
    __enable_irq();

    //torque_ref = ref_amplitude; // TODO: For debugging purposes only

#elif defined(LEFT_MOTOR_SVM) && !defined(LEFT_MOTOR_FOC)
    if(ctrl_mode == CONTROL_UF) {
      // Simple U/F control where voltage and rotation speed are set manually
      torque_ref =  fx_mul(motor_state[STATE_LEFT].ref.value, voltage_scale);
      speed_ctrl = ISIGN(torque_ref);
      torque_ref = ABS(torque_ref);

      speed_ctrl *= (ANGLE_60DEG * motor_state[STATE_LEFT].ref.value) / (FIXED_ONE * motor_nominal_counts);

      __disable_irq();
      motor_state[STATE_LEFT].ctrl.amplitude = torque_ref;
      motor_state[STATE_LEFT].ctrl.speed = speed_ctrl;
      __enable_irq();
    } else if(ctrl_mode == CONTROL_ANGLE) {
      __disable_irq();
      motor_state[STATE_LEFT].ctrl.speed = 0;
      motor_state[STATE_LEFT].ctrl.angle = motor_state[STATE_LEFT].ref.value;
      motor_state[STATE_LEFT].ctrl.amplitude = IR_MINIMUM_VOLTAGE * 1.5;
      __enable_irq();
    } else {
      // Torque or speed control mode with SVM
      // Scale ramped reference according to DC voltage
      torque_ref = fx_mul(torque_ref, voltage_scale);

      // Use 100 degrees phase shift by default which seems to make quiet operation and easy start
      angle_advance = ANGLE_100DEG;

#if defined(SVM_FIELD_WEAKENING)
      tref_abs = ABS(torque_ref);
      if(tref_abs > cfg.vars.max_pwm_l) {
        tref_abs -= cfg.vars.max_pwm_l; // Excess voltage request is applied as phase advance
        tref_abs = CLAMP(tref_abs, 0, FIXED_ONE);
        tref_abs = fx_mul(tref_abs, ANGLE_45DEG);	// Max 45 degrees of advance
        angle_advance += tref_abs;
        status_bits |= STATUS_FIELD_WEAK;
      }
#endif
      cfg.vars.l_angle_adv = angle_advance;

      // Limit pwm value
      torque_ref = LIMIT(torque_ref, cfg.vars.max_pwm_l);
      ref_sign = ISIGN(torque_ref);
      torque_ref = ABS(torque_ref);

      cfg.vars.pwm_l = torque_ref;

      // Apply the reference
      __disable_irq();
      motor_state[STATE_LEFT].ctrl.amplitude = torque_ref;
      motor_state[STATE_LEFT].ctrl.angle = ref_sign * angle_advance;
      __enable_irq();
    }



#elif defined(LEFT_MOTOR_BLDC)
    // Torque (voltage) control of left motor in BLDC mode

    // Scale ramped reference according to DC voltage
    torque_ref = fx_mul(torque_ref, voltage_scale);

#if defined(BLDC_FIELD_WEAKENING)
    // Apply field weakening if necessary
    tref_abs = ABS(torque_ref);
    if(tref_abs > cfg.vars.max_pwm_l) {
      // "Excess" torque reference
      tref_abs = tref_abs - cfg.vars.max_pwm_l;
      status_bits |= STATUS_FIELD_WEAK;
    } else {
      tref_abs = 0;
    }

    // Limit the torque reference to one (full modulation amplitude)
    torque_ref = LIMIT(torque_ref, cfg.vars.max_pwm_l);

    cfg.vars.pwm_l = torque_ref;

    __disable_irq();
    // Excess is directly applied as the field weakening ref
    motor_state[STATE_LEFT].ctrl.angle = tref_abs;
    motor_state[STATE_LEFT].ctrl.amplitude = torque_ref;
    __enable_irq();
#else
    // Limit pwm value
    torque_ref = LIMIT(torque_ref, cfg.vars.max_pwm_l);

    // Apply the reference
    __disable_irq();
    motor_state[STATE_LEFT].ctrl.amplitude = torque_ref;
    __enable_irq();
#endif

#endif // LEFT_MOTOR_BLDC




    // ------------
    // Right motor

    ctrl_mode = motor_state[STATE_RIGHT].ref.control_mode;

    if(ctrl_mode == CONTROL_SPEED) {
      // Speed control loop for right motor
      // FOC and BLCD in speed mode --> run PI controller
      speed_error = motor_state[STATE_RIGHT].ref.value - speed_r;
      speed_error_int_r += speed_error / speed_int_divisor;
      speed_error_int_r = LIMIT(speed_error_int_r, speed_int_max);
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
      torque_lim_speed = CLAMP(ABS(speed_r) - OVERSPEED_LIMIT, overspeed_lim_min, overspeed_lim_max) * OVERSPEED_LIM_GAIN;
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


    cfg.vars.t_req_r = torque_ref; // TODO: DEBUG



#if defined(RIGHT_CURRENT_TFORM)
    if(imeas_calibration_done) {
      // Transform to rotor coordinate frame
      clarke(ia_r, ib_r, &ialpha, &ibeta);
      park(ialpha, ibeta, angle_r, &id, &iq);

      // Filter id and iq
      id = FILTER(id, id_old_r, cfg.vars.i_filter);
      iq = FILTER(iq, iq_old_r, cfg.vars.i_filter);
      id_old_r = id;
      iq_old_r = iq;

      cfg.vars.rdsonra = ia_r;
      cfg.vars.rdsonrb = ib_r;
    }
#endif



#if defined(RIGHT_MOTOR_FOC)

    // Run the PI controllers...
    // ... for Q axis current which sets the reference amplitude
    iq_error = torque_ref - iq;

    iq_error_int_r += iq_error;
    iq_error_int_r = LIMIT(iq_error_int_r, idq_int_max);
    ref_amplitude = fx_mul(iq_error, kp_iq) + fx_mul(iq_error_int_r, ki_iq);

    ref_sign = ISIGN(ref_amplitude);

    // Apply DC voltage scaling
    ref_amplitude = fx_mul(ref_amplitude, voltage_scale);

    // Apply limiter
    tref_abs = ABS(ref_amplitude);
    ref_amplitude = MIN(tref_abs, cfg.vars.max_pwm_r);


    // ... for D axis current which sets the angle advance
    id_error = id;		// Try to set d axis current to zero

#if defined(FOC_FIELD_WEAKENING)
    if(tref_abs > cfg.vars.max_pwm_r) {
      tref_abs -= cfg.vars.max_pwm_r;
      tref_abs = CLAMP(tref_abs, 0, MOTOR_MAX_ID);
      id_error = id + tref_abs;		// Excess amplitude request will be directly d axis current
    }
#endif

    id_error_int_r += id_error / int_divisor;
    id_error_int_r = LIMIT(id_error_int_r, idq_int_max);
    angle_advance = fx_mul(id_error, kp_id) + fx_mul(id_error_int_r, ki_id);
    angle_advance *= 8;

    cfg.vars.r_angle_adv = angle_advance;


    // Apply references
    __disable_irq();
    motor_state[STATE_RIGHT].ctrl.amplitude = (uint16_t)ref_amplitude;
    motor_state[STATE_RIGHT].ctrl.angle = ref_sign * ((uint16_t)angle_advance + ANGLE_100DEG);
    __enable_irq();

    //torque_ref = ref_amplitude; // TODO: Debug


#elif defined(RIGHT_MOTOR_SVM)  && !defined(RIGHT_MOTOR_FOC)
    if(ctrl_mode == CONTROL_UF) {
      // Simple U/F control where voltage and rotation speed are set manually
      torque_ref =  fx_mul(motor_state[STATE_RIGHT].ref.value, voltage_scale);
      speed_ctrl = ISIGN(torque_ref);
      torque_ref = ABS(torque_ref);

      speed_ctrl *= (ANGLE_60DEG * motor_state[STATE_RIGHT].ref.value) / (FIXED_ONE * motor_nominal_counts);

      __disable_irq();
      motor_state[STATE_RIGHT].ctrl.amplitude = torque_ref;
      motor_state[STATE_RIGHT].ctrl.speed = speed_ctrl;
      __enable_irq();
    } else if(ctrl_mode == CONTROL_ANGLE) {
      // Direct angle control for debugging purposes
      __disable_irq();
      motor_state[STATE_RIGHT].ctrl.speed = 0;
      motor_state[STATE_RIGHT].ctrl.angle = motor_state[STATE_RIGHT].ref.value;
      motor_state[STATE_RIGHT].ctrl.amplitude = IR_MINIMUM_VOLTAGE * 1.5;
      __enable_irq();
    } else {
      // Torque or speed control mode with SVM
      // Scale ramped reference according to DC voltage
      torque_ref = fx_mul(torque_ref, voltage_scale);

      // Use 100 degrees phase shift by default which seems to make quiet operation and easy start
      angle_advance = ANGLE_100DEG;

#if defined(SVM_FIELD_WEAKENING)
      tref_abs = ABS(torque_ref);
      if(tref_abs > cfg.vars.max_pwm_r) {
        tref_abs -= cfg.vars.max_pwm_r; // Excess voltage request is applied as phase advance
        tref_abs = CLAMP(tref_abs, 0, FIXED_ONE);
        tref_abs = fx_mul(tref_abs, ANGLE_45DEG);	// Max 45 degrees of advance
        angle_advance += tref_abs;
        status_bits |= STATUS_FIELD_WEAK;
      }
#endif

      cfg.vars.r_angle_adv = angle_advance;

      // Limit pwm value
      torque_ref = LIMIT(torque_ref, cfg.vars.max_pwm_r);
      ref_sign = ISIGN(torque_ref);
      torque_ref = ABS(torque_ref);

      // Apply the reference
      __disable_irq();
      motor_state[STATE_RIGHT].ctrl.amplitude = torque_ref;
      motor_state[STATE_RIGHT].ctrl.angle = ref_sign * angle_advance;
      __enable_irq();
    }

#elif defined(RIGHT_MOTOR_BLDC)
    // Torque (voltage) control of left motor in BLDC mode

    // Scale ramped reference according to DC voltage
    torque_ref = fx_mul(torque_ref, voltage_scale);

#if defined(BLDC_FIELD_WEAKENING)
    // Apply field weakening if necessary
    tref_abs = ABS(torque_ref);
    if(tref_abs > cfg.vars.max_pwm_r) {
      // "Excess" torque reference
      tref_abs = tref_abs - cfg.vars.max_pwm_r;
      status_bits |= STATUS_FIELD_WEAK;
    } else {
      tref_abs = 0;
    }

    // Limit the torque reference to one (full modulation amplitude)
    torque_ref = LIMIT(torque_ref, cfg.vars.max_pwm_r);

    // Excess is directly applied as the field weakening ref
    __disable_irq();
    motor_state[STATE_RIGHT].ctrl.angle = tref_abs;
    motor_state[STATE_RIGHT].ctrl.amplitude = torque_ref;
    __enable_irq();

#else
    // Limit pwm reference to maximum limits
    torque_ref = LIMIT(torque_ref, cfg.vars.max_pwm_r);

    // Apply the reference
    __disable_irq();
    motor_state[STATE_RIGHT].ctrl.amplitude = torque_ref;
    __enable_irq();
#endif

#endif // RIGHT_MOTOR_BLDC


  } // End status ready &  imeas calibration done


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


  motor_state[STATE_LEFT].act.current[0] = ia_l; //i_meas.i_lA;
  motor_state[STATE_LEFT].act.current[1] = ib_l; //i_meas.i_lB;
  motor_state[STATE_LEFT].act.current[2] = ic_l; //-i_meas.i_lA - i_meas.i_lB;
  motor_state[STATE_RIGHT].act.current[0] = ia_r; //-i_meas.i_lB - i_meas.i_rC;
  motor_state[STATE_RIGHT].act.current[1] = ib_r; //i_meas.i_rB;
  motor_state[STATE_RIGHT].act.current[2] = ic_r; //i_meas.i_rC;


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

  //HAL_GPIO_TogglePin(LED_PORT,LED_PIN); // TODO: Debug LED
}

#pragma once
#include "stm32f1xx_hal.h"


// =============================
// Features

// Configure which feature is enabled on the right sensor board connection
#define LEFT_SENSOR_MODBUS			// Enable modbus (UART3)
// #define LEFT_SENSOR_SCOPE		// Enable uart scope on left sensor (UART3)

// Configure which feature is enabled on the left sensor board connection
//#define RIGHT_SENSOR_MODBUS		// Enable modbus (UART2)
#define RIGHT_SENSOR_ANALOG			// Enable analog inputs
// #define RIGHT_SENSOR_SCOPE		// Enable uart scope on right sensor (UART2)

// Configure power button operation
#define POWER_BUTTON_NORMAL			// "Normal" operation: press twice for on, press twice for off, long press resets
//#define POWER_BUTTON_ESTOP		// Emergency stop opeartion: Must be pressed for power on, release for shutdown


// =============================
// Motor parameters
#define MOTOR_VOLTS				18.0 //15.0	// Volts (phase) at rated speed (RMS) (10 rounds/sec, ~20V amplitude)
#define MOTOR_SPEED				600.0	// Nominal speed rpm
#define MOTOR_POLEPAIRS			15.0	// Polepairs, mechanical speed to electrical speed (15 electrical rounds/1 mechanical round)
#define MOTOR_CUR				15.0	// [A] at rated load, phase (RMS)
#define MOTOR_CIRCUMFERENCE		0.534	// meters, motor outside circumference

#define MOTOR_MIN_VOLTS			0.5		// Volts to apply at zero and low speed, to get the motor started (IR compensation)



// =============================
// Control methods

// Left motor
// What control method to use for which motor
//#define LEFT_MOTOR_BLDC		// Use BLDC for left motor
//#define LEFT_MOTOR_SVM			// Use SVM for left motor
#define LEFT_MOTOR_FOC			// Use field oriented control for left motor (requires SVM also)

// Right motor
//#define RIGHT_MOTOR_BLDC		// BLDC for right motor
//#define RIGHT_MOTOR_SVM			// SVM for right motor
#define RIGHT_MOTOR_FOC			// Use field oriented control for right motor (requires SVM also)


// Current measurement using Rds,on
#define I_MEAS_RDSON

// Convert measured currents to id & iq
// These are required for FOC but can be enabled also for other modes
#define LEFT_CURRENT_TFORM
#define RIGHT_CURRENT_TFORM


// Use field weakening region in BLDC mode
//#define BLDC_FIELD_WEAKENING

// Controller parameters
#define CONTROL_FREQ	1000		// Controller is run at this rate
#define CONTROL_PERIOD	(64000000 / CONTROL_FREQ)

// =============================
// Reference source
//#define REFERENCE_MODBUS			// Use modbus for references
#define REFERENCE_ADC				// Use analog inputs for reference
//#define REFERENCE_ADC_DIFF		// Differential ADC inputs, requires ADC reference
//#define REFERENCE_ADC_SINGLE		// Use only single channel ADC ref for both sides
#define REFERENCE_ADC_EBIKE		// Ebike style analog reference with re-generation

// =============================
// Limits/warnings/faults

#define OVERVOLTAGE_WARN	40		// Overvoltage warning level in volts
#define OVERVOLTAGE_TRIP	41		// Overvoltage trip level in volts
#define UNDERVOLTAGE_WARN	32		// Undervoltage warning level in volts
#define UNDERVOLTAGE_TRIP	30		// Undervoltage trip level in volts

#define OVERVOLTAGE_LIM_OFFSET	(FIXED_ONE)
//#define OVERVOLTAGE_LIM_GAIN	4	// Gain of braking torque limitation in per-units when voltage > overvoltage warn (TODO: causes SHAKING)

#define UNDERVOLTAGE_LIM_OFFSET	(FIXED_ONE)
//#define UNDERVOLTAGE_LIM_GAIN	4	// Gain of motoring torque limitation in per-units when voltage < undervoltage warn (TODO: causes SHAKING)


#define OVERCURRENT_TRIP	(3.0*FIXED_ONE)		// Overcurrent trip compared to motor nominal current

#define OVERSPEED_TRIP		(1.8*FIXED_ONE)		// Compared to rated rotation speed

#define OVERSPEED_LIMIT		(1.4*FIXED_ONE)		// Start limiting torque above this, also speed control max
//#define OVERSPEED_LIM_GAIN	4					// Gain of how much torque is removed above the overspeed limit (TODO: causes SHAKING)
#define OVERSPEED_LIM_OFFSET	(FIXED_ONE)



// =============================
// Other parameters

// Enable this to force config bus to load defaults instead of reading from EEPROM
// Note: EEPROM (or write to it) does not currently seem to work, so keep this for now!
//#define CFGBUS_FORCE_DEFAULTS

// HALL sensor wiring
//#define HALL_GBYGBY					// Motor wiring U:Green, V:Blue, W:Yellow, board U:G, V:B, W:Y
#define HALL_GBYBGY						// Motor wiring U:Green, V:Blue, W:Yellow, board U:B, V:G, W:Y (blue/green crossed in connector)

// Modulation parameters
#define PWM_FREQ         16000		// PWM frequency in Hz

#define DEAD_TIME        	32		// PWM deadtime (see STM32F103 datasheet for values)
#define BLDC_SHORT_PULSE	112		// Shortest pulse length in BLDC, in 1/64 MHz (112 = 1.75 us)

#define SVM_SHORT_PULSE		0		// Shortest active pulse length, in 1/64 MHz (64 = 1 us)
#define SVM_SHORT_ZPULSE	112		// Minimum zero pulse length, in 1/64 MHz (64 = 1 us, 112 = 1.75 us). This should be the time required by rdson current meas
#define SVM_DEAD_TIME_COMP	10		// 64 = 1us


// ADC calibration values
// ADC battery reading to volts
#define ADC_BATTERY_VOLTS   0.024	// Approximately: (3.3 V / 4096) * (15k+15k+1k) / 1k, but calibrated by hand
#define ADC_BATTERY_OFFSET	102		// ADC offset when reading zero volts, in ADC units

// How many times to sample ADC to get offsets
#define ADC_OFFSET_SAMPLES		1024

// How much offset between left and right timers so that
// currents of both sides is sampled at the middle of lower zero vector
// Note! if DUAL_ADC_MODE is not enabled, this should be twice the value here
#define TIMER_OFFSET_FOR_ADC	(14*8) // 1.5 cycles sampling + 12.5 cycles conversion times clock divider of 8 (see AdcClockSelection in main.c)

// Enable dual ADC mode for current measurement
#define DUAL_ADC_MODE

// Enable this to use DPWMMIN modulation method, i.e. use only lower zero 000, never upper zero 111
#define DPWMMIN

// Power button pressed threshold
// The measurement has a gain of about 0.06 until battery voltage is 21 V, then the gain drops
// gradually to about 0.04.
// Power button threshold of about 18 V (required for supply to operate properly) is about 1340 in ADC units
#define ADC_POWERSW_THRESHOLD	1340
// Should be higher than undervoltage limit. Scale is roughly V_meas * 0.056 / 3.3 V * 4095 ~= V_meas * 70
//#define ADC_POWERSW_THRESHOLD	((UNDERVOLTAGE_WARN) * 70)

// Power switch time that it is allowed to be in off-state before next press (in control timer units)
#define POWERSW_OFF_TIMER		800
// Power switch time that it has to be pressed the second time before we really turn on
#define POWERSW_ON_TIMER		1200

// Speed limit for normal power off, in fixed point per unit
#define POWEROFF_SPEED_LIMIT	200		// About 5 % of rated speed

// E-stop powerbutton sample count before emergency shutdown
#define POWERSW_ESTOP_SAMPLES	3

// How long power button must be pressed for fault reset
#define POWERSW_FAULT_RESET		3000	// About 3 seconds

// Mosfet Rds,on, e.g. equivalent resistance in on-state
// This is the equivalent value taking into account the voltage measurement
// gain of about 2.5 ... 3 => if Rdson is about 0.0056 ohm, the value is 0.0056 * 3 = 0.017
// Increased a bit because seems to be more accurate that way
#define RDSON		0.019


// ============================
// Datalogger
#define DATALOGGER_ENABLE
#define DATALOGGER_MAX			0xFF		// Size of dataloggger, how many samples times n variables
#define DATALOGGER_TYPE			uint16_t	// Type of the variables the datalogger can store
#define DATALOGGER_COUNT_TYPE	uint8_t		// Type of counter variable to hold write offset, must hold DATALOGGER_MAX
#define DATALOGGER_DIVIDER		0			// Divide PWM rate by this+1 to get datalogger sampling rate
#define DATALOGGER_CHANNELS		8			// How many channels to sample
#define DATALOGGER_TRIG_TRIP				// If defined, datalogger will trigger to (some) trips
#define DATALOGGER_SAMPLES_AFTER	32		// If defined, datalogger will roll continuously and sample this amount after the trigger


// =============================
// Feature configuration

#ifdef LEFT_SENSOR_MODBUS
#define CFG_BUS_UART (UARTCh3)
#else
#define CFG_BUS_UART (UARTCh2)	// To build without errors, Uart must be defined even
#endif							// though it will not be used

#ifdef RIGHT_SENSOR_SCOPE
#define SCOPE_UART UARTCh2
#endif

#ifdef LEFT_SENSOR_SCOPE
#define SCOPE_UART UARTCh3
#endif


// =============================
// Calculations

// Calculated motor parameters
#define MOTOR_NOMINAL_VOLTS		(MOTOR_VOLTS * 2.45)	// Phase voltage RMS * sqrt(3) * sqrt(2)
#define MOTOR_VOLTS_AT_ZERO		(MOTOR_MIN_VOLTS * 2.45)
#define MOTOR_VOLTS_AT_ZERO_PU	(4096.0 * MOTOR_MIN_VOLTS / MOTOR_VOLTS)	// In per-unit values
// HALL sector change duration at nominal speed, in milliseconds (TODO: is about 1 ms so might change to us)
#define MOTOR_NOMINAL_PERIOD	(1000.0 * 60.0 / (MOTOR_SPEED * MOTOR_POLEPAIRS * 6.0))
#define MOTOR_VOLTS_PER_HZ		(MOTOR_VOLTS * 60.0 / (MOTOR_SPEED * MOTOR_POLEPAIRS))

// IR compensation (for SVM, U/F modulation). Sets the minimum voltage applied at low speeds, fixed point per unit
#define IR_MINIMUM_VOLTAGE		400		// Approximately 5 % TODO: Increased from 200 to about 500 --> terrible... down to 100

#define PWM_PERIOD       (64000000 / 2 / PWM_FREQ)

#define SVM_LONG_PULSE (PWM_PERIOD - SVM_SHORT_PULSE - SVM_SHORT_ZPULSE)	// Maximum pulse length


// Sanity checks here

// Only one modulation method can be enabled
#if defined(LEFT_MOTOR_BLDC) && defined(LEFT_MOTOR_SVM)
#error config.h: Only one modulation method can be active for left motor
#endif

#if defined(RIGHT_MOTOR_BLDC) && defined(RIGHT_MOTOR_SVM)
#error config.h: Only one modulation method can be active for right motor
#endif

// Only one side can handle config bus (modbus)
#if defined(LEFT_SENSOR_MODBUS) && defined(RIGHT_SENSOR_MODBUS)
#error config.h: Only one sensor configuration can have modbus enabled
#endif

// Only one side can be scope
#if defined(LEFT_SENSOR_SCOPE) && defined(RIGHT_SENSOR_SCOPE)
#error config.h: Only one sensor can have scope enabled
#endif

// Check overlapping features
#if defined(LEFT_SENSOR_MODBUS) && defined(LEFT_SENSOR_SCOPE)
#error config.h: Left sensor can only have one feature enabled, not modbus and scope
#endif
#if defined(RIGHT_SENSOR_MODBUS) && defined(RIGHT_SENSOR_SCOPE)
#error config.h: Right sensor can only have one feature enabled, not modbus and scope
#endif
#if defined(RIGHT_SENSOR_MODBUS) && defined(RIGHT_SENSOR_ANALOG)
#error config.h: Right sensor can only have one feature enabled, not modbus and analog
#endif
#if defined(RIGHT_SENSOR_SCOPE) && defined(RIGHT_SENSOR_ANALOG)
#error config.h: Right sensor can only have one feature enabled, not scope and analog
#endif

#if defined(REFERENCE_MODBUS) && defined(REFERENCE_ADC)
#error config.h: Only one reference source can be selected
#endif
#if defined(REFERENCE_ADC_DIFF) && !defined(REFERENCE_ADC)
#warning config.h: Adc differential reference selected without ADC reference
#endif

#if defined(LEFT_MOTOR_FOC)
#define LEFT_MOTOR_SVM
#define LEFT_CURRENT_TFORM
#endif

#if defined(RIGHT_MOTOR_FOC)
#define RIGHT_MOTOR_SVM
#define RIGHT_CURRENT_TFORM
#endif


#if defined(LEFT_MOTOR_FOC) && !defined(LEFT_MOTOR_SVM)
#error config.h: Left motor FOC control requires SVM
#endif

#if defined(RIGHT_MOTOR_FOC) && !defined(RIGHT_MOTOR_SVM)
#error config.h: Right motor FOC requires SVM
#endif

#pragma once
#include "stm32f1xx_hal.h"


// =============================
// Features

// Configure which feature is enabled on the right sensor board connection
#define LEFT_SENSOR_MODBUS			// Enable modbus (UART3)
// #define LEFT_SENSOR_SCOPE		// Enable uart scope on left sensor (UART3)

// Configure which feature is enabled on the left sensor board connection
//#define RIGHT_SENSOR_MODBUS		// Enable modbus (UART2)
// #define RIGHT_SENSOR_ANALOG			// Enable analog inputs
#define RIGHT_SENSOR_SCOPE		// Enable uart scope on right sensor (UART2)

// Configure power button operation
#define POWER_BUTTON_NORMAL			// "Normal" operation: press twice for on, press twice for off, long press resets
//#define POWER_BUTTON_ESTOP		// Emergency stop opeartion: Must be pressed for power on, release for shutdown


// =============================
// Motor parameters
#define MOTOR_VOLTS				15.0	// Volts (phase) at rated speed (RMS) (10 rounds/sec, ~20V amplitude)
#define MOTOR_SPEED				600.0	// Nominal speed rpm
#define MOTOR_POLEPAIRS			15.0	// Polepairs, mechanical speed to electrical speed (15 electrical rounds/1 mechanical round)
#define MOTOR_CUR				5.0		// TODO: [A] at rated load / phase (RMS)
#define MOTOR_CIRCUMFERENCE		0.534	// meters, motor outside circumference

#define MOTOR_MIN_VOLTS			0.5		// Volts to apply at zero and low speed, to get the motor started (IR compensation)



// =============================
// Control methods

// Left motor
#define LEFT_MOTOR_BLDC			// Use BLDC for left motor
//#define LEFT_MOTOR_SVM		// Use SVM for left motor

// Right motor
//#define RIGHT_MOTOR_BLDC		// BLDC for right motor
#define RIGHT_MOTOR_SVM			// SVM for right motor

// For space vector modulation, update reference angle
// and speed from hall sensor data
//#define SVM_HALL_UPDATE	1


// Current measurement using Rds,on
#define I_MEAS_RDSON


// =============================
// Limits/warnings/faults

#define OVERVOLTAGE_WARN	26		// Overvoltage warning level in volts
#define OVERVOLTAGE_TRIP	28		// Overvoltage trip level in volts
#define UNDERVOLTAGE_WARN	22		// Undervoltage warning level in volts
#define UNDERVOLTAGE_TRIP	20		// Undervoltage trip level in volts

#define OVERCURRENT_TRIP	(1.0*FIXED_ONE)		// Overcurrent trip compared to motor nominal current


// =============================
// Other parameters

// Modulation parameters
#define PWM_FREQ         16000		// PWM frequency in Hz

#define DEAD_TIME        	32		// PWM deadtime (see ST32F103 datasheet for values)
#define BLDC_SHORT_PULSE	10		// Shortest pulse length in BLDC, in 1/64 MHz

#define SVM_SHORT_PULSE		30		// Shortest active pulse length, in 1/64 MHz
#define SVM_SHORT_ZPULSE	30		// Minimum zero pulse length, in 1/64 MHz
#define SVM_DEAD_TIME_COMP	10		// 64 = 1us


// ADC calibration values
// ADC reading to volts
#define ADC_BATTERY_VOLTS   0.02444	// Approximately: (3.3 V / 4096) * (15k+15k+1k) / 1k, but calibrated by hand
#define ADC_BATTERY_OFFSET	87		// ADC offset when reading zero volts, in ADC units

// How many times to sample ADC to get offsets
#define ADC_OFFSET_SAMPLES		1024

// Power button pressed threshold
// The measurement has a gain of about 0.06 until battery voltage is 21 V, then the gain drops
// gradually to about 0.04.
// Power button threshold of about 18 V (required for supply to operate properly) is about 1340 in ADC units
#define ADC_POWERSW_THRESHOLD	1340

// Power switch time that it is allowed to be in off-state before next press (in control timer units)
#define POWERSW_OFF_TIMER		1000
// Power switch time that it has to be pressed the second time before we really turn on
#define POWERSW_ON_TIMER		2000

// Speed limit for normal power off, in fixed point per unit
#define POWEROFF_SPEED_LIMIT	200		// About 5 % of rated speed

// E-stop powerbutton sample count before emergency shutdown
#define POWERSW_ESTOP_SAMPLES	3

// How long power button must be pressed for fault reset
#define POWERSW_FAULT_RESET		3000	// About 3 seconds

// Mosfet Rds,on, e.g. equivalent resistance in on-state
// This is the equivalent value taking into account the voltage measurement
// gain of about 2.5 ... 3 => if Rdson is about 0.0056 ohm, the value is 0.0056 * 3 = 0.017
#define RDSON		0.017


// =============================
// Feature configuration

#ifdef LEFT_SENSOR_MODBUS
#define CFG_BUS_UART (UARTCh3)
#endif

#ifdef RIGHT_SENSOR_MODBUS
#define CFG_BUS_UART (UARTCh2)
#endif

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

#pragma once
#include "stm32f1xx_hal.h"

// Motor parameters
#define MOTOR_VOLTS				15.0	// Volts (phase) at rated speed (RMS) (10 rounds/sec, ~20V amplitude)
#define MOTOR_SPEED				600.0	// Nominal speed rpm
#define MOTOR_POLEPAIRS			15.0	// Polepairs, mechanical speed to electrical speed (15 electrical rounds/1 mechanical round)
#define MOTOR_CUR				5.0		// TODO: [A] at rated load / phase (RMS)
#define MOTOR_CIRCUMFERENCE		0.534	// meters, motor outside circumference

#define MOTOR_MIN_VOLTS			0.5		// Volts to apply at zero and low speed, to get the motor started (IR compensation)

// Calculated motor parameters
#define MOTOR_NOMINAL_VOLTS		(MOTOR_VOLTS * 2.45)	// Phase voltage RMS * sqrt(3) * sqrt(2)
#define MOTOR_VOLTS_AT_ZERO		(MOTOR_MIN_VOLTS * 2.45)
#define MOTOR_VOLTS_AT_ZERO_PU	(4096.0 * MOTOR_MIN_VOLTS / MOTOR_VOLTS)	// In per-unit values
// HALL sector change duration at nominal speed, in milliseconds (TODO: is about 1 ms so might change to us)
#define MOTOR_NOMINAL_PERIOD	(1000.0 * 60.0 / (MOTOR_SPEED * MOTOR_POLEPAIRS * 6.0))
#define MOTOR_VOLTS_PER_HZ		(MOTOR_VOLTS * 60.0 / (MOTOR_SPEED * MOTOR_POLEPAIRS))


#define MOTOR_PERIOD_TO_MS		0.5874	// Motor sector change period in ms to m/s speed



// What control method to use for which motor
#define LEFT_MOTOR_BLDC		// Use BLDC for left motor
//#define LEFT_MOTOR_SVM			// Use SVM for left motor

//#define RIGHT_MOTOR_BLDC		// BLDC for right motor
#define RIGHT_MOTOR_SVM			// SVM for right motor


//#define SVM_HALL_UPDATE	1		// Update reference position from HALL sensors or not
#undef SVM_HALL_UPDATE


// Modulation parameters
#define PWM_FREQ         16000		// PWM frequency in Hz
#define PWM_PERIOD       (64000000 / 2 / PWM_FREQ)

#define DEAD_TIME        	32		// PWM deadtime (see ST32F103 datasheet)
#define BLDC_SHORT_PULSE	10		// Shortest pulse length in BLDC, in 1/64 MHz
//#define SVM_SHORT_PULSE		192		// in 1/64 MHz, 192 = 3 us
//#define SVM_SHORT_ZPULSE	86		// Minimum zero pulse length, in 1/64 MHz
#define SVM_SHORT_PULSE		30		// in 1/64 MHz
#define SVM_SHORT_ZPULSE	30		// Minimum zero pulse length, in 1/64 MHz
#define SVM_LONG_PULSE (PWM_PERIOD - SVM_SHORT_PULSE - SVM_SHORT_ZPULSE)	// Maximum pulse length

#define SVM_DEAD_TIME_COMP	10		// 64 = 1us

// Limits
#define DC_CUR_LIMIT     1         // Motor DC current limit in amps
#define PPM_NUM_CHANNELS 6         // number of PPM channels to receive

// ADC calibration values
// ADC reading to volts
#define ADC_BATTERY_VOLTS   0.02444	// Approximately: (3.3 V / 4096) * (15k+15k+1k) / 1k, but calibrated by hand
#define ADC_BATTERY_OFFSET	87		// ADC offset when reading zero volts, in ADC units

//do not change, deducted from other settings
#define DC_CUR_THRESHOLD  (DC_CUR_LIMIT*50) // x50 = /0.02 (old MOTOR_AMP_CONV_DC_AMP)


// How many times to sample ADC to get offsets
#define ADC_OFFSET_SAMPLES		1024


// Current measurement using Rds,on
#define I_MEAS_RDSON	1
//#undef I_MEAS_RDSON

// Mosfet Rds,on, e.g. equivalent resistance in on-state
// This is the equivalent value taking into account the voltage measurement
// gain of about 2.5 ... 3 => if Rdson is about 0.0056 ohm, the value is 0.0056 * 3 = 0.017
#define RDSON		0.017


// Sanity checks here

// Only one modulation method can be enabled
#if defined(LEFT_MOTOR_BLDC) && defined(LEFT_MOTOR_SVM)
#error config.h: Only one modulation method can be active for left motor
#endif

#if defined(RIGHT_MOTOR_BLDC) && defined(RIGHT_MOTOR_SVM)
#error config.h: Only one modulation method can be active for right motor
#endif


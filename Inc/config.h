#pragma once
#include "stm32f1xx_hal.h"

// Motor parameters
#define MOTOR_VOLTS		36		// Volts (phase) at rated speed (RMS)
#define MOTOR_SPEED		750		// Nominal speed rpm
#define MOTOR_POLES		4		// Polepairs, mechanical speed to electrical speed
#define MOTOR_CUR		5		// A at rated load / phase (RMS)

#define MOTOR_PERIOD_TO_CMS		0.007854	// Motor sector change period in ms to cm/s speed value

// What control method to use for which motor
//#define LEFT_MOTOR_BLDC		// Use BLDC for left motor
#define LEFT_MOTOR_SVM			// Use SVM for left motor

#define RIGHT_MOTOR_BLDC		// BLDC for right motor
//#define RIGHT_MOTOR_SVM			// SVM for right motor

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

// Limits
#define DC_CUR_LIMIT     1         // Motor DC current limit in amps
#define PPM_NUM_CHANNELS 6         // number of PPM channels to receive

// ADC calibration values
// ADC reading to 100 mVolts (i.e. volts x 10)
#define ADC_BATTERY_VOLTS   0.2498	// 1k / (15k+15k+1k) * 4095/3.3V => Analog attenuation * ADC scaling * 10

//do not change, deducted from other settings
#define DC_CUR_THRESHOLD  (DC_CUR_LIMIT*50) // x50 = /0.02 (old MOTOR_AMP_CONV_DC_AMP)


// How many times to sample ADC to get offsets
#define ADC_OFFSET_SAMPLES		1024



// Sanity checks here

// Only one modulation method can be enabled
#if defined(LEFT_MOTOR_BLDC) && defined(LEFT_MOTOR_SVM)
#error config.h: Only one modulation method can be active for left motor
#endif

#if defined(RIGHT_MOTOR_BLDC) && defined(RIGHT_MOTOR_SVM)
#error config.h: Only one modulation method can be active for right motor
#endif


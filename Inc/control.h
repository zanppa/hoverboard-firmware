/*
 * control.h
 *
 *  Created on: May 6, 2018
 *      Author: tomvoc
 */

#ifndef INC_CONTROL_H_
#define INC_CONTROL_H_

#include "stm32f1xx_hal.h"

// Fault types
#define FAULT_OVERCURRENT		0x01	// Phase current exceeded set limit
#define FAULT_SHORT				0x02	// Shunt current measurement triggered short circuit
#define FAULT_OVERVOLTAGE		0x04	// Battery overvoltage
#define FAULT_UNDERVOLTAGE		0x08	// Battery undervoltage
#define FAULT_OVERTEMP			0x10	// Over temperature measured

// Status bits
#define STATUS_READY			0x01	// Booted up and intialized everything
#define STATUS_RUN				0x02	// One or both motors are running
#define STATUS_CURRENT_LIMIT	0x04	// One or both motors run at current limit
#define STATUS_FIELD_WEAK		0x08	// One or both motors run at field weakening
#define STATUS_OVERVOLTAGE_WARN	0x10
#define STATUS_UNDERVOLTAGE_WARN	0x20


void led_update(void);
void update_controls(void);
void init_controls(void);

void do_fault(uint8_t sides);
void clear_fault(uint8_t sides);
void check_sc() ;

void disable_motors(uint8_t sides);
void enable_motors(uint8_t sides);

void initialize_control_state(void);

uint8_t read_left_hall(void);
uint8_t read_right_hall(void);


#endif /* INC_CONTROL_H_ */

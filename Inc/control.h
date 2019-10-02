/*
 * control.h
 *
 *  Created on: May 6, 2018
 *      Author: tomvoc
 */

#ifndef INC_CONTROL_H_
#define INC_CONTROL_H_

#include "stm32f1xx_hal.h"

void led_update(void);
void update_controls(void);
void init_controls(void);

void do_fault(uint8_t sides);
void clear_fault(uint8_t sides);

void initialize_control_state(void);

uint8_t read_left_hall(void);
uint8_t read_right_hall(void);


#endif /* INC_CONTROL_H_ */

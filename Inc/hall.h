#pragma once

#define HALL_TIMER_FREQ 64000   // Hz, should be multiple of (PWM frequency * 2)


void hall_setup(void);
uint8_t read_left_hall(void);
uint8_t read_right_hall(void);

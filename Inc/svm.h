#pragma once

// Calculate space vector modulator given fixed point modulation index and angle, return
// vector chane times t0, t1 and t2 relative to PWM period
// (t0 is half of the total zero vector length)
void calculate_modulator(uint16_t midx, uint16_t angle, uint16_t *t0, uint16_t *t1, uint16_t *t2);

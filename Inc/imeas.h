#pragma once

void ADC1_init(void);
void ADC2_init(void);
void ADC12_calibrate(void);

typedef struct {
  int16_t i_lA;
  int16_t i_lB;
  int16_t i_lC;
  int16_t i_rA;
  int16_t i_rB;
  int16_t i_rC;
} i_meas_t;

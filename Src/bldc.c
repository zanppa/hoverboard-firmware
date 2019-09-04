
#include "stm32f1xx_hal.h"
#include <math.h>
#include "defines.h"
#include "setup.h"
#include "config.h"
#include "cfgbus.h"
#include "bldc.h"

#include "control.h" // Debug

uint8_t enable = 0;

uint32_t offsetcount = 0;
adc_offsets_t offsets = {0};

uint16_t _lastPosL = 0;
uint16_t _lastPosR = 0;

const int pwm_res = 64000000 / 2 / PWM_FREQ; // = 2000


// Array to convert HALL sensor readings (order ABC) to sector number
// Note that index 0 and 7 are "guards" and should never happen when sensors work properly
const uint8_t hall_to_sector[8] = { 2, 5, 1, 0, 3, 4, 2, 2 };

// Map sectors to correct timer capture/compare registers to generate the modulation pattern
// Array is [sector][vector] where vector states in which order the switches are turned.
// 000 -> Active 1 -> Active 2 -> 111 -> Active 2 -> Active 1 -> 000
// This works as long as left and right use same channel mapping (CCR1=U etc.)
// CCR is uint32_t, this should be used like (uint_32t *)(TIM8+mod_pattern[0][0]) or something
// Offsets should be CCR1=0x34, CCR2=0x38 and CCR3=0x3C so uint8_t is enough
const uint8_t svm_mod_pattern[6][3] = {
  {offsetof(TIM_TypeDef, LEFT_TIM_V), offsetof(TIM_TypeDef, LEFT_TIM_U), offsetof(TIM_TypeDef, LEFT_TIM_W)},
  {offsetof(TIM_TypeDef, LEFT_TIM_V), offsetof(TIM_TypeDef, LEFT_TIM_W), offsetof(TIM_TypeDef, LEFT_TIM_U)},
  {offsetof(TIM_TypeDef, LEFT_TIM_W), offsetof(TIM_TypeDef, LEFT_TIM_V), offsetof(TIM_TypeDef, LEFT_TIM_U)},
  {offsetof(TIM_TypeDef, LEFT_TIM_W), offsetof(TIM_TypeDef, LEFT_TIM_U), offsetof(TIM_TypeDef, LEFT_TIM_V)},
  {offsetof(TIM_TypeDef, LEFT_TIM_U), offsetof(TIM_TypeDef, LEFT_TIM_W), offsetof(TIM_TypeDef, LEFT_TIM_V)},
  {offsetof(TIM_TypeDef, LEFT_TIM_U), offsetof(TIM_TypeDef, LEFT_TIM_V), offsetof(TIM_TypeDef, LEFT_TIM_W)}
};

// Same method for bldc commutation, in this case [sector] contains first positive phase and then negative
const uint8_t bldc_mod_pattern[6][3] = {
  {offsetof(TIM_TypeDef, LEFT_TIM_V), offsetof(TIM_TypeDef, LEFT_TIM_W)},
  {offsetof(TIM_TypeDef, LEFT_TIM_V), offsetof(TIM_TypeDef, LEFT_TIM_U)},
  {offsetof(TIM_TypeDef, LEFT_TIM_W), offsetof(TIM_TypeDef, LEFT_TIM_U)},
  {offsetof(TIM_TypeDef, LEFT_TIM_W), offsetof(TIM_TypeDef, LEFT_TIM_V)},
  {offsetof(TIM_TypeDef, LEFT_TIM_U), offsetof(TIM_TypeDef, LEFT_TIM_V)},
  {offsetof(TIM_TypeDef, LEFT_TIM_U), offsetof(TIM_TypeDef, LEFT_TIM_W)}
};


inline void blockPWM(int pwm, int pos, int *u, int *v, int *w) {
  switch(pos) {
    case 0:
      *u = 0;
      *v = pwm;
      *w = -pwm;
      break;
    case 1:
      *u = -pwm;
      *v = pwm;
      *w = 0;
      break;
    case 2:
      *u = -pwm;
      *v = 0;
      *w = pwm;
      break;
    case 3:
      *u = 0;
      *v = -pwm;
      *w = pwm;
      break;
    case 4:
      *u = pwm;
      *v = -pwm;
      *w = 0;
      break;
    case 5:
      *u = pwm;
      *v = 0;
      *w = -pwm;
      break;
    default:
      *u = 0;
      *v = pwm;	// DEBUG, was 0
      *w = -pwm;	// DEBUG, was 0
  }
}


// Right motor timer update event handler
void TIM1_UP_IRQHandler() {
  // Read HALL sensors to detect rotor position
  uint8_t sector_l = (LEFT_HALL_PORT->IDR >> LEFT_HALL_LSB_PIN) & 0b111;
  sector_l = hall_to_sector[sector_l];

  uint8_t sector_r = (RIGHT_HALL_PORT->IDR >> RIGHT_HALL_LSB_PIN) & 0b111;
  sector_r = hall_to_sector[sector_r];

  // Debug: blink the led here
  led_update();

  // Update modbus variables (globals)
  cfg.vars.pos_l = sector_l;
  cfg.vars.pos_r = sector_r;
}

// Left motor timer update event handler
void TIM8_UP_IRQHandler() {
  // Do nothing, timer 1 handles the calculations
  // And both timers should run at the same rate
}

// Calculate the timer values given the desired voltage vector
// length and angle
void calculate_modulator(uint16_t midx, uint16_t angle) {
  uint16_t ta1;
  uint16_t ta2;
  uint16_t t0;

  // Clamp modulation index to 1.0
  if(midx > 4096) midx = 4096;

  // Calculate the vector times
  ta1 = fx_mulu(midx, array_sin(angle));
  ta1 = fx_mulu(ta1, PWM_PERIOD);

  ta2 = fx_mulu(midx,  array_sin(683 - angle));
  ta2 = fx_mulu(ta2, PWM_PERIOD);

  t0 = (PWM_PERIOD - ta1 - ta2) / 2;
}


//scan 8 channels with 2ADCs @ 20 clk cycles per sample
//meaning ~80 ADC clock cycles @ 8MHz until new DMA interrupt =~ 100KHz
//=640 cpu cycles
void DMA1_Channel1_IRQHandler() {
  DMA1->IFCR = DMA_IFCR_CTCIF1;

  // callibrate ADC offset before startup by averaging 1024 samples.
  if(offsetcount < 1024) {
    offsetcount++;
    offsets.rl1 += adc_buffer.rl1;
    offsets.rl2 += adc_buffer.rl2;
    offsets.rr1 += adc_buffer.rr1;
    offsets.rr2 += adc_buffer.rr2;
    offsets.dcl += adc_buffer.dcl;
    offsets.dcr += adc_buffer.dcr;
    offsets.temp += adc_buffer.temp;
    offsets.vbat += adc_buffer.vbat;
    return;
  } else if (offsetcount == 1024) {
	offsetcount++;
    offsets.rl1 /= 1024;
    offsets.rl2 /= 1024;
    offsets.rr1 /= 1024;
    offsets.rr2 /= 1024;
    offsets.dcl /= 1024;
    offsets.dcr /= 1024;
    offsets.temp /= 1024;
    offsets.vbat /= 1024;
  }

  //disable PWM when current limit is reached (current chopping)
  if(ABS(adc_buffer.dcl - offsets.dcl) > DC_CUR_THRESHOLD || enable == 0) {
    LEFT_TIM->BDTR &= ~TIM_BDTR_MOE;
  } else {
    LEFT_TIM->BDTR |= TIM_BDTR_MOE;
  }

  if(ABS(adc_buffer.dcr - offsets.dcr) > DC_CUR_THRESHOLD || enable == 0) {
    RIGHT_TIM->BDTR &= ~TIM_BDTR_MOE;
  } else {
    RIGHT_TIM->BDTR |= TIM_BDTR_MOE;
  }

  //determine next position based on hall sensors
  uint8_t hall_l =  (LEFT_HALL_PORT->IDR >> LEFT_HALL_LSB_PIN) & 0b111;
  uint8_t hall_r =  (RIGHT_HALL_PORT->IDR >> RIGHT_HALL_LSB_PIN) & 0b111;

  cfg.vars.pos_r = hall_to_sector[hall_r];
  cfg.vars.pos_l = hall_to_sector[hall_l];

  //keep track of wheel movement
  if(_lastPosL != cfg.vars.pos_l)
    cfg.vars.tacho_l += (_lastPosL == (cfg.vars.pos_l + 1)%6) ? 1 : -1;

  if(_lastPosR != cfg.vars.pos_r)
    cfg.vars.tacho_r += (_lastPosR == (cfg.vars.pos_r + 1)%6) ? 1 : -1;

  _lastPosL = cfg.vars.pos_l;
  _lastPosR = cfg.vars.pos_r;

  //update PWM channels based on position
  int ul, vl, wl;
  int ur, vr, wr;

  //blockPWM(cfg.vars.pwm_l, cfg.vars.pos_l, &ul, &vl, &wl);
  //blockPWM(cfg.vars.pwm_r, cfg.vars.pos_r, &ur, &vr, &wr);
  // DEBUG: Output some pwm all the time
  blockPWM(40, cfg.vars.pos_l, &ul, &vl, &wl);
  blockPWM(80, cfg.vars.pos_r, &ur, &vr, &wr);

  LEFT_TIM->LEFT_TIM_U = CLAMP(ul + pwm_res / 2, 10, pwm_res-10);
  LEFT_TIM->LEFT_TIM_V = CLAMP(vl + pwm_res / 2, 10, pwm_res-10);
  LEFT_TIM->LEFT_TIM_W = CLAMP(wl + pwm_res / 2, 10, pwm_res-10);

  RIGHT_TIM->RIGHT_TIM_U = CLAMP(ur + pwm_res / 2, 10, pwm_res-10);
  RIGHT_TIM->RIGHT_TIM_V = CLAMP(vr + pwm_res / 2, 10, pwm_res-10);
  RIGHT_TIM->RIGHT_TIM_W = CLAMP(wr + pwm_res / 2, 10, pwm_res-10);
}

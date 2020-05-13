// Generic math helper functions

/*
Copyright (C) 2019 Lauri Peltonen

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#include <stdint.h>
#include "math.h"

// Fixed point arithmetic

// Multiply two unsigned fixed points together
uint16_t fx_mulu(uint16_t a, uint16_t b) {
  uint32_t res = a * b;	// STM32 has hardware mul from 16-bits to 32 bits
  res >>= FIXED_SHIFT;
  return res & 0xFFFF;
}

// Multiply two signed fixed points together
int16_t fx_mul(int16_t a, int16_t b) {
  int32_t res = a * b;
  res /= FIXED_ONE;
  return (int16_t)res;
}

// Divide two unsigned fixed points, a/b
uint16_t fx_divu(uint16_t a, uint16_t b) {
  uint32_t res = a;
  res <<= FIXED_SHIFT;
  return res / b;
}

// Divide two signed fixed points, a/b
int16_t fx_div(int16_t a, int16_t b) {
  int32_t res = a * FIXED_ONE;
  return res / b;
}

// Linear interpolation of unsigned fixed points
// from a to b whne t goes from 0 to 1 [0...FIXED_ONE]
uint16_t fx_lerpu(uint16_t a, uint16_t b, uint16_t t)
{
  t = CLAMP(t, 0, FIXED_ONE);	// Sanity check
  return fx_mulu(a, FIXED_ONE - t) + fx_mulu(b, t);
}

// Linear interpolation of signed fixed points
// from a to b whne t goes from 0 to 1 [0...FIXED_ONE]
int16_t fx_lerp(int16_t a, int16_t b, int16_t t)
{
  t = CLAMP(t, 0, FIXED_ONE);	// Sanity check
  return fx_mul(a, FIXED_ONE - t) + fx_mul(b, t);
}


// Trigonometric functions

extern uint16_t sine_array[256];

int16_t array_sin(uint16_t angle) {
  uint8_t modulo;
  uint16_t sine;
  uint16_t inv_sine;

  // Change from 12 bits = circle to full 16 bits = circle
  //angle &= FIXED_MASK;	// Clamp to 0...4096 (0...360 degrees)
  //modulo = (angle & 0x03FF) >> 2; // 0...90 degrees (0...1024) scaled to array (256=90 deg)
  modulo = angle >> 6;		// 16384 (90 degrees) to an array of 256 values

  sine = sine_array[modulo];
  inv_sine = sine_array[255-modulo];

  if(angle < ANGLE_90DEG) return sine;
  else if(angle < ANGLE_180DEG) return inv_sine;
  else if(angle < ANGLE_270DEG) return -sine;
  else return -inv_sine;
}

// Following math is from Texas Instruments BPRA048
// Clarke & Park transforms on the TMS320C2xx

// Clarke transform
// Trasform balanced currents a, b to alpha, beta
// assuming a + b + c = 0
void clarke(int16_t a, int16_t b, int16_t *alpha, int16_t *beta) {
  int16_t b2;

  *alpha = a;

  b2 = a + 2*b;
  *beta = fx_mul(ONE_PER_SQRT3, b2);	// beta = 1/sqrt(3) * a + 2/sqrt(3) * b
}

// Inverse clarke transform
void inv_clarke(int16_t alpha, int16_t beta, int16_t *a, int16_t *b) {
  *a = alpha;
  *b = -alpha/2 + fx_mul(SQRT3_PER_2, beta);
}

// Park transform
// Transform current alpha and beta to rotating frame d, q given angle theta
void park(int16_t alpha, int16_t beta, uint16_t theta, int16_t *d, int16_t *q) {
  int16_t sine = array_sin(theta);
  int16_t cosine = array_sin(theta + ANGLE_90DEG);

  *d = fx_mul(alpha, cosine) + fx_mul(beta, sine);
  *q = fx_mul(-alpha, sine) + fx_mul(beta, cosine);
}

// Inverse park transform
void inv_park(int16_t d, int16_t q, uint16_t theta, int16_t *alpha, int16_t *beta) {
  int16_t sine = array_sin(theta);
  int16_t cosine = array_sin(theta + ANGLE_90DEG);

  *alpha = fx_mul(d, cosine) - fx_mul(q, sine);
  *beta = fx_mul(d, sine) + fx_mul(q, cosine);
}

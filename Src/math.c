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
  uint32_t res = a << FIXED_SHIFT;
  return res / b;
}

// Divide two signed fixed points, a/b
int16_t fx_div(int16_t a, int16_t b) {
  int32_t res = a * FIXED_ONE;
  return res / b;
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

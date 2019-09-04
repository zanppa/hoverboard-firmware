// Generic math helper functions
// Copyright (C) 2019 Lauri Peltonen

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




// Trigonometric functions

extern uint16_t sine_array[512];

int16_t array_sin(uint16_t angle) {
  uint8_t modulo;
  uint16_t sine;
  uint16_t inv_sine;

  angle &= 0x0FFF;	// Clamp to 0...4096 (0...360 degrees)
  modulo = angle & 0x01FF; // 0...45 degrees

  sine = sine_array[modulo];
  inv_sine = sine_array[255-modulo];

  if(angle < 0x200) return sine;
  else if(angle < 0x400) return 4096-inv_sine;
  else if(angle < 0x600) return 4096-sine;
  else if(angle < 0x800) return inv_sine;
  else if(angle < 0xA00) return -sine;
  else if(angle < 0xD00) return inv_sine-4096;
  else if(angle < 0xE00) return sine-4096;
  else return -inv_sine;
}

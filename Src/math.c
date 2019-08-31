// Generic math helper functions
// Copyright (C) 2019 Lauri Peltonen

#include <stdint.h>

extern uint16_t sine_array[512];

uint16_t array_sin(uint16_t angle) {
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

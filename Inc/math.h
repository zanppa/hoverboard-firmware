#pragma once

#define FIXED_ONE		4096
#define FIXED_SHIFT		12
#define FIXED_MASK		0x0FFF
#define FIXED_45DEG		512
#define FIXED_60DEG		683		// 682.6666

uint16_t fx_mulu(uint16_t a, uint16_t b);
int16_t fx_mul(int16_t a, int16_t b);

int16_t array_sin(uint16_t angle);


#pragma once
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

// Fixed point number definitions
#define FIXED_ONE		4096
#define FIXED_SHIFT		12
#define FIXED_MASK		0x0FFF

// Angle definitions
#define ANGLE_45DEG		8192
#define ANGLE_60DEG		10922	// 10922.667
#define ANGLE_90DEG		16384
#define ANGLE_120DEG	21845
#define ANGLE_180DEG	32768
#define ANGLE_240DEG	43690
#define ANGLE_270DEG	49152
#define ANGLE_300DEG	54613

// Clarke transform
#define ONE_PER_SQRT3	2365	// FIXED_ONE / sqrt(3)
#define SQRT3_PER_2		3547	// FIXED_ONE * sqrt(3) / 2

uint16_t fx_mulu(uint16_t a, uint16_t b);
int16_t fx_mul(int16_t a, int16_t b);

uint16_t fx_divu(uint16_t a, uint16_t b);
int16_t fx_div(int16_t a, int16_t b);

int16_t array_sin(uint16_t angle);

void clarke(int16_t a, int16_t b, int16_t *alpha, int16_t *beta);
void inv_clarke(int16_t alpha, int16_t beta, int16_t *a, int16_t *b);
void park(int16_t alpha, int16_t beta, uint16_t theta, int16_t *d, int16_t *q);
void inv_park(int16_t d, int16_t q, uint16_t theta, int16_t *alpha, int16_t *beta);

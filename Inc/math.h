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


// Math macros
#define ABS(a) (((a) < 0.0) ? -(a) : (a))
#define LIMIT(x, lowhigh) (((x) > (lowhigh)) ? (lowhigh) : (((x) < (-lowhigh)) ? (-lowhigh) : (x)))
#define SAT(x, lowhigh) (((x) > (lowhigh)) ? (1.0) : (((x) < (-lowhigh)) ? (-1.0) : (0.0)))
#define SAT2(x, low, high) (((x) > (high)) ? (1.0) : (((x) < (low)) ? (-1.0) : (0.0)))
#define STEP(from, to, step) (((from) < (to)) ? (MIN((from) + (step), (to))) : (MAX((from) - (step), (to))))
#define DEG(a) ((a)*M_PI / 180.0)
#define RAD(a) ((a)*180.0 / M_PI)
#define SIGN(a) (((a) < 0.0) ? (-1.0) : (((a) > 0.0) ? (1.0) : (0.0)))
#define CLAMP(x, low, high) (((x) > (high)) ? (high) : (((x) < (low)) ? (low) : (x)))
#define SCALE(value, high, max) MIN(MAX(((max) - (value)) / ((max) - (high)), 0.0), 1.0)
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#define MIN3(a, b, c) MIN(a, MIN(b, c))
#define MAX3(a, b, c) MAX(a, MAX(b, c))
#define FILTER(a, b, t) (fx_mul((a), (t)) + fx_mul((b), (FIXED_ONE)-(t)))
#define FILTERU(a, b, t) (fx_mulu((a), (t)) + fx_mulu((b), (FIXED_ONE)-(t)))

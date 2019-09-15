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

#define FIXED_ONE		4096
#define FIXED_SHIFT		12
#define FIXED_MASK		0x0FFF

#define ANGLE_45DEG		8192
#define ANGLE_60DEG		10922	// 10922.667
#define ANGLE_90DEG		16384
#define ANGLE_120DEG	21845
#define ANGLE_180DEG	32768
#define ANGLE_240DEG	43690
#define ANGLE_270DEG	49152
#define ANGLE_300DEG	54613

uint16_t fx_mulu(uint16_t a, uint16_t b);
int16_t fx_mul(int16_t a, int16_t b);

uint16_t fx_divu(uint16_t a, uint16_t b);
int16_t fx_div(int16_t a, int16_t b);

int16_t array_sin(uint16_t angle);

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
#define FIXED_45DEG		512
#define FIXED_60DEG		683		// 682.6666

uint16_t fx_mulu(uint16_t a, uint16_t b);
int16_t fx_mul(int16_t a, int16_t b);

int16_t array_sin(uint16_t angle);


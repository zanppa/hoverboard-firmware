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


// This struct holds the references for space vector modulation
typedef struct {
  uint16_t modulation_index;
  uint16_t angle;
} svm_ref_t;

// Dead time structure
typedef struct {
  int16_t u_up;		// Going from low to high (up)
  int16_t u_down;	// Going from high to low (down)
  int16_t v_up;
  int16_t v_down;
  int16_t w_up;
  int16_t w_down;
} dead_time_t;

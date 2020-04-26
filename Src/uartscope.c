// Simple UART debug "scope"

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

#include "config.h"
#include "uartscope.h"
#include "uart.h"


// Start byte, data bytes, stop byte
// No need to be volatile since these should be written in
// one place only
// 0xAAEE = Magic start byte,0xBEEF magic end byte (works with 16 bit datatype)
// Some GCC compiler magic to initialize the array with start and end bytes
SCOPE_DATATYPE scope_channel[SCOPE_CHANNELS+2] = {0xAAEE, [1 ... SCOPE_CHANNELS] = 0, 0xBEEF};

void scope_set_data(uint8_t ch, SCOPE_DATATYPE data)
{
  if(ch < SCOPE_CHANNELS) scope_channel[ch+1] = data;
}

// Send data over UART
// returns 1 on success, 0 on error (e.g. previous data not yet transmitted)
uint8_t scope_send()
{
#if defined(SCOPE_UART)
  return UARTSend(SCOPE_UART, (const uint8_t *)&scope_channel[0], sizeof(SCOPE_DATATYPE)*(SCOPE_CHANNELS+2));
#else
  return 0;	// Not implemented
#endif
}

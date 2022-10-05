#!/usr/bin/env python3
import minimalmodbus

# integers should be swapped, strings not
def swap(uint16):
  hi = (uint16 & 0xff00)
  lo = (uint16 & 0x00ff)

  return (lo << 8) + (hi >> 8)

def signed(uint16):
  if uint16 >= 32768:
    return -(65535 - uint16)
  else:
    return uint16

# Default slave ID is 16, set in Inc/modbus.h MB_SLAVE_ID
instrument = minimalmodbus.Instrument('/dev/serial0', 16)  # port name, slave address (in decimal)
instrument.serial.baudrate = 115200

battery = 27 # Battery voltage

while 1:
  v_bat = swap(instrument.read_register(battery))

  print('Battery voltage: {}'.format(v_bat))

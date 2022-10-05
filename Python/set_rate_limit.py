#!/usr/bin/env python3
import minimalmodbus
from time import sleep

def swap(uint16):
  hi = (uint16 & 0xff00)
  lo = (uint16 & 0x00ff)

  return (lo << 8) + (hi >> 8)


# Default slave ID is 16, set in Inc/modbus.h MB_SLAVE_ID
instrument = minimalmodbus.Instrument('/dev/serial0', 16)  # port name, slave address (in decimal)
instrument.serial.baudrate = 115200

rate_limit = swap(instrument.read_register(14))
print('Rate limit', rate_limit)

# Write max pwm values
print('Enter new rate_limit')
rate_limit = int(input())
print('Setting rate_limit to {}'.format(rate_limit))
instrument.write_register(14, swap(rate_limit), 0)


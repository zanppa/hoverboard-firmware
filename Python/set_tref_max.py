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

max_torque = swap(instrument.read_register(17))
print('Left max torque', max_torque)

# Write max torque values
print('Enter new max torque for left')
max_torque = int(input())
print('Setting max torque amplitude to {}'.format(max_torque))
instrument.write_register(17, swap(max_torque), 0)



max_torque = swap(instrument.read_register(18))
print('Right max torque', max_torque)

# Write max torque values
print('Enter new max torque for right')
max_torque = int(input())
print('Setting max torque amplitude to {}'.format(max_torque))


instrument.write_register(18, swap(max_torque), 0)


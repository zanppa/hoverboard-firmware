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

max_pwm = swap(instrument.read_register(15))
print('Left max PWM', max_pwm)

# Write max pwm values
print('Enter new max pwm for left')
max_pwm = int(input())
print('Setting max pwm amplitude to {}'.format(max_pwm))
instrument.write_register(15, swap(max_pwm), 0)



max_pwm = swap(instrument.read_register(16))
print('Right max PWM', max_pwm)

# Write max pwm values
print('Enter new max pwm for right')
max_pwm = int(input())
print('Setting max pwm amplitude to {}'.format(max_pwm))


instrument.write_register(16, swap(max_pwm), 0)


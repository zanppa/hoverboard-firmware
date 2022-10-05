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

setpoint_l = 12
value = 0

print("Current left ref: ", swap(instrument.read_register(setpoint_l)))
print("Enter new left ref:")
ref_speed = int(input())

instrument.write_register(setpoint_l, swap(ref_speed), 0)  # Registernumber, value, number of decimals for storage

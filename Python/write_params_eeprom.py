#!/usr/bin/env python3
import minimalmodbus
from time import sleep

def swap(uint16):
  hi = (uint16 & 0xff00)
  lo = (uint16 & 0x00ff)

  return (lo << 8) + (hi >> 8)


# Default slave ID is 16, set in Inc/modbus.h MB_SLAVE_ID
instrument = minimalmodbus.Instrument('/dev/serial0', 16, debug=True)  # port name, slave address (in decimal)
instrument.serial.baudrate = 115200

register = 0
value = 0xC0DE # Should equal to what is read first. For safety reason a hardcoded value is used here

print("Forcing parameter write to EEPROM")
print("Reading magic... ", hex(swap(instrument.read_register(register))))
print("Using magic value ", hex(value))
instrument.write_register(register, swap(value), 0)  # Registernumber, value, number of decimals for storage

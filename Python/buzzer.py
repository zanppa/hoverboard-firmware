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

buz_ena = 19
tone = 25
pattern = 26

val_ena = swap(instrument.read_register(buz_ena))
val_tone = swap(instrument.read_register(tone))
val_pat = swap(instrument.read_register(pattern))

print(val_ena, val_tone, val_pat)

while True:
  print("Enter tone value, ctrl+c to exit")
  val_tone = int(input())
  print("Enter pattern value, ctrl+c to exit")
  val_pat = int(input())
  print("Enter enable value, ctrl+c to exit")
  val_ena = int(input())
  print('Writing' )
  instrument.write_register(tone, swap(val_tone), 0)  # Registernumber, value, number of decimals for storage
  instrument.write_register(pattern, swap(val_pat), 0)  # Registernumber, value, number of decimals for storage
  instrument.write_register(buz_ena, swap(val_ena), 0)  # Registernumber, value, number of decimals for storage

  val_ena = swap(instrument.read_register(buz_ena))
  val_tone = swap(instrument.read_register(tone))
  val_pat = swap(instrument.read_register(pattern))

  print(val_ena, val_tone, val_pat)

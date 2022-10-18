#!/usr/bin/env python3
import minimalmodbus
from time import sleep

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

ctrl_reg = 45
data_reg = 46

ct = swap(instrument.read_register(ctrl_reg))

print("Control byte: ", hex(ct))
print("Press enter to trigger")
input()
ct = ct | 1
instrument.write_register(ctrl_reg, swap(ct), 0)

print("Wait until ready...")
ct = swap(instrument.read_register(ctrl_reg))
while not (ct & 2):
  sleep(0.01)
  ct = swap(instrument.read_register(ctrl_reg))


print("Reading data...")
print("Index,Data0,Data1,Data2,Data3,Data4,Data5,Data6,Data7")
for i in range(256):
  addr = (255-i) * 2 * 4 # Last index is the first stored value. Keep low 2 bits zero to not trigger again

  # Read first 4 channels
  instrument.write_register(ctrl_reg, swap(addr), 0)
  data = instrument.read_registers(data_reg, 4)

  # Read last 4 channels
  instrument.write_register(ctrl_reg, swap(addr + (1*4)), 0) # offset times 4 to keep the low 2 bits zero
  data2 = instrument.read_registers(data_reg, 4)
  data.extend(data2)

  for j in range(8):
    data[j] = swap(data[j])

  # Handle data type dependent magic here
  data[2] = signed(data[2])
  data[3] = signed(data[3])

  # Insert index as the first column
  data.insert(0, i)

  print(','.join(map(str, data)))

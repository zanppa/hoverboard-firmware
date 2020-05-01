# Hoverboard firmware
This document is part of the hoverboard firmware documentation.

## Modbus
This document describes the modbus communication

### Configuration
Modbus can be enabled form `Inc/config.h` by defining either `LEFT_SENSOR_MODBUS` 
or `RIGHT_SENSOR_MODBUS` (but not both). This enables the UART on the respective 
sensor board connection and configures it for modbus usage. The left sensor board 
uses UART 3 while the right board uses UART 2.

### Connection
Electrical connections of the sensor board are such that the black wire is ground, 
blue wire is TX (transmit) and green wire is RX (receive). Red wire is +15V (approximately), 
and typically can be left unconnected.

To use this with Raspberry PI Zero W, connect the black ground wire to any ground pin, 
e.g. number 6. Blue TX pin needs to be connected to  pin 10 (Raspberry PI RXD, BCM 15) 
and green RX wire to pin 8 (Raspberry PI TXD, BCM 14).

The modbus uses RTU protocol, which is a serial binary protocol. The serial port 
parameters are 115200 bps, 8 bits, no parity, 1 stop bit. No hardware/XON/XOFF or 
similar flow control is used.

The default slave address for the hoverboard is 16 (decimal). This can be changed 
from `Inc/modbus.h` with variable `MB_SLAVE_ID` if necessary.


### Usage
The modbus is based on registers that can be read or written. All registers are 16 bits by 
default, and are indexed as register number, starting from 0. Strings, floats etc. take 
more than one register if their size exceeds 16 bits.

The registers are defined in `Inc/cfgbus.h`. The register map is not fixed yet, but will be 
provided once it stabilizes. At the moment the registers start with following lists:

```
#define CFG_ENTRIES(_ENTRY) \
	_ENTRY( magic          , uint16_t   , _U16     , false     , "Magic Value"       ) \
	_ENTRY( nr_entries     , uint16_t   , _U16     , false     , "Nr Entries"        ) \
	_ENTRY( dev_name[12]   , char       , _STR12   , false     , "Device Name"       ) \
	_ENTRY( err_code       , uint16_t   , _U16     , true      , "Error Code"        ) \
	_ENTRY( err_cnt        , uint16_t   , _U16     , true      , "Error Count"       ) \
	...
```
These map to register number as follows:

| Register n. | Name       | Size (words, 16 bits) |
|:-----------:|------------|-----------------------|
|      0      | magic      | 1                     |
|      1      | nr_entries | 1                     |
|      2      | dev_name   | 6 (12 characters)     |
|      8      | err_code   | 1                     |
|      9      | err_cnt    | 1                     |

I've used [minimalmodbus](https://github.com/pyhys/minimalmodbus) to succesfully interface 
with the hoverboard. Following is an example code that reads the first 5 variables and prints 
their value.

```
#!/usr/bin/env python3
import minimalmodbus

instrument = minimalmodbus.Instrument('/dev/serial0', 16)  # port name, slave address (in decimal)
instrument.serial.baudrate = 115200

print('Magic:', instrument.read_register(0))
print('Entries:', instrument.read_register(1))
print('Name:', instrument.read_string(2, 6))
print('Error code:', instrument.read_register(8))
print('Error count:', instrument.read_register(9))
```
Note that the 16 bits are sometimes stored in different order than what minimalmodbus or python expects, and the bytes need
to be swapped. This probably depends on the host architecture, and is easily verified with the magic word.


### Storing the configuration
The configuration bus implementation supports writing the registers to eeprom so that the values 
survive hoverboard reset. The store is triggered by writing the magic value to the first (0th) 
register.

The configuration should be loaded when the board is booted, but is not yet implemented.


## Copyright
Copyright (C) 2020 Lauri Peltonen

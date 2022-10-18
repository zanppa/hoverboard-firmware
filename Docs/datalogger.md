# Hoverboard firmware
This document is part of the hoverboard firmware documentation.

## Datalogger
This document describes the high-speed datalogger feature that can be 
used to sample several variables at a high speed for later analysis.

The datalogger can sample 8 variables simultaneously and at it runs at 
the PWM frequency, which can also be divided by an integer if slower 
sampling is required. This allows sampling for example rotor angle, currents 
and modulation vector length at a rate necessary for debugging the system.

### Configuration
The datalogger is configured off-line directly in the source code.

The basic features, like number of samples and sampling speed, are in `Inc/config.h`:

```C
#define DATALOGGER_ENABLE
#define DATALOGGER_MAX          0xFF
#define DATALOGGER_TYPE         uint16_t
#define DATALOGGER_COUNT_TYPE   uint8_t
#define DATALOGGER_DIVIDER      7
#define DATALOGGER_CHANNELS		8
#define DATALOGGER_TRIG_TRIP
#define DATALOGGER_SAMPLES_AFTER    32
```

If the `DATALOGGER_ENABLE` is defined, then the datalogger feature will be compiled in.

`DATALOGGER_MAX` tells the number of samples to take. I have not tried more than 256, it 
may work or there may be some memory-borders that prevent having an array that large... Also 
config bus currently hard limits the maximum samples to transfer to 14 bits, i.e. 16 ksamples.

`DATALOGGER_TYPE` is the variable type (byte size) of the variables to sample and `DATALOGGER_COUNT_TYPE` is
the type to be used for the index counter. The first cannot really be changed as the size is 
hardcoded in several places like in config bus. The latter must be changed so that the 
amount of samples fits inside that.

The `DATALOGGER_DIVIDER` tells how often to sample the variables. Setting 0 here 
samples every PWM period while 1 samples every other and 7 samples only every 8th PWM period.

`DATALOGGER_CHANNELS` define how many channels there is. Currently some parts of 
the code (transfering the data over modbus) are hard coded to support 8 channels only.

`DATALOGGER_TRIG_TRIP` controls whether some faults (like overcurrent & short circuit) trigger 
the datalogger or not. If this is defined, then the faults trigger datalogger. Should be used 
together with `DATALOGGER_SAMPLES_AFTER`.

Finally, the `DATALOGGER_SAMPLES_AFTER` controls whether sampling is done also before the trigger 
and only the defined amount of samples is stored after trigger. If not defined, everything is 
samples after the trigger. This is useful with the `DATALOGGER_TRIG_TRIP` to see what happened 
just before the trip.

### Triggering and loading data
Triggering of the datalogger is currently done only from modbus. There is a control word called
`dlog_ctrl` currently in register 45 that is used to control the datalogger. It contains following:

 - 0th bit (LSB) is the trigger bit. Setting this to 1 triggers the datalogger
 - 1st bit indicates whether the datalogger is stopped. 0 means logger is running while 1 means it is stopped
 - 2nd bit is used to reset the datalogger (write offsets etc)
 - bits 3...15 address of the sample to read. Note that the largest address is the first sampled

The data to read from the datalogger is in register dlog_data, which is a 8 byte register spanning 
addresses 46...50. To read data from the datalogger:

 1. Trigger the datalogger by writing 1 to the least significant bit of dlog_ctrl register
 2. Wait until the second bit (bit 1) is 1
 3. Write address of the sample to read to the dlog_cltr bits 2...15 (i.e. address * 4)
 4. Read 4 (16-bit) registers from dlog_data register to get the 4 variables
 5. Repeat 3...5 until all samples are read

Note that the data is stored in reverse order, i.e. the sample with address 0 is the last sample.

The lowest bit tells whether to transfer the first 4 or last 4 sampled variables. I.e. reading 
address 0 results in the first 4 channels and address 1 the last 4 channels. For this reason 
you need to read addresses up to two times the number of samples.

If `DATALOGGER_SAMPLES_AFTER` is defined, after 1st bit indicates that the datalogger has stopped, 
the address (bits 3...15) indicate the position after where the datalogger stopped writing in the ring 
buffer, i.e. it contains the offset of the oldest sample. Note that as the logger writes in reverse 
order, the address should be then decreased from this offset to read the values in order.

### Implementation
The variables are configured in `Src/main.c` so that the pointers to the variables to be sampled are 
written to `datalogger_var[0]` through `datalogger_var[7]`. These are (void *) pointers, but internally 
currently sample 16 bits. If the variable is less wide (e.g. byte), other bits of the sample may 
contain rubbish and must be cleaned after fetching the data.

The sampling itself happens in `Src/svm.c`, after the modulation. This allows for fast sampling at the 
PWM frequency, but as the sampling is done after modulation it does not affect the modulator performance 
and as a side benefit it has access to calculated vector lengths as well as it should have fresh 
current measurement values also.


## Copyright
Copyright (C) 2022 Lauri Peltonen

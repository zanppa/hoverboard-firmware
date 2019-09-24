# Hoverboard firmware
This document is part of the hoverboard firmware documentation.

## Units
This document describes how different units of measure are represented in the firmware.

### Fixed point
To speed up calculations, fixed point arithmetic is used. In the selected notation the floating 
point values is multiplied by 4096 and then converted to 16 bit signed integer. As a result, the 
fixed point number can hold values between +-8. The value precision is approximately 0.024 %.

As the value range is quite low, a per unit system is used where most values used lay between 0 and 1.

### Angles
Angles are represented such that full circle, 360 degrees equals 0xFFFF e.g. 65535 in decimal.
The angle resolution is then approximately 0.0055 degrees.

Selected like this, 45 degrees happens to be 8192 decimal in the representation. 60 degrees, 
which is used in the space vector modulation is 10922.667 which is not integer, but hopefully the 
small error does not affect the modulation quality.

### Per-unit system
Voltages and currents are represented in per-unit values. For this representation, base values are 
first selected.

The base voltage is the nominal peak voltage of the motor, running at nominal speed. At this point 
it is selected to be 36 V which is the typical voltage of the battery.

The base current is selected to be 5 A at the moment.

The base frequency is currently not known. It should be the speed at which the nominal voltage is 
achieved (back-emf).

All the values are then scaled such that 1.0 represents the base value, voltage, current or speed.


## Copyright
Copyright (C) 2019 Lauri Peltonen

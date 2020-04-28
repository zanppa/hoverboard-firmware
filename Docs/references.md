# Hoverboard firmware
This document is part of the hoverboard firmware documentation.

## References
This document described the references and how they are used

### Reference sources
In the firmware only one reference source can be active at a time. Currently 
there is two possible sources of reference, and both motors must use the same 
reference source.

 * Modbus reference
 * Analog reference

The modbus reference is a signed fixed point value.

Analog reference can be used either in direct or differential mode. In the direct 
mode analog 1 is applied to the left motor and 2 to the right motor. The analog 
value is mapped such that zero is at the middle of the analog range.

In the differential mode the analog reference 1 is the base reference applied to 
both motors. Then, analog reference 2 is added to the left motor and subtracted 
from the right motor reference. This mode can be used as a "steering wheel" mode.

### Reference modes
The reference may represent different things depending on the selected reference mode. 
In this chapter the reference modes are explained.

#### Torque
In torque mode the reference represent the torque produced by the motor. At 0 
reference, the produced torque is also 0 and the motor rotates freely. Applying 
positive or negative reference adds torque and causes the motor speed to increase. 
The speed is not limited by the reference, only physical phenomena limit the speed.

Torque reference can be used in BLDC or FOC control modes. In BLDC mode the applied 
voltage represents the produced torque such that reference of 1.0 causes motor rated 
voltage to be applied.

In FOC mode the reference controls the applied (Q-axis) motor current, such that 
reference of 1.0 causes the control to apply the motor rated current.

#### Speed
In speed control the reference controls the speed of the motor such that at 0 reference 
the motor is at rest while with reference of 1.0 the motor rotates at rated speed.

Speed control can be used with all control modes. In BLDC or FOC, the speed controller 
outputs a torque reference that is applied to the motor by using a PI type controller.

In SVM control the speed reference directly sets the modulator rotation speed and 
applied voltage level.

#### Angle
Angle control can only be used with SVM control. In angle mode the reference sets the 
applied voltage angle directly, and as such keeps the motor still at the desired position. 
I have not decided yet how to set the voltage amplitude...


## Copyright
Copyright (C) 2020 Lauri Peltonen

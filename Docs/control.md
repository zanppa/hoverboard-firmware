# Hoverboard firmware
This document is part of the hoverboard firmware documentation.

## Control
This document describes the control schemes used.

### V/f control
V/f control is the simplest control method. The basic idea is that the amplitude of the 
permanent magnet motor back-EMF depends linearly on the motor speed. To keep the motor flux 
constant, the input voltage amplitude is also varied linearly with rotation speed.

Because of resistances and other losses of the motors, the voltage cannot be ramped to zero 
at low speeds, but the voltage must decrease slower when approaching zero. This is typically 
called IR compensation (current*resistance). Easiest way to achieve this is to keep the 
applied voltage at a constant level below some pre-defined frequency.

![V/f control with IR compensation](images/vf_control_with_ir.png)

### BLDC control
BLDC control is one of the simplest ways to control a permanent magnet motor. In BLDC control 
the motor has hall effect switches that are used to sense the rotor position, and according 
to the position, a voltage is applied. The sensor feedback and the voltage applies is matched 
such that the motor current will be in phase with the motor back-emf and as a result torque 
is generated.

This method is simple since it needs no complex control schemes, just position sensing which 
determines which phases are energized. The torque amplitude can be controlled by using PWM to 
reduce the applied (effective) voltage and thus current. Downside is that the torque generated 
is not constant over the rotation period and the current is not accurately in phase with the 
back-emf which lowers the efficiency and causes vibration and noise in the motor.

### FOC (field-oriented control)
In field oriented control the voltage applied to the motor is controlled in such a manner that 
the current produced is sinusoidal and in phase with the back-emf of the motor. If correctly 
applied, the torque produced is very constant and the motor efficiency is high and noise low.

FOC can be either sensored or sensorless. In sensored control the hall effect sensors are used 
to determine the rotor position (and speed) while in sensorless control, current measurement 
is used as feedback from the control, and together with motor model used to determine the 
rotor position.



## Copyright
Copyright (C) 2019 Lauri Peltonen

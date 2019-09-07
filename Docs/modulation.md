# Hoverboard firmware
This document is part of the hoverboard firmware documentation.

## Modulation
The modulation method used in the firmware is described in this document.

### BLDC modulation
In this chapter the original BLDC trapezoidal PWM scheme is described. The operation is 
analyzed from the NiklasFauth's source code.

The modulation is a trapezoidal modulation, in which one phase is not switching and 
the two others have positive (> 50 %) or negative (< 50 %) PWM. First, the hall sensors 
are used to determine the motor position and select which channels to use, according 
to following table.

| Sector | Hall A | Hall B | Hall C | Output A | Output B | Output C |
|--------|--------|--------|--------|----------|----------|----------|
| 0      | 1      | 1      | 0      | 0        | +        | -        |
| 1      | 0      | 1      | 0      | -        | +        | 0        |
| 2      | 0      | 1      | 1      | -        | 0        | +        |
| 3      | 0      | 0      | 1      | 0        | -        | +        |
| 4      | 1      | 0      | 1      | +        | -        | 0        |
| 5      | 1      | 0      | 0      | +        | 0        | -        |

The PWM duty cycle is set directly by the given (and low-pass filtered) speed reference.

This basically means that the motor applies a torque which is dependent on 
the requested speed reference, and as a result, the speed settles to a point 
in which losses (air drag, friction) equal the torque generated.

### SVM modulation
SVM (Space Vector Modulation) is the modulation scheme used in this firmware.

The space vector hexagon is seen in image below. The bold numbers inside the sectors 
correspond to the sector numbers used in the earlier BLDC modulation scheme. The oblique numbers 
at the corners is the output switch states, given in ABC order. It can be seen that for example 
at corners of sectors 0 and 1 the B phase is high (1), which is similar to the BLDC modulation where 
the B output was positive.

![SVM hexagon](svm_hexagon.png)

The given voltage vector `Uref` can be realized using the neighboring corners and one or both zero 
vectors. First, it is necessary to select the correct sector to know which vectors to use. The sector 
and vectors are selected according to following table.

| Sector |    Angle   | Zero 1 | Active 1 | Active 2 | Zero 2 |
|:------:|:----------:|:------:|:--------:|:--------:|:------:|
|    0   |  60...120  |   000  |    010   |    110   |   111  |
|    1   |  120...180 |   000  |    010   |    011   |   111  |
|    2   |  180...240 |   000  |    001   |    011   |   111  |
|    3   |  240...300 |   000  |    001   |    101   |   111  |
|    4   | 300..360/0 |   000  |    100   |    101   |   111  |
|    5   |   0...60   |   000  |    100   |    110   |   111  |

The modulation pattern is `Zero 1 -> Active 1 -> Active 2 -> Zero 2 -> Active 2 -> Active 1 -> Zero 1`. 
Active vectors are selected in such order that only one transistor needs to be switched to transit 
from one vector to another. The angle is the desired voltage vector angle, while amplitude only 
affects the vector times.

Knowing which vectors to use, vector times are calculated using
```
Ta1 = Ts * 2/sqrt(3) * M * sin(pi/3 - angle)
Ta2 = Ts * 2/sqrt(3) * M * sin(angle)
T0 = Ts - Ta1 - Ta2
```
Where `Ta1` and `Ta2` are active vector times and `T0` is the sum of the zero vector times. `M` is 
the modulation index, i.e. the voltage amplitude and `angle` is the desired voltage angle inside the 
sector, i.e. 0...60 degrees.

Since it is quite time consuming to calculate sinusoidal function, the value of `Ts * 2/sqrt(3) * sin(angle)` is 
pre-calculated into an array for 0...60 degrees angle. The `pi/3 - angle` is achieved by reading the 
array backwards. Since `Ts` is pre-calculated into the array, it is necessary to re-compute the array 
if switching frequency is changed.

Functions above require modulation index to be equal or smaller than `sqrt(3)/2=0.866`, which is the 
maximum radius of a circle that fits inside the SVM hexagon. In practice, the equations are normalized 
by dividing with that value, so that modulation index of one gives the maximum circle.

The modulator requires, as an input, the desired voltage vector length and angle, and as a result it 
gives out the switching instants of the three switches.


## Copyright
Copyright (C) 2019 Lauri Peltonen

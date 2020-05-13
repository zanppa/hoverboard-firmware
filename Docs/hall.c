# Hoverboard firmware
This document is part of the hoverboard firmware documentation.

## Hall sensors
The hall sensor reading with interrupts is described in this document.

### Connections
The HALL sensors in the hoverboard are connected in following way:

| Phase | Left motor pin | Pin interrupt  | Right motor pin | Pin interrupt |
|:-----:|:--------------:|:--------------:|:---------------:|:-------------:|
|  A/U  |    PB5         | EXTI5          |   PC10          | EXTI10        |
|  B/V  |    PB6         | EXTI6          |   PC11          | EXTI11        |
|  C/W  |    PB7         | EXTI7          |   PC12          | EXTI12        |

### Configuration
Even though all pins have individiual interrupt lines as shown in previous table, 
only EXTI0...4 have individual interrupt handlers. Others are bundled together so 
that there is only
 * EXTI9_5 that handles EXTI5 ... EXTI9
and
 * EXTI15_10 that handles EXTI10 ... EXTI15
which means that left motor hall sensors can be handled in the EXTI9_5 handler 
and right motor hall sensors can be handled in EXTI15_10 handler.

### Process
It is possible to combine the interrupt handler for both motors to same function, 
but it might make sense to duplicate the code for both motors to separate functions.

What the iterrupt handler must do is
 1. Determine the sector in which the motor is and update the modulator
 2. Calculate the time it took to cross over the sector (i.e. rotation speed)
 3. Determine rotation direction

The first point is trivial and is done by reading all three I/O pins and reading 
the sector from the hall-to-sector array. This value is then updated to the motor 
struct.

The second point requires that we have either separate upcounting timers for both motors, and 
the timer is read at the beginning of the hall interrupt. Then the timer is reset. This 
gives the time it took to cross the sector and thus rotation speed. Note that it may 
or may not be necessary to read the pisn first and see that the sector really changed before 
doing this, if there are some very short glitches in the I/O lines... In this case a 16 bit timer 
running at 100 kHz would allow counting up to 0.65 secods which is quite a low speed already. 
Slowing the timer to e.g. 20 khz would probably still give enough precision but count up to 3.2 
seconds.

Other way is to have a common timer that is never reset but counts for a longer time, e.g. by using 
a 16-bit overflow counter that is increased every time the timer/counter overflows. This only uses 
one timer which also may run at much faster rate since there is effectivaly 32 bits, but at the 
same time adds complexity as there is two variables to read, need to account for the overflow 
counter overflow and an additional overflow interrupt handler.

A typical way to handle the overflow is by using following pseudocode
```
do {
  H1 = overflow_counter;
  L = timer_counter;
  H2 = overflow_counter;
} while(H1 != H2);
```
If the overflow period is e.g. 0.6 seconds, the operation is guaranteed to succeed in 2 iterations.

The first method from these two was implemented.

The third point is laso easily accomplished by checking whether the preivous sector was larger 
or smaller (taking into account sector overflow) than the current sector.

### Zero speed detection
Zero speed detection is done using the timer overflow interrupts. It is assumed that if the 
sector has not changed before the timer overflows, the speed is so slow that it can be approximated 
to be zero.

Thus the timer overflow interrupts are enabled and the interrupt handler sets the motor rotation 
speed to zero. This allows the speed to reach zero in quite a short time. Without the interrupt, the 
speed should be set to zero by some other means in e.g. the control loop.

## Copyright
Copyright (C) 2020 Lauri Peltonen

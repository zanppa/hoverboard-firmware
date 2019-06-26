# hoverboard-firmware-hack

This repo is a fork of Niklas Fauths Hoverboard Hack Firmware for some of the newer mainboards with the AT32F403Rx_HD controller. In order to keep the code mostly compatible the HAL libraries were adapted to fit the AT32 naming scheme and registers. This is possible as the AT32F403 is a mostly compatible device to the STM32F103 used on older and other Hoverboard mainboards.

---

## Build Instructions

The code was compiled with GNU arm-none-eabi-gcc 5.4.1 under windows and should be compilable with other version of the arm-gcc.


---

## Hardware
![otter](https://raw.githubusercontent.com/cloidnerux/hoverboard-firmware-hack/master/pinout.png)

The main difference of the AT32 board is the use of dedicated gate drivers (IR2108 clones) and featuring less
capacitors. Also there is no current sense resistor for each motor anymore but some overcurrent trigger on one GPIO pin.
Lastly the connectors are a bit different from other versions of the hoverboard controllers.

These boards see a lot of small changes over the years to make them cheaper or to implement certain functions. Your board
might not be quite the same as the board shown here.

The reverse-engineered schematics of the mainboard can be found here:
http://vocke.tv/lib/exe/fetch.php?media=20150722_hoverboard_sch.pdf

---

## Flashing
To build the firmware, just type "make". Make sure you have specified your gcc-arm-none-eabi binary location in the Makefile ("PREFIX = ...") (version 7 works, there is a version that does not!) (if the ons in linux repos do not work, use the official version: https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads). Right to the STM32, there is a debugging header with GND, 3V3, SWDIO and SWCLK. Connect GND, SWDIO and SWCLK to your SWD programmer, like the ST-Link found on many STM devboards.

Do not power the mainboard from the 3.3V of your programmer! This has already killed multiple mainboards.

Make sure you hold the powerbutton or connect a jumper to the power button pins while flashing the firmware, as the STM might release the power latch and switches itself off during flashing. Battery > 36V have to be connected while flashing.

I flashed my AT32 with a Segger J-Link mini using the SWD connector. 

As there is no direct flash access via SWD a flash loader has to be used. The appropriate flashloader was taken from the Keil uVision support package and is in a CMSIS Pack format. Using Segger OpenLoader feature one can add his own support for the AT32F403.
For this add following code to your `<Segger Installation Dir>\JLinkDevices.xml`
    
	<Device>
		<ChipInfo Vendor="Artery" Name="AT32F403Rx_HD" WorkRAMAddr="0x20000000" WorkRAMSize="0x38000" Core="JLINK_CORE_CORTEX_M4" />
		<FlashBankInfo Name="Internal Flash Bank 1" BaseAddr="0x08000000" MaxSize="0x80000" Loader="Devices/AT32F403_1024.FLM" LoaderType="FLASH_ALGO_TYPE_OPEN" />
	</Device>

and copy the "AT32F403_1024.FLM" from the folder flash to `<Segger Installation Dir>\Devices`

I did not test to flash the AT32 with a ST-Link but I think this should be no big problem.

## Troubleshooting
First, check that power is connected and voltage is >36V while flashing.
If the board draws more than 100mA in idle, it's probably broken.

If the motors do something, but don't rotate smooth and quietly, try to use an alternative phase mapping. Usually, color-correct mapping (blue to blue, green to green, yellow to yellow) works fine. However, some hoverboards have a different layout then others, and this might be the reason your motor isn't spinning.

Nunchuck not working: Use the right one of the 2 types of nunchucks. Use i2c pullups.

Nunchuck or PPM working bad: The i2c bus and PPM signal are very sensitive to emv distortions of the motor controller. They get stronger the faster you are. Keep cables short, use shielded cable, use ferrits, stabilize voltage in nunchuck or reviever, add i2c pullups. To many errors leads to very high accelerations which triggers the protection board within the battery to shut everything down.

Most robust way for input is to use the ADC and potis. It works well even on 1m unshielded cable. Solder ~100k Ohm resistors between ADC-inputs and gnd directly on the mainboard. Use potis as pullups to 3.3V.

---


## Examples

Have a look at the config.h in the Inc directory. That's where you configure to firmware to match your project.
Currently supported: Wii Nunchuck, analog potentiometer and PPM-Sum signal from a RC remote.
A good example of control via UART, eg. from an Arduino or raspberryPi, can be found here:
https://github.com/p-h-a-i-l/hoverboard-firmware-hack

If you need additional features like a boost button, have a look at the while(1) loop in the main.c

### Additional Hardware

* [breakout/interconnect boards](https://github.com/Jan--Henrik/hoverboard-breakout)  Breakout/Interconnection boards for hoverboard hacking.

### Projects based on it
* [bobbycar-optimized firmware](https://github.com/larsmm/hoverboard-firmware-hack-bbcar)  based on this one with driving modes, acceleration ramps and some other features
* [wheel chair](https://github.com/Lahorde/steer_speed_ctrl) controlled with a joystick or using a CC2650 sensortag to control it over  bluetooth with pitch/roll.
* [TranspOtterNG](https://github.com/Jan--Henrik/transpOtterNG) TranspOtter is an open source semi self driving transportation platform based on hoverboard hardware
* [BiPropellant](https://github.com/bipropellant) - fork which focusses on reliable machine control, but also retains HoverBoard functionality if desired.

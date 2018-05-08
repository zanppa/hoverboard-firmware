# hoverboard-firmware-hack

This is a Fork fom NiklasFauth's hoverboard-firmware-hack.

The goal is to make the board more configurable without requiring reprogramming for every setting, 
and to allow controller the board over different interfaces.

Current state:
 - implemented dma/interrupt based UART2/3 RX/TX (see uart.h/c)
 - implemented simple modbus slave (supports commands 0x3(read) and ox10(write). Works with open-source QtModMaster tool
 - implemented modbus wrapper to allow user-defined variables to be accessed in code and over modbus. 
 - Built GUI to talk to modbus-wrapper and allow plotting/changing and storing device settings
 - Added Tacho/Speed signals for both motors
 - cleaned code from some hacks/features. Some will be re-added later
      - removed I2C/ADC/PWM control. This will be re-integrated soon
      - clean up BLDC ADC handler, now more efficient, and only dealing with motors
      - no more delays in code, but rather non-blocking Tick() based updates
      - disabled buzzer, man I hate buzzers
 - implement flash-eeprom emulation to store settings in bvm memory
      
      
Coming soon:
  - selectable control method. (Uart,I2C,ADC,PPM)
  
  
If you have any feature requests, open up an issue and I will get back to you a.s.a.p!

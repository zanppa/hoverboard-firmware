This HAL driver is a combination of:
 1. Archives/STM32F1xx_HAL_Driver
 2. Archives/AT32F4xx_StdPeriph_Driver
 
This combination was possible due to the high similarity of the AT32
chip compared to the STM32 originals. It allows the use of only 1
peripheral library.

When using an STM32 MCU, define the right processor type in the MakeFile
For the AT32, only AT32F403Rx_HD is supported

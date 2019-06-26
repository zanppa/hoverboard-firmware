### STM32 HAL adaption to AT32 ###

The AT32 is mostly pin and register compatible to the STM32, so the main work was to rename some stuff and 
make it work with the different names from the AT32f403xx.h file

I aliased some stuff, others I copied the register and bit definition from the stm files.

There are however some differences regarding the flash and the rcc, that have to be checked.
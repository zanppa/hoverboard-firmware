######################################
# target
######################################
TARGET = hover

######################################
# building variables
######################################
# debug build?
DEBUG = 1
# optimization
OPT = -Og

# Build path
BUILD_DIR = build

######################################
# source
######################################
# C sources
C_SOURCES =  \
Drivers/Modified_HAL_Driver/Src/stm32f1xx_hal_flash.c \
Drivers/Modified_HAL_Driver/Src/stm32f1xx_hal_flash_ex.c \
Drivers/Modified_HAL_Driver/Src/stm32f1xx_hal_rcc.c \
Drivers/Modified_HAL_Driver/Src/stm32f1xx_hal_tim.c \
Drivers/Modified_HAL_Driver/Src/stm32f1xx_hal_tim_ex.c \
Drivers/Modified_HAL_Driver/Src/stm32f1xx_hal_gpio_ex.c \
Drivers/Modified_HAL_Driver/Src/stm32f1xx_hal_adc_ex.c \
Drivers/Modified_HAL_Driver/Src/stm32f1xx_hal_cortex.c \
Drivers/Modified_HAL_Driver/Src/stm32f1xx_hal_gpio.c \
Drivers/Modified_HAL_Driver/Src/stm32f1xx_hal_rcc_ex.c \
Drivers/Modified_HAL_Driver/Src/stm32f1xx_hal_pwr.c \
Drivers/Modified_HAL_Driver/Src/stm32f1xx_hal.c \
Drivers/Modified_HAL_Driver/Src/stm32f1xx_hal_adc.c \
Drivers/Modified_HAL_Driver/Src/stm32f1xx_hal_uart.c \
Drivers/Modified_HAL_Driver/Src/stm32f1xx_hal_i2c.c \
Drivers/Modified_HAL_Driver/Src/stm32f1xx_hal_dma.c \
Src/system_at32f4xx.c \
Src/setup.c \
Src/control.c \
Src/main.c \
Src/bldc.c \
Src/comms.c \
Src/stm32f1xx_it.c \
Src/BLDC_controller_data.c \
Src/BLDC_controller.c



# ASM sources
ASM_SOURCES =  \
startup_at32f403xe.s

#######################################
# binaries
#######################################
PREFIX = arm-none-eabi-
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
AR = $(PREFIX)ar
SZ = $(PREFIX)size
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S

#######################################
# CFLAGS
#######################################
# cpu
CPU = -mcpu=cortex-m4

# fpu
# NONE for Cortex-M0/M0+/M3
FPU=-mfpu=fpv4-sp-d16
# float-abi
FLOAT-ABI=-mfloat-abi=softfp

# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# macros for gcc
# AS defines
AS_DEFS =

# C defines
C_DEFS =  \
-DUSE_HAL_DRIVER \
-DAT32F403Rx_HD


# AS includes
AS_INCLUDES =

# C includes
C_INCLUDES =  \
-IInc \
-IDrivers/Modified_HAL_Driver/Inc \
-IDrivers/CMSIS/CM4/DeviceSupport \
-IDrivers/CMSIS/CM4/CoreSupport \

#-IDrivers/stm32f1xx_StdPeriph_Driver/Inc \

# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections -std=gnu11

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif


# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)"


#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = AT32F403RCTx_FLASH.ld

# libraries
LIBS = -lc -lm -lnosys
LIBDIR =
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin


#######################################
# build the application
#######################################
# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Inc/config.h Makefile | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@ -include Inc/stm32f1xx_conf.h

$(BUILD_DIR)/%.o: %.s Inc/config.h Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@

$(BUILD_DIR):
	mkdir -p $@

format:
	find Src/ Inc/ -iname '*.h' -o -iname '*.c' | xargs clang-format -i
#######################################
# clean up
#######################################
clean:
	-rm -fR .dep $(BUILD_DIR)

flash:
	st-flash --reset write $(BUILD_DIR)/$(TARGET).bin 0x8000000

flash-jlink:
	 JLinkExe -if swd -device Cortex-M4 -speed 4000 -SettingsFile .\JLinkSettings.ini -CommanderScript jlink-command.jlink

unlock:
	openocd -f interface/stlink-v2.cfg -f target/stm32f1x.cfg -c init -c "reset halt" -c "stm32f1x unlock 0"

#######################################
# dependencies
#######################################
-include $(shell mkdir .dep 2>/dev/null) $(wildcard .dep/*)

# *** EOF ***

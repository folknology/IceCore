# gnu makefile to build arm program to load ice40 bitstreams
#   using STM32Cube library
#
# make elf	- build executable (can be loaded by openocd)
# make hex	- build intelhex file (can be loaded by stm32flash)
# make flash	- build and load with stm32flash
# make oflash	- build and load with openocd
# make dfu      - build raw image and load with dfu-util

TARGET	= mystorm
CPP	= mystorm.cpp \
		main.cpp
OCDCONFIG = stlink.cfg

# default target
all: elf

# select STM32 SoC
MCU	= STM32F730XX
ASM	= ../startup_stm32f730xx.s
LDSCRIPT = ../STM32F730R8Tx_FLASH.ld

# select ARM architecture
ARCH	= -mcpu=cortex-m7

# select floating-point support
#FPU = -mfloat-abi=hard -mfpu=fpv5-sp-d16

# uncomment to include debug symbols
DEBUG	= -g

# directories
#OUTPUT_DIR	= $(TARGET)_output
OUTPUT_DIR	= output
OBJ_DIR		= $(OUTPUT_DIR)/obj

DRIVERS		= ../Drivers/STM32F7xx_HAL_Driver
USB			= ../Middlewares/ST/STM32_USB_Device_Library
CMSIS		= ../Drivers/CMSIS/Device/ST/STM32F7xx

INCLUDE	= \
	-I$(USB)/Core/Inc \
	-I$(USB)/Class/CDC/Inc \
	-I../Inc \
	-I../Drivers/CMSIS/Include \
	-I$(DRIVERS)/Inc \
	-I$(DRIVERS)/Inc/Legacy \
	-I$(CMSIS)/Include \

VPATH	= \
	$(DRIVERS)/Src \
	$(USB)/Core/Src \
	$(USB)/Class/CDC/Src \
	$(CMSIS)/Source/Templates/gcc \
	$(CMSIS)/Source/Templates

# source and object files
SRC =  \
	usbd_cdc_if.c \
	usbd_conf.c \
	usbd_desc.c \
	usb_device.c \
	stm32f7xx_it.c \
	stm32f7xx_hal_msp.c \
	system_stm32f7xx.c 

HAL_SRC := $(wildcard $(DRIVERS)/Src/*.c) $(wildcard $(USB)/Core/Src/*.c) $(wildcard $(USB)/Class/CDC/Src/*.c)

HAL_OBJECTS	:= $(addprefix $(OBJ_DIR)/,$(notdir $(HAL_SRC:.c=.o)))
ASM_OBJECTS	:= $(addprefix $(OBJ_DIR)/,$(notdir $(ASM:.s=.o)))
SRC_OBJECTS	:= $(addprefix $(OBJ_DIR)/,$(notdir $(SRC:.c=.o)))
CPP_OBJECTS	:= $(addprefix $(OBJ_DIR)/,$(notdir $(CPP:.cpp=.o)))

# C defines
C_DEFS =  \
-DUSE_HAL_DRIVER \
-DSTM32F730xx

GCCPATH = /home/awood/arm-gcc/gcc-arm-none-eabi-8-2018-q4-major/bin

# toolchain definitions
CP	= $(GCCPATH)/arm-none-eabi-g++
CC	= $(GCCPATH)/arm-none-eabi-gcc
AR	= $(GCCPATH)/arm-none-eabi-ar
OC  = $(GCCPATH)/arm-none-eabi-objcopy
SZ  = $(GCCPATH)/arm-none-eabi-size

ASMFLAGS = -x assembler-with-cpp

CPPFLAGS	= -D$(MCU) $(C_DEFS) $(INCLUDE)

CPFLAGS = -ffunction-sections -fdata-sections -O2 -Wall 
CPFLAGS += -mthumb $(ARCH) $(FPU) $(DEBUG)

CFLAGS	= -ffunction-sections -fdata-sections -O2 --std=c11 -Wall
#CFLAGS	= -ffunction-sections -fdata-sections -O0 -Werror --std=c11 -Wall
CFLAGS	+= -mthumb $(ARCH) $(FPU) $(DEBUG)

LDFLAGS	= -Wl,--gc-sections -Wall -specs=nosys.specs
#LDFLAGS += -Wl,-Map=$(OUTPUT_DIR)/$(TARGET).map,--cref
LDFLAGS += -mthumb $(ARCH) $(FPU) $(DEBUG)

#$(DRIVERS)/Src/stm32f7xx_ll_usb.c: CPPFLAGS += -Wno-attributes
#$(OBJ_DIR)/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_ll_usb.o: CPPFLAGS += -Wno-attributes

# toolchain recipes
$(OBJ_DIR)/%.o: %.cpp
	$(CP) -c $(CPPFLAGS) $(CPFLAGS) $< -o $@

$(OBJ_DIR)/%.o: %.c
	$(CC) -c $(CPPFLAGS) $(CFLAGS) $< -o $@

$(OBJ_DIR)/%.o: %.s
	$(CC) -c $(ASMFLAGS) $(CPPFLAGS) $(CFLAGS) $< -o $@

PAD = $(OUTPUT_DIR)/$(TARGET).pad
$(OUTPUT_DIR)/$(TARGET).elf : $(HAL_OBJECTS) $(ASM_OBJECTS) $(CPP_OBJECTS) $(SRC_OBJECTS) $(LDSCRIPT)
	$(CC) $(LDFLAGS) $(ASM_OBJECTS) $(CPP_OBJECTS) $(SRC_OBJECTS) $(HAL_OBJECTS) -T$(LDSCRIPT) -o $@
	$(SZ) $@ | tail -1 | awk '{printf "0x%x\n",int((0x08000000+$$1+$$2+7)/8)*8}' > $(PAD)

# dependency rules

HEADERS := $(wildcard ../Inc/*.h \
	../Drivers/CMSIS/Include/*.h \
	../Drivers/CMSIS/Device/ST/STM32F7xx/Include/*.h \
	../Drivers/STM32F7xx_HAL_Driver/Inc/*.h \
	../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy/*.h \
	../Middlewares/ST/STM32_USB_Device_Library/Core/Inc/*.h \
	../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc/*.h)

$(CPP_OBJECTS) $(SRC_OBJECTS) $(HAL_OBJECTS): $(HEADERS)
$(ASM_OBJECTS) $(CPP_OBJECTS) $(SRC_OBJECTS) $(HAL_OBJECTS): | $(OBJ_DIR)

$(OBJ_DIR):
	-mkdir -p $(OBJ_DIR)

.INTERMEDIATE: $(HAL_OBJECTS)

$(OUTPUT_DIR)/$(TARGET).hex : $(OUTPUT_DIR)/$(TARGET).elf
	$(OC) -O ihex --pad-to `cat $(PAD)` $< $@

$(OUTPUT_DIR)/$(TARGET).raw: $(OUTPUT_DIR)/$(TARGET).elf
	$(OC) -O binary --pad-to `cat $(PAD)` $< $@

# top-level targets
clean:
	-rm -r $(OUTPUT_DIR)

elf: $(OUTPUT_DIR)/$(TARGET).elf

hex: $(OUTPUT_DIR)/$(TARGET).hex

raw: $(OUTPUT_DIR)/$(TARGET).raw

flash: hex
	stm32flash -w $(OUTPUT_DIR)/$(TARGET).hex -v -g 0x0 /dev/ttyAMA0

oflash: elf
	openocd -f $(OCDCONFIG) -c "program $(OUTPUT_DIR)/$(TARGET).elf verify reset exit"

dfu: raw
	dfu-util -s 0x08000000:leave -a 0 -D $(OUTPUT_DIR)/$(TARGET).raw -t 1024

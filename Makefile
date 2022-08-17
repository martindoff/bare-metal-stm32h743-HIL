######################################################
#                                                    #
#                     Makefile                       #
# Martin Doff-Sotta (martin.doff-sotta@eng.ox.ac.uk) #
#                                                    #
######################################################

TARGET = main

MCU_SPEC = cortex-m7

# Define the chip architecture.
LD_SCRIPT = STM32H743VITX_FLASH.ld

TOOLCHAIN = /opt/gcc-arm/bin
CC  = $(TOOLCHAIN)/arm-none-eabi-gcc
CPP = $(TOOLCHAIN)/arm-none-eabi-g++
AS  = $(TOOLCHAIN)/arm-none-eabi-as
LD  = $(TOOLCHAIN)/arm-none-eabi-ld
OC  = $(TOOLCHAIN)/arm-none-eabi-objcopy
OD  = $(TOOLCHAIN)/arm-none-eabi-objdump
OS  = $(TOOLCHAIN)/arm-none-eabi-size

# Assembly directives.
ASFLAGS += -mcpu=$(MCU_SPEC)
ASFLAGS += -mthumb

# C compilation directives
CFLAGS += -mcpu=$(MCU_SPEC)
CFLAGS += -mthumb
CFLAGS += -mhard-float
CFLAGS += -mfloat-abi=hard
CFLAGS += -mfpu=fpv5-d16
CFLAGS += -Wall
CFLAGS += -g
CFLAGS += -Os
CFLAGS += -fmessage-length=0 -fno-common
CFLAGS += -ffunction-sections -fdata-sections
CFLAGS += -std=c99

# C++ compilation directives
CPPFLAGS += -mcpu=$(MCU_SPEC)
CPPFLAGS += -mthumb
CPPFLAGS += -mhard-float
CPPFLAGS += -mfloat-abi=hard
CPPFLAGS += -mfpu=fpv5-d16
CPPFLAGS += -Wall
CPFLAGS += -g
CPPFLAGS += -Os
CPPFLAGS += -fmessage-length=0 -fno-common
CPPFLAGS += -ffunction-sections -fdata-sections
CPPFLAGS += -fno-exceptions

# Linker directives.
LSCRIPT = ./$(LD_SCRIPT)
LFLAGS += -mcpu=$(MCU_SPEC)
LFLAGS += -mthumb
LFLAGS += -mhard-float
LFLAGS += -mfloat-abi=hard
LFLAGS += -mfpu=fpv5-d16
LFLAGS += -Wall
LFLAGS += --specs=nosys.specs
LFLAGS += --static
LFLAGS += -Wl,-Map=$(TARGET).map
LFLAGS += -Wl,--gc-sections
LFLAGS += -lgcc
LFLAGS += -lc
LFLAGS += -T$(LSCRIPT)

# Source files.
AS_SRC    = ./startup/startup_stm32h743vitx.s
C_SRC   = ./src/main.c
C_SRC  += ./src/system_stm32h7xx.c

INCLUDE  += -I./include
INCLUDE  += -I./include
INCLUDE  += -I./drivers

OBJS  = $(C_SRC:.c=.o)
OBJS += $(CPP_SRC:.cpp=.o)
OBJS += $(AS_SRC:.s=.o)

.PHONY: all
all: $(TARGET).bin

%.o: %.s
	$(AS) $(ASFLAGS) -c $< -o $@

%.o: %.c
	$(CC) -c $(CFLAGS) $(INCLUDE) $< -o $@

%.o: %.cpp
	$(CPP) -c $(CPPFLAGS) $(INCLUDE) $< -o $@

$(TARGET).elf: $(OBJS)
	$(CPP) $^ $(LFLAGS) -o $@

$(TARGET).bin: $(TARGET).elf
	$(OC) -S -O binary $< $@
	$(OS) $<

.PHONY: flash 
flash: all
	st-flash erase 
	st-flash write ./$(TARGET).bin 0x08000000 

.PHONY: clean
clean:
	rm -f $(OBJS)
	rm -f $(TARGET).elf
	rm -f $(TARGET).bin
	rm -f $(TARGET).map
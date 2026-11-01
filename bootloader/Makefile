#############################################################################
# STM32F103 Bare-metal USB DFU Bootloader Makefile
#############################################################################

# Toolchain
CROSS_COMPILE ?= arm-none-eabi-
CC = $(CROSS_COMPILE)gcc
AS = $(CROSS_COMPILE)as
LD = $(CROSS_COMPILE)ld
OBJCOPY = $(CROSS_COMPILE)objcopy
SIZE = $(CROSS_COMPILE)size

# Target
TARGET = bootloader

# Directories
SRC_DIR = src
INC_DIR = inc
BUILD_DIR = build

# Source files
SRCS = $(SRC_DIR)/startup.c \
       $(SRC_DIR)/main.c \
       $(SRC_DIR)/system.c \
       $(SRC_DIR)/gpio.c \
       $(SRC_DIR)/flash.c \
       $(SRC_DIR)/usb_dfu.c

# Object files
OBJS = $(patsubst $(SRC_DIR)/%.c,$(BUILD_DIR)/%.o,$(SRCS))

# Linker script
LDSCRIPT = stm32f103_bootloader.ld

# Compiler flags
CFLAGS = -mcpu=cortex-m3 -mthumb -Os -g
CFLAGS += -ffunction-sections -fdata-sections
CFLAGS += -Wall -Wextra
CFLAGS += -I$(INC_DIR)
CFLAGS += -DSTM32F103xB

# Linker flags
LDFLAGS = -mcpu=cortex-m3 -mthumb
LDFLAGS += -T$(LDSCRIPT)
LDFLAGS += -Wl,--gc-sections
LDFLAGS += -nostartfiles
LDFLAGS += -specs=nano.specs

# Build rules
all: $(BUILD_DIR) $(TARGET).bin size

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c
	$(CC) $(CFLAGS) -c $< -o $@

$(TARGET).elf: $(OBJS)
	$(CC) $(LDFLAGS) $(OBJS) -o $@

$(TARGET).bin: $(TARGET).elf
	$(OBJCOPY) -O binary $< $@

size: $(TARGET).elf
	$(SIZE) $<

clean:
	rm -rf $(BUILD_DIR) $(TARGET).elf $(TARGET).bin

flash: $(TARGET).bin
	st-flash write $(TARGET).bin 0x08000000

.PHONY: all clean flash size

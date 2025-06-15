# Project Name
PROJECT = BLE_USB

# MCU and Arch Settings
MCU = CH32V208GBU6
ARCH = rv32imac
ABI = ilp32

# Toolchain prefix (can be overridden from command line)
CROSS_COMPILE ?= riscv-none-embed-
TOOLCHAIN_PREFIX = $(CROSS_COMPILE)

# Tools
CC = $(TOOLCHAIN_PREFIX)gcc
AS = $(TOOLCHAIN_PREFIX)gcc -x assembler-with-cpp
CP = $(TOOLCHAIN_PREFIX)objcopy
SZ = $(TOOLCHAIN_PREFIX)size
AR = $(TOOLCHAIN_PREFIX)ar
HEX = $(CP) -O ihex

# Build path (can be overridden from command line)
BUILD_DIR ?= obj

# Optimization settings
OPT = -Os

# C compilation flags
CFLAGS = $(OPT)
CFLAGS += -march=$(ARCH) -mabi=$(ABI)
CFLAGS += -fdata-sections -ffunction-sections
CFLAGS += -fno-common
CFLAGS += --specs=nano.specs
CFLAGS += -std=gnu99
CFLAGS += -Wall

# Defined symbols
CFLAGS += -DBLE_BUFF_MAX_LEN=516
CFLAGS += -DCH32V20x_D8W
CFLAGS += -DBLE_MEMHEAP_SIZE=10240
CFLAGS += -DBLE_TX_NUM_EVENT=5

# Include paths
CFLAGS += -I./SRC/Core
CFLAGS += -I./SRC/Startup
CFLAGS += -I./SRC/Debug
CFLAGS += -I./SRC/Peripheral/inc
CFLAGS += -I./APP/include
CFLAGS += -I./Profile/include
CFLAGS += -I./HAL/include
CFLAGS += -I./LIB
CFLAGS += -I./APP/UART
CFLAGS += -I./APP/USBLIB/CONFIG
CFLAGS += -I./APP/USBLIB/USB-Driver/inc

# AS compilation flags
AS_INCLUDES = 
AS_FLAGS = $(CFLAGS)

# Linker flags
LDSCRIPT = ./HAL/Link.ld
LIBS = -lwchble -lm
LIBDIR = -L../ -L./LIB
LDFLAGS = -T$(LDSCRIPT) $(LIBDIR) $(LIBS)
LDFLAGS += -march=$(ARCH) -mabi=$(ABI)
LDFLAGS += -Wl,--gc-sections
LDFLAGS += -Wl,--print-memory-usage
LDFLAGS += -nostartfiles
LDFLAGS += --specs=nano.specs

# Get all source files recursively, excluding specific files
FIND_SRCS = $(shell find $(1) -name '*.c' -not -path './APP/include/app_usb.h' \
                                       -not -path './APP/app_usb.c' \
                                       -not -path './HAL/Profile/*' \
                                       -not -path './HAL/KEY.c' \
                                       -not -path './HAL/LED.c')

FIND_ASMS = $(shell find $(1) -name '*.S' -not -path './SRC/Startup/startup_ch32v20x_D8.S' \
                                       -not -path './SRC/Startup/startup_ch32v20x_D6.S' \
                                       -not -path './SRC/Ld/*')

# Source paths
C_SOURCES = $(call FIND_SRCS, .)
ASM_SOURCES = $(call FIND_ASMS, .)

# Build objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.S=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
vpath %.S $(sort $(dir $(ASM_SOURCES)))

# Build all target
all: $(BUILD_DIR) $(BUILD_DIR)/$(PROJECT).elf $(BUILD_DIR)/$(PROJECT).hex $(BUILD_DIR)/$(PROJECT).lst

# Build the project
$(BUILD_DIR)/$(PROJECT).elf: $(OBJECTS)
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf
	$(HEX) $< $@

$(BUILD_DIR)/%.lst: $(BUILD_DIR)/%.elf
	$(TOOLCHAIN_PREFIX)objdump -h -S $< > $@

# Compile C sources
$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

# Assemble ASM sources
$(BUILD_DIR)/%.o: %.S Makefile | $(BUILD_DIR)
	$(AS) -c $(AS_FLAGS) $< -o $@

# Create build directory
$(BUILD_DIR):
    mkdir -p $@

# Clean up
clean:
	rm -rf $(BUILD_DIR)

# Flash the device (add your flashing tool command here)
flash: all
    # Add your flash command - example for OpenOCD
    # openocd -f interface/wch-link.cfg -f target/ch32v20x.cfg -c "program $(BUILD_DIR)/$(PROJECT).elf verify reset exit"

# Build target (alias for 'all')
build: all

.PHONY: all clean flash build
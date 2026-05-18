# -- Адаптированная версия для MIK32 (RISC-V) на основе старого стиля
#  Part of Grbl
#
#  Copyright (c) 2009-2011 Simen Svale Skogsrud
#  Copyright (c) 2012-2015 Sungeun K. Jeon
#
#  Grbl is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  Grbl is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.


# This is a prototype Makefile. Modify it according to your needs.
# You should at least check the settings for
# DEVICE ....... The RISC-V device you compile for
# CLOCK ........ Target CPU clock rate in Hertz
# OBJECTS ...... The object files created from your source files. This list is
#                usually the same as the list of source files with suffix ".o".
# PROGRAMMER ... Options for uploading to the MIK32

# Target configuration
TARGET = firmware
DEVICE = mik32v2
CLOCK = 48000000
ARCH = rv32imc
ABI = ilp32

# Directories
BUILDDIR = build
SRCDIR = src
LIBDIR = lib

# Toolchain - адаптируйте пути под вашу систему
# Для Linux (если компилятор установлен системно):
GCC_PATH = riscv-none-embed-
# Для Windows раскомментируйте:
# GCC_PATH = D:/Distributive/TOOL_CHAIN/xpack-riscv-none-embed-gcc-10.1.0-1.1/bin/riscv-none-embed-
# Для Linux с локальной установкой:
# GCC_PATH = /opt/xpack-riscv-none-embed-gcc/bin/riscv-none-embed-

CC = $(GCC_PATH)gcc
CXX = $(GCC_PATH)g++
AS = $(GCC_PATH)gcc -x assembler-with-cpp
OBJCOPY = $(GCC_PATH)objcopy
SIZE = $(GCC_PATH)size

# Source files (основные исходники Grbl)
C_SOURCES = \
	src/main.cpp \
	lib/grbl/src/eeprom.cpp \
	lib/grbl/src/settings.cpp \
	lib/grbl/src/planner.cpp \
	lib/grbl/src/nuts_bolts.cpp \
	lib/grbl/src/limits.cpp \
	lib/grbl/src/print.cpp \
	lib/grbl/src/probe.cpp \
	lib/grbl/src/report.cpp \
	lib/grbl/src/system.cpp \
	lib/grbl/src/motion_control.cpp \
	lib/grbl/src/gcode.cpp \
	lib/grbl/src/spindle_control.cpp \
	lib/grbl/src/coolant_control.cpp \
	lib/grbl/src/serial.cpp \
	lib/grbl/src/protocol.cpp \
	lib/grbl/src/stepper.cpp \
	lib/grbl/src/jog.cpp

# Assembly sources
ASM_SOURCES = \
	../libs/mik32v2-shared/runtime/crt0.S

# Include directories
C_INCLUDES = \
	-I. \
	-Isrc \
	-Ilib/grbl/src \
	-I../libs/mik32v2-shared/include \
	-I../libs/mik32v2-shared/periphery \
	-I../libs/mik32v2-shared/libs \
	-I../libs/mik32-hal/peripherals/Include \
	-I../libs/mik32-hal/utilities/Include \
	-I../libs/mik32-hal/core/Include

# Compiler flags
MCU = -march=$(ARCH) -mabi=$(ABI) -MD -fstrict-volatile-bitfields -fno-strict-aliasing -fno-common -fno-builtin-printf
OPT = -Os
CFLAGS = $(MCU) $(C_INCLUDES) $(OPT) -MMD -MP -MF"$(@:%.o=%.d)" -g -std=gnu11
CXXFLAGS = $(MCU) $(C_INCLUDES) $(OPT) -MMD -MP -MF"$(@:%.o=%.d)" -g -std=gnu++11
ASFLAGS = $(MCU) $(AS_INCLUDES) $(OPT) -MMD -MP -MF"$(@:%.o=%.d)"

# Linker script
LDSCRIPT = ../libs/mik32v2-shared/ldscripts/eeprom.ld
# LDSCRIPT = ../libs/mik32v2-shared/ldscripts/spifi.ld

# Linker flags
LIBDIR =
LIBS = -lnosys -lgcc
LDFLAGS = $(MCU) --specs=nano.specs -nostdlib -mcmodel=medlow $(LIBDIR) $(LIBS) -nostartfiles -ffreestanding $(OPT) \
	-L ../libs/mik32v2-shared/ldscripts -Wl,-Bstatic,-T,$(LDSCRIPT),-Map,$(BUILDDIR)/$(TARGET).map,--print-memory-usage,--gc-sections

# Object files
OBJECTS = $(addprefix $(BUILDDIR)/,$(notdir $(C_SOURCES:.cpp=.o)))
vpath %.cpp $(sort $(dir $(C_SOURCES)))

OBJECTS += $(addprefix $(BUILDDIR)/,$(notdir $(ASM_SOURCES:.S=.o)))
vpath %.S $(sort $(dir $(ASM_SOURCES)))

# Upload tools (адаптируйте пути)
OCD = ../libs/openocd/bin/openocd
OPENOCD_CFG = ../libs/Util/m-link.cfg
OPENOCD_TARGET_CFG = ../libs/Util/mik32.cfg
MIK32_UPLOADER_DIR = ../libs/mik32-uploader

# Tune the lines below only if you know what you are doing:

# symbolic targets:
all: $(BUILDDIR)/$(TARGET).hex

$(BUILDDIR)/%.o: %.cpp
	@echo "  CXX $<"
	@mkdir -p $(dir $@)
	$(CXX) -c $(CXXFLAGS) -MT"$(@)" -c "$<" -o "$@"

$(BUILDDIR)/%.o: %.c
	@echo "  CC  $<"
	@mkdir -p $(dir $@)
	$(CC) -c $(CFLAGS) -MT"$(@)" -c "$<" -o "$@"

$(BUILDDIR)/%.o: %.S
	@echo "  AS  $<"
	@mkdir -p $(dir $@)
	$(AS) -c $(ASFLAGS) -MT"$(@)" -c "$<" -o "$@"

$(BUILDDIR)/$(TARGET).elf: $(OBJECTS)
	@echo "  LD  $@"
	$(CXX) $(OBJECTS) $(LDFLAGS) -o $@
	$(SIZE) $@

$(BUILDDIR)/%.hex: $(BUILDDIR)/%.elf
	@echo "  HEX $@"
	$(OBJCOPY) -O ihex $< $@

$(BUILDDIR)/%.bin: $(BUILDDIR)/%.elf
	@echo "  BIN $@"
	$(OBJCOPY) -O binary -S $< $@

# Upload targets (аналогично старому 'flash' target)
flash: $(BUILDDIR)/$(TARGET).hex
	python $(MIK32_UPLOADER_DIR)/mik32_upload.py $(BUILDDIR)/$(TARGET).hex \
		--run-openocd \
		--openocd-exec=$(OCD) \
		--openocd-interface=$(OPENOCD_CFG) \
		--openocd-target=$(OPENOCD_TARGET_CFG)

# Clean target (адаптирован для Linux/Windows)
clean:
	rm -rf $(BUILDDIR)

# Install target (как старый install)
install: flash

# Debug targets
disasm: $(BUILDDIR)/$(TARGET).elf
	$(GCC_PATH)objdump -d $(BUILDDIR)/$(TARGET).elf

size: $(BUILDDIR)/$(TARGET).elf
	$(SIZE) $(BUILDDIR)/$(TARGET).elf

# Include generated dependencies
-include $(OBJECTS:.o=.d)

.PHONY: all clean flash install disasm size

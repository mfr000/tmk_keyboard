# Target file name (without extension).
PROJECT = ch

# Directory common source files exist
TMK_DIR = ../../tmk_core

# Directory keyboard dependent files exist
TARGET_DIR = .

# project specific files
SRC =	matrix.c \
	keymap_phantom.c \
	led.c

CONFIG_H = config.h

## chip/board settings
# - the next two should match the directories in
#   <chibios>/os/hal/ports/$(MCU_FAMILY)/$(MCU_SERIES)
# - For Teensies, FAMILY = KINETIS and SERIES is either
#   KL2x (LC) or K20x (3.0,3.1,3.2).
# - For Maple-Mini and clones, FAMILY=STM32 and SERIES is STM32F1xx
MCU_FAMILY = STM32
MCU_SERIES = STM32F1xx

# Linker script to use
# - it should exist either in <chibios>/os/common/ports/ARMCMx/compilers/GCC/ld/
#   or <this_dir>/ld/
# - NOTE: a custom ld script is needed for EEPROM on Teensy LC
# - LDSCRIPT =
#   - MKL26Z64 for Teensy LC
#   - MK20DX128 for Teensy 3.0
#   - MK20DX256 for Teensy 3.1 and 3.2
# - 
#MCU_LDSCRIPT = MK20DX128
#MCU_LDSCRIPT = MK20DX128BLDR4
MCU_LDSCRIPT = STM32F103xB_maplemini

# Startup code to use
#  - it should exist in <chibios>/os/common/ports/ARMCMx/compilers/GCC/mk/
# - STARTUP =
#   - kl2x for Teensy LC
#   - k20x5 for Teensy 3.0
#   - k20x7 for Teensy 3.1 and 3.2
#   - smt32f[012347]xx for STM32
MCU_STARTUP = stm32f1xx

# Board: it should exist either in <chibios>/os/hal/boards/
#  or <this_dir>/boards
# - BOARD =
#   - PJRC_TEENSY_LC for Teensy LC
#   - PJRC_TEENSY_3 for Teensy 3.0
#   - PJRC_TEENSY_3_1 for Teensy 3.1 or 3.2
#   - MAPLEMINI_STM32_F103 for Maple Mini and clones
BOARD = MAPLEMINI_STM32_F103

# Cortex version
# Teensy LC is cortex-m0; Teensy 3.x are cortex-m4
# Maple-Mini's STM32F103 is cortex-m3
MCU  = cortex-m3

# ARM version, CORTEX-M0/M1 are 6, CORTEX-M3/M4/M7 are 7
# I.e. 6 for Teensy LC; 7 for Teensy 3.x
ARMV = 7

# Vector table for application
# 0x00000000-0x00001000 area is occupied by bootlaoder.*/
# For Maple-Mini, bootloader occupies 0x000000000-0x08005000
# but is defined when USE_MAPLEMINI_BOOTLOADER is defined.
#OPT_DEFS = -DCORTEX_VTOR_INIT=0x00001000
#OPT_DEFS = -DCORTEX_VTOR_INIT=0x00005000 -DUSE_MAPLEMINI_BOOTLOADER
OPT_DEFS = -DUSE_MAPLEMINI_BOOTLOADER


# Build Options
#   comment out to disable the options.
#
#BOOTMAGIC_ENABLE = yes	# Virtual DIP switch configuration
## (Note that for BOOTMAGIC on Teensy LC you have to use a custom .ld script.)
#MOUSEKEY_ENABLE = yes	# Mouse keys
EXTRAKEY_ENABLE = yes	# Audio control and System control
#CONSOLE_ENABLE = yes	# Console for debug
COMMAND_ENABLE = yes    # Commands for debug and configuration
#SLEEP_LED_ENABLE = yes  # Breathing sleep LED during USB suspend
NKRO_ENABLE = yes	    # USB Nkey Rollover

include $(TMK_DIR)/tool/chibios/common.mk
include $(TMK_DIR)/tool/chibios/chibios.mk

ansi: OPT_DEFS += -DLAYOUT_ANSI
ansi: all

ansi_150: OPT_DEFS += -DLAYOUT_ANSI_150
ansi_150: all

iso: OPT_DEFS += -DLAYOUT_ISO
iso: all

iso_150: OPT_DEFS += -DLAYOUT_ISO_150
iso_150: all

7bit: OPT_DEFS += -DLAYOUT_7BIT
7bit: all

program: $(BUILDDIR)/$(PROJECT).bin
	dfu-util -D $(BUILDDIR)/$(PROJECT).bin

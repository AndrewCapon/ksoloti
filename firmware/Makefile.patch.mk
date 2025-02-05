BOARDDEF =
FWOPTIONDEF =
PATCH_ITCM = 1

# Some new options are important to keep
# SRAM usage and DSP load low with newer GCC versions.
# "--param max-completely-peeled-insns=100" makes a big difference to get SRAM down. Newer GCC versions use 200 here, original axoloti (GCC 4.9) used 100.
# below the single backslash line are options which are unknown to make any difference so far
# CCFLAGS =  -mcpu=cortex-m7 -O3 -ggdb -fomit-frame-pointer -falign-functions=16 -DBOARD_KSOLOTI_CORE -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fno-rtti -Wall -Wextra -Wa,-alms=build/ksoloti/normal/lst/modulator.lst  -DCORTEX_USE_FPU=TRUE -DCORE_CM7 \
#     -Wno-implicit-fallthrough \
#     -Wno-unused-parameter \
#     -Wno-unused-function \
#     -Wno-return-type \
           
CCFLAGS = \
    -DCORTEX_USE_FPU=TRUE -DCORE_CM7 \
    -Wno-implicit-fallthrough \
    -Wno-unused-parameter \
    -Wno-return-type \
    -ggdb3 \
    -mcpu=cortex-m7 \
    -mfloat-abi=hard \
    -mfpu=fpv5-sp-d16 \
    -mthumb \
    -mtune=cortex-m7 \
    -mword-relocations \
    -nostartfiles \
    -nostdlib \
    -std=c++11 \
    -O3 \
    --param max-completely-peeled-insns=100 \
    -fcode-hoisting \
    -fno-threadsafe-statics \
    -ffunction-sections \
    -fdata-sections \
    -fno-common \
    -fno-math-errno \
    -fno-reorder-blocks \
    -fno-rtti \
    -mno-thumb-interwork \
    -fno-use-cxa-atexit \
    -fpermissive \
    -ffast-math \
    

# CCFLAGS = \
#     -Wno-implicit-fallthrough \
#     -Wno-unused-parameter \
#     -Wno-return-type \
#     -ggdb3 \
#     -mcpu=cortex-m7 \
#     -mfloat-abi=hard \
#     -mtune=cortex-m7 \
#     -mword-relocations \
#     -nostartfiles \
#     -nostdlib \
#     -std=c++11 \
#     -O0 \
#     --param max-completely-peeled-insns=100 \
#     -fcode-hoisting \
#     -fno-threadsafe-statics \
#     -ffunction-sections \
#     -fdata-sections \
#     -fno-common \
#     -fno-math-errno \
#     -fno-reorder-blocks \
#     -fno-rtti \
#     -fno-use-cxa-atexit \
#     -fpermissive \
#     -ffast-math \
#     \

DEFS = \
    -D$(BOARDDEF) \
    -DARM_MATH_CM5 \
    -DCORTEX_USE_FPU=TRUE \
    -DSTM32F455xx \
    -D__FPU_PRESENT \
    -DCORE_CM7 

ifneq ($(FWOPTIONDEF),)
  DEFS := $(DEFS) -D$(FWOPTIONDEF)
endif


ELFNAME=
ifeq ($(BOARDDEF),BOARD_KSOLOTI_CORE)
  ifeq ($(PATCH_ITCM),1)
    $(info Building for itcm)
    RAMLINKOPT = -Tramlink_ksoloti_itcm.ld
  else
    $(info Building for sram)
    RAMLINKOPT = -Tramlink_ksoloti.ld
  endif
  ELFNAME = ksoloti
else ifeq ($(BOARDDEF),BOARD_AXOLOTI_CORE)
  RAMLINKOPT = -Tramlink_axoloti.ld
  ELFNAME = axoloti
endif

ifeq ($(FWOPTIONDEF),FW_SPILINK)
  ELFNAME := $(ELFNAME)_spilink
endif

ifeq ($(FWOPTIONDEF),FW_USBAUDIO)
  ELFNAME := $(ELFNAME)_usbaudio
endif

# If enabled, this option allows to compile the application in THUMB mode.
ifeq ($(USE_THUMB),)
  USE_THUMB = no
endif

LDFLAGS = \
    $(RAMLINKOPT) \
    -Bsymbolic \
    -Wl,--gc-sections \
    -Wl,--print-memory-usage \
    -fno-common \
    -mcpu=cortex-m4 \
    -mfloat-abi=hard \
    -mfpu=fpv4-sp-d16 \
    -mtune=cortex-m4 \
    -nostartfiles

TRGT = arm-none-eabi-
CC=$(TRGT)gcc
CPP=$(TRGT)g++
LD=$(TRGT)gcc
CP=$(TRGT)objcopy
DMP=$(TRGT)objdump
SIZ=$(TRGT)size

axoloti_runtime ?= ..
axoloti_release ?= ..
axoloti_home ?= ..
axoloti_libraries ?= ..
axoloti_firmware ?= ../firmware

CHIBIOS = ${axoloti_release}/chibios
CMSIS = ${axoloti_release}/CMSIS

EMPTY := 
SPACE := $(EMPTY) $(EMPTY)
BUILDDIR=$(subst $(SPACE),\ ,${axoloti_libraries}/build)
FIRMWARE=$(subst $(SPACE),\ ,${axoloti_firmware})

$(info    BUILDDIR is $(BUILDDIR))
$(info    FIRMWARE is $(FIRMWARE))
$(info    ELFNAME is $(ELFNAME))

# Licensing files.
include $(CHIBIOS)/os/license/license.mk
# Startup files.
include $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/mk/startup_stm32h7xx.mk
# HAL-OSAL files (optional).
include $(CHIBIOS)/os/hal/hal.mk
include $(CHIBIOS)/os/hal/ports/STM32/STM32H7xx/platform.mk
include $(CHIBIOS)/os/hal/boards/ST_NUCLEO144_H755ZI/board.mk
include $(CHIBIOS)/os/hal/osal/rt-nil/osal.mk
# RTOS files (optional).
include $(CHIBIOS)/os/rt/rt.mk
include $(CHIBIOS)/os/common/ports/ARMCMx/compilers/GCC/mk/port_v7m.mk
# FAT stuff
include $(CHIBIOS)/os/various/fatfs_bindings/fatfs.mk

MCU  = cortex-m7

INCDIR = $(CMSIS)/Core/Include \
	 $(CMSIS)/DSP/Include \
         $(PORTINC) $(KERNINC) $(TESTINC) \
         $(HALINC) $(PLATFORMINC) $(BOARDINC) \
	 $(FATFSINC) \
         $(OSALINC) \
         ${FIRMWARE} \
	 $(CHIBIOS) \
         $(CHIBIOS)/os/various \
         ${FIRMWARE}/STM32F4xx_HAL_Driver/Inc \
	 ${FIRMWARE}/mutable_instruments \
	 $(CHIBIOS)/os/common/ports/ARMCMx/compilers/GCC \
         $(CHIBIOS)/os/common/ports/ARMCMx/devices/STM32F4xx \
         $(CHIBIOS)/os/ext/CMSIS/include \
         $(CHIBIOS)/os/ext/CMSIS/ST/STM32F4xx

# INCDIR = ${CMSIS}/Core/Include \
#          ${CMSIS}/DSP/Include \
#          $(STARTUPINC) \
#          $(PORTINC) $(KERNINC) $(TESTINC) \
#          $(HALINC) $(PLATFORMINC) $(BOARDINC) \
#          $(FATFSINC) \
#          $(OSALINC) \
#          $(CHIBIOS)/os/various \
#          ${FIRMWARE}/STM32_USB_Host_Library/Core/Inc \
#          ${FIRMWARE}/STM32_USB_Host_Library/Class/HID/Inc \
#          ${FIRMWARE}/STM32F4xx_HAL_Driver/Inc \
#          ${MUTABLE_INSTRUMENTS} \
#          ${MUTABLE_INSTRUMENTS}/warps \
#          ${MUTABLE_INSTRUMENTS}/braids \
#          ${MUTABLE_INSTRUMENTS}/rings \
#          ${MUTABLE_INSTRUMENTS}/elements \
#          ${MUTABLE_INSTRUMENTS}/stmlib \
# 	     $(CHIBIOS)/os/hal/lib/streams

# Paths
IINCDIR = $(patsubst %,-I%,$(INCDIR) $(DINCDIR) $(UINCDIR) $(CONFDIR) $(ALLINC))
LLIBDIR = $(patsubst %,-L%,$(DLIBDIR) $(ULIBDIR))

all: ${BUILDDIR}/xpatch.bin

${BUILDDIR}/xpatch.h.gch: ${FIRMWARE}/xpatch.h ${FIRMWARE}/patch.h ${FIRMWARE}/axoloti.h ${FIRMWARE}/parameter_functions.h ${FIRMWARE}/axoloti_math.h ${FIRMWARE}/axoloti_filters.h
#	@echo Building precompiled header
	@$(CPP) $(CCFLAGS) $(DEFS) $(IINCDIR) -Winvalid-pch -MD -MP -c ${FIRMWARE}/xpatch.h -o ${BUILDDIR}/xpatch.h.gch

${BUILDDIR}/xpatch.bin: ${BUILDDIR}/xpatch.cpp ${BUILDDIR}/xpatch.h.gch
#	@echo Removing previous build files
	@rm -f ${BUILDDIR}/xpatch.o ${BUILDDIR}/xpatch.elf ${BUILDDIR}/xpatch.bin ${BUILDDIR}/xpatch.d ${BUILDDIR}/xpatch.map ${BUILDDIR}/xpatch.lst ${BUILDDIR}/xpatch.siz
#	@echo Compiling patch dependencies
	@$(CPP) $(CCFLAGS) $(DEFS) -H $(IINCDIR) -Winvalid-pch -MD -MP --include ${BUILDDIR}/xpatch.h -c ${BUILDDIR}/xpatch.cpp -o ${BUILDDIR}/xpatch.o
#	@echo Linking patch dependencies
	@$(LD) $(LDFLAGS) ${BUILDDIR}/xpatch.o -Wl,-Map=${BUILDDIR}/xpatch.map,--cref,--just-symbols=${FIRMWARE}/build/$(ELFNAME).elf -o ${BUILDDIR}/xpatch.elf

#	@echo Creating binary
	@$(CP) -O binary ${BUILDDIR}/xpatch.elf ${BUILDDIR}/xpatch.bin

#	@echo Creating SIZe statistic file for debugging
	@$(SIZ) --format=sysv ${BUILDDIR}/xpatch.elf > ${BUILDDIR}/xpatch.siz
	@$(SIZ) --format=berkeley ${BUILDDIR}/xpatch.elf >> ${BUILDDIR}/xpatch.siz

#	@echo Creating LST file for debugging
	@$(DMP) --demangle --source-comment --disassemble ${BUILDDIR}/xpatch.elf > ${BUILDDIR}/xpatch.lst
#   (--source-comment now supported in gcc9) --line-numbers 

.PHONY: clean

clean:
	@rm -f ${BUILDDIR}/xpatch.o ${BUILDDIR}/xpatch.elf ${BUILDDIR}/xpatch.bin ${BUILDDIR}/xpatch.d ${BUILDDIR}/xpatch.map ${BUILDDIR}/xpatch.lst ${BUILDDIR}/xpatch.h.gch
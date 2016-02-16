# Get the $(FREERTOSPATH)
include $(OPENMRNPATH)/etc/freertos.mk

# Get the $(CFLAGSENV), $(CXXFLAGSENV), $(LDFLAGSENV)
include $(OPENMRNPATH)/etc/env.mk

# Get the $(TOOLPATH)
ifeq ($(TOOLPATH),)
TOOLPATH=$(MIPSGCCPATH)
endif

DEPS+= MIPSGCCPATH PIC32MXLIBPATH

PREFIX = $(TOOLPATH)/bin/mips-sde-elf-

AS = $(PREFIX)gcc
CC = $(PREFIX)gcc
CXX = $(PREFIX)g++
AR = $(PREFIX)ar
LD = $(PREFIX)g++
SIZE = $(PREFIX)size
OBJCOPY = $(PREFIX)objcopy
OBJDUMP = $(PREFIX)objdump

AROPTS=D

STARTGROUP := -Wl,--start-group
ENDGROUP := -Wl,--end-group

INCLUDES += -I$(FREERTOSPATH)/Source/include \
            -I$(FREERTOSPATH)/Source/portable/MPLAB/PIC32MX \
            -I$(OPENMRNPATH)/include/freertos \
            -idirafter $(OPENMRNPATH)/include/freertos_select \
            -I$(OPENMRNPATH)/src/freertos_drivers/common

ARCHOPTIMIZATION = -O3 -fno-strict-aliasing
#ARCHOPTIMIZATION = -O3 -fno-strict-aliasing -fno-strength-reduce -fomit-frame-pointer

BASEDEFS= -D__PIC32MX__ -D__XC__ -D__XC32 -D__XC -D__FreeRTOS__

# @TODO(balazs.racz) consider moving this to the drivers compile makefile. It
# is important to search this dir after the system include directories, because
# it contains a full libc system header set that is incompatible with vanilla
# GCC.
INCLUDES += -idirafter $(PIC32MXLIBPATH)/pic32mx/include
BASEDEFS += -DTARGET_PIC32MX -D__32MX795F512H__ \
	-D__PIC32_FEATURE_SET__=795

# This will create macros for the functions __builtin_mfc0 et al.
INCLUDES += -include freertos_drivers/pic32mx/builtins.h

ASFLAGS = -c -g -EL -MD -MP $(BASEDEFS) -D__LANGUAGE_ASSEMBLY__ -fdollars-in-identifiers -msoft-float -DTARGET_PIC32MX -march=mips32r2 $(INCLUDES)

#           -march=armv7-m -mthumb -mfloat-abi=soft

CORECFLAGS = -c -EL -g -msoft-float -march=mips32r2 $(ARCHOPTIMIZATION) \
	     -Wall -Werror -Wno-unknown-pragmas -MD -MP \
             -fno-stack-protector -DTARGET_PIC32MX \
             -D_POSIX_C_SOURCE=200112 $(BASEDEFS) -D__LANGUAGE_C__ \
             -ffunction-sections -fdata-sections

#             -march=armv7-m -mthumb -mfloat-abi=soft -mfix-cortex-m3-ldrd \
#             -DINTERRUPT_ATTRIBUTE=   


CFLAGS =  $(CORECFLAGS) -std=gnu99 -Wstrict-prototypes  $(CFLAGSENV)

CXXFLAGS = $(CORECFLAGS)  -std=c++0x  -D_ISOC99_SOURCE -fno-exceptions  \
           -fno-rtti -D__STDC_FORMAT_MACROS $(CXXFLAGSENV) -U__STRICT_ANSI__ # -D__STDC_VERSION__=199902L

LDFLAGS = -EL -g -T target.ld -fdata-sections -ffunction-sections  -Xlinker \
	-Map="$(@:%.elf=%.map)"  \
	-msoft-float -Wl,--defsym,__cs3_mips_float_type=2 \
	-Wl,--gc-sections -Wl,--undefined=ignore_fn \
          $(LDFLAGSEXTRA) $(LDFLAGSENV)

ifdef TRACE_MALLOC
LDFLAGS += \
          -Wl,--wrap=malloc   \

endif

SYSLIBRARIES +=  $(SYSLIBRARIESEXTRA)
SYSLIB_SUBDIRS += 

EXTENTION = .elf


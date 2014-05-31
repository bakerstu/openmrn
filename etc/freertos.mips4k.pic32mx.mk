# Get the $(FREERTOSPATH)
include $(OPENMRNPATH)/etc/freertos.mk

# Get the $(CFLAGSENV), $(CXXFLAGSENV), $(LDFLAGSENV)
include $(OPENMRNPATH)/etc/env.mk

include $(OPENMRNPATH)/etc/path.mk

# Get the $(TOOLPATH)
ifeq ($(TOOLPATH),)
TOOLPATH=$(MIPSGCCPATH)
endif

DEPS+= MIPSGCCPATH PIC32MXLIBPATH

# Get $(MBEDPATH)
#include $(OPENMRNPATH)/etc/mbed.mk

PREFIX = $(TOOLPATH)/bin/mips-sde-elf-

AS = $(PREFIX)gcc
CC = $(PREFIX)gcc
CXX = $(PREFIX)g++
AR = $(PREFIX)ar
LD = $(PREFIX)g++
SIZE = $(PREFIX)size
OBJCOPY = $(PREFIX)objcopy
OBJDUMP = $(PREFIX)objdump

STARTGROUP := -Wl,--start-group
ENDGROUP := -Wl,--end-group

INCLUDES += -I$(FREERTOSPATH)/Source/include \
            -I$(FREERTOSPATH)/Source/portable/MPLAB/PIC32MX \
            -I$(OPENMRNPATH)/include/freertos \
            -I$(OPENMRNPATH)/src/freertos_drivers/common

INCLUDES += -I$(PIC32MXLIBPATH)


ARCHOPTIMIZATION = -O3 -fno-strict-aliasing
#ARCHOPTIMIZATION = -O3 -fno-strict-aliasing -fno-strength-reduce -fomit-frame-pointer

BASEDEFS= -D__PIC32MX__ -D__XC__ -D__XC32  -D__32MX795F512H__ \
	-D__PIC32_FEATURE_SET__=795 -D__FreeRTOS__ 
ASFLAGS = -c -g -EL -MD -MP $(BASEDEFS) -D__LANGUAGE_ASSEMBLY__ -fdollars-in-identifiers -msoft-float -DTARGET_PIC32MX -march=mips32r2 $(INCLUDES)

#           -march=armv7-m -mthumb -mfloat-abi=soft

CORECFLAGS = -c -EL -g -msoft-float -march=mips32r2 $(ARCHOPTIMIZATION) -Wall -Werror -MD -MP \
             -fno-builtin -fno-stack-protector -DTARGET_PIC32MX \
             -D_POSIX_C_SOURCE=200112 $(BASEDEFS) -D__LANGUAGE_C__ \
             -ffunction-sections -fdata-sections

#             -march=armv7-m -mthumb -mfloat-abi=soft -mfix-cortex-m3-ldrd \
#             -DINTERRUPT_ATTRIBUTE=   


CFLAGS =  $(CORECFLAGS) -std=gnu99 -Wstrict-prototypes  $(CFLAGSENV)

CXXFLAGS = $(CORECFLAGS)  -std=c++0x  -D_ISOC99_SOURCE -fno-exceptions  \
           -fno-rtti -D__STDC_FORMAT_MACROS -D__STDC_VERSION__=199901L $(CXXFLAGSENV)

LDFLAGS = -EL -g -T target.ld -Xlinker \
	-Map="$(@:%.elf=%.map)"  \
	-msoft-float -Wl,--defsym,__cs3_mips_float_type=2 \
          $(LDFLAGSEXTRA) $(LDFLAGSENV)

SYSLIBRARIES +=  $(SYSLIBRARIESEXTRA)
SYSLIB_SUBDIRS += 

EXTENTION = .elf


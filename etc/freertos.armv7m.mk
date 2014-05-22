# Get the $(FREERTOSPATH)
include $(OPENMRNPATH)/etc/freertos.mk

# Get the $(CFLAGSENV), $(CXXFLAGSENV), $(LDFLAGSENV)
include $(OPENMRNPATH)/etc/env.mk

# Get the $(TOOLPATH)
ifneq ($(FREERTOSPATH),)
include $(OPENMRNPATH)/etc/armgcc-s.mk
endif

PREFIX = $(TOOLPATH)/bin/arm-none-eabi-

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
            -I$(FREERTOSPATH)/Source/portable/GCC/ARM_CM3 \
            -I$(OPENMRNPATH)/include/freertos \
            -I$(OPENMRNPATH)/src/freertos_drivers/common

ARCHOPTIMIZATION = -D__NEWLIB__
#ARCHOPTIMIZATION += -O3 -fno-strict-aliasing -fno-strength-reduce -fomit-frame-pointer
ARCHOPTIMIZATION += -Os -fno-strict-aliasing -fno-strength-reduce -fomit-frame-pointer -fdata-sections -ffunction-sections

ARCHFLAGS = -g -MD -MP -march=armv7-m -mthumb -mfloat-abi=soft

ASFLAGS = -c $(ARCHFLAGS)

CORECFLAGS = $(ARCHFLAGS) -Wall -Werror -fdata-sections -ffunction-sections \
	-fno-builtin -fno-stack-protector -mfix-cortex-m3-ldrd \
	-D__FreeRTOS__ -DGCC_ARMCM3  

CFLAGS = -c  $(ARCHOPTIMIZATION) $(CORECFLAGS) -std=gnu99 \
          -Wstrict-prototypes \
          $(CFLAGSENV) $(CFLAGSEXTRA)

CXXFLAGS = -c $(ARCHOPTIMIZATION) $(CORECFLAGS) -std=gnu++0x  \
           -D_ISOC99_SOURCE -D__USE_LIBSTDCPP__ -D__STDC_FORMAT_MACROS \
           -fno-exceptions -fno-rtti \
            $(CXXFLAGSENV) $(CXXFLAGSEXTRA) \
           -D__LINEAR_MAP__ #-D__STDC_VERSION__=199901

LDFLAGS = -g -fdata-sections -ffunction-sections -T target.ld \
          -march=armv7-m -mthumb -L$(TOOLPATH)/arm-none-eabi/lib/armv7-m \
          -Wl,-Map="$(@:%.elf=%.map)" -Wl,--gc-sections \
          $(LDFLAGSEXTRA) $(LDFLAGSENV) 

SYSLIBRARIES += $(SYSLIBRARIESEXTRA) -Wl,--wrap=__cxa_pure_virtual  -Wl,--wrap=__cxa_atexit -Wl,--defsym=__wrap___cxa_pure_virtual=abort -Wl,--defsym=__wrap___cxa_atexit=ignore_fn

EXTENTION = .elf


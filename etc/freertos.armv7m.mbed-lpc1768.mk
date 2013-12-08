# Get the $(FREERTOSPATH)
include $(OPENMRNPATH)/etc/freertos.mk

# Get the $(CFLAGSENV), $(CXXFLAGSENV), $(LDFLAGSENV)
include $(OPENMRNPATH)/etc/env.mk

# Get the $(TOOLPATH)
ifeq ($(TOOLPATH),)
include $(OPENMRNPATH)/etc/armgcc-s.mk
endif

# Get $(MBEDPATH)
include $(OPENMRNPATH)/etc/mbed.mk

PREFIX = $(TOOLPATH)/bin/arm-none-eabi-

AS = $(PREFIX)gcc
CC = $(PREFIX)gcc
CXX = $(PREFIX)g++
AR = $(PREFIX)ar
LD = $(PREFIX)g++
SIZE = $(PREFIX)size
OBJCOPY = $(PREFIX)objcopy
OBJDUMP = $(PREFIX)objdump

ifeq ($(TOOLPATH),/usr/local/lpcxpresso_5.1.2_2065/lpcxpresso/tools)
CLIBPATH=$(TOOLPATH)/lib/gcc/arm-none-eabi/4.6.2
CPPLIBPATH=$(TOOLPATH)/arm-none-eabi/include/c++/4.6.2
else
CLIBPATH=$(TOOLPATH)/lib/gcc/arm-none-eabi/4.7.3
CPPLIBPATH=$(TOOLPATH)/arm-none-eabi/include/c++/4.7.3
endif

STARTGROUP := -Wl,--start-group
ENDGROUP := -Wl,--end-group

INCLUDES += -I$(FREERTOSPATH)/Source/include \
            -I$(FREERTOSPATH)/Source/portable/GCC/ARM_CM3 \
            -I$(OPENMRNPATH)/include/freertos \
            -I$(OPENMRNPATH)/src/freertos_drivers/common

INCLUDES += -I$(TOOLPATH)/arm-none-eabi/include -I$(CLIBPATH)/include-fixed -I$(CLIBPATH)/include -I$(CPPLIBPATH)/backward -I$(CPPLIBPATH)/arm-none-eabi -I"$(MBEDSRCPATH)/cpp" -I"$(MBEDPATH)/mbed/vendor/NXP/capi" -I"$(MBEDPATH)/mbed/vendor/NXP/capi/LPC1768" -I"$(MBEDPATH)/mbed/vendor/NXP/cmsis/LPC1768" -I"$(MBEDPATH)/USBDevice/USBDevice" -I"$(MBEDPATH)/USBDevice/USBSerial" -I"$(MBEDSRCPATH)/capi" 


ARCHOPTIMIZATION = -Os -D__NEWLIB__ -fno-strict-aliasing
#ARCHOPTIMIZATION = -O3 -fno-strict-aliasing -fno-strength-reduce -fomit-frame-pointer

ASFLAGS = -c -g -MD -MP \
           -march=armv7-m -mthumb -mfloat-abi=soft

CORECFLAGS = -c -g $(ARCHOPTIMIZATION) -Wall -Werror -MD -MP -D__FreeRTOS__ \
             -fno-builtin -fno-stack-protector -DTARGET_LPC1768 \
             -march=armv7-m -mthumb -mfloat-abi=soft -mfix-cortex-m3-ldrd \
             -DINTERRUPT_ATTRIBUTE=   -D_POSIX_C_SOURCE=200112


CFLAGS =  $(CORECFLAGS) -std=gnu99 -Wstrict-prototypes  $(CFLAGSENV)
# On a cortex-m3 we can compile IRQ handlers as thumb too.
ARM_CFLAGS = $(CFLAGS)
CXXFLAGS = $(CORECFLAGS)  -std=c++0x  -D_ISOC99_SOURCE -fno-exceptions  \
           -fno-rtti -D__STDC_FORMAT_MACROS -D__STDC_VERSION__=199901 $(CXXFLAGSENV)

LDFLAGS = -g -nostdlib -nostartfiles -T target.ld -march=armv7-m -mthumb -L$(TOOLPATH)/arm-none-eabi/lib/thumb2 -Xlinker -Map="$(@:%.elf=%.map)" --specs=nano.specs \
          $(LDFLAGSEXTRA) $(LDFLAGSENV)

SYSLIBRARIES += -lmbed $(SYSLIBRARIESEXTRA)
SYSLIB_SUBDIRS += mbed

EXTENTION = .elf


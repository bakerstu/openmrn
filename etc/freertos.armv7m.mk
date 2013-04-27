# Get the $(FREERTOSPATH)
include $(OPENMRNPATH)/etc/freertos.mk

# Get the $(CFLAGSENV), $(CXXFLAGSENV), $(LDFLAGSENV)
include $(OPENMRNPATH)/etc/env.mk

# Get the $(TOOLPATH)
ifneq ($(FREERTOSPATH),)
include $(OPENMRNPATH)/etc/armgcc.mk
endif

PREFIX = $(TOOLPATH)/bin/arm-none-eabi-

AS = $(PREFIX)gcc
CC = $(PREFIX)gcc
CXX = $(PREFIX)g++
AR = $(PREFIX)ar
LD = $(PREFIX)g++

STARTGROUP := -Wl,--start-group
ENDGROUP := -Wl,--end-group

INCLUDES += -I$(FREERTOSPATH)/Source/include \
            -I$(FREERTOSPATH)/Source/portable/GCC/ARM_CM3 \
            -I$(OPENMRNPATH)/include/freertos \
            -I$(OPENMRNPATH)/src/freertos_drivers/common

ARCHOPTIMIZATION = -D__NEWLIB__
#ARCHOPTIMIZATION = -O3 -fno-strict-aliasing -fno-strength-reduce -fomit-frame-pointer

ASFLAGS = -c -g -MD -MP \
           -march=armv7-m -mthumb -mfloat-abi=soft

CFLAGS = -c -g $(ARCHOPTIMIZATION) -Wall -Werror -MD -MP -std=gnu99 -D__FreeRTOS__ \
         -fno-builtin \
         -march=armv7-m -mthumb -mfloat-abi=soft -Wstrict-prototypes \
         -fno-stack-protector -mfix-cortex-m3-ldrd -DGCC_ARMCM3 $(CFLAGSENV)
CXXFLAGS = -c -g $(ARCHOPTIMIZATION) -Wall -Werror -MD -MP -D__FreeRTOS__ \
           -fno-builtin \
           -march=armv7-m -mthumb -mfloat-abi=soft \
           -fno-stack-protector -mfix-cortex-m3-ldrd -fno-exceptions -DGCC_ARMCM3 \
           -D__STDC_FORMAT_MACROS $(CXXFLAGSENV)

LDFLAGS = -g -T target.ld -march=armv7-m -mthumb -L$(TOOLPATH)/arm-none-eabi/lib/thumb2 \
          $(LDFLAGSEXTRA) $(LDFLAGSENV)

SYSLIBRARIES = -lfreertos \
               -lfreertos_drivers  \
               $(SYSLIBRARIESEXTRA)

EXTENTION = .elf


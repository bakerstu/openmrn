# Get the $(NUTTXPATH)
include $(OPENMRNPATH)/etc/nuttx.mk

DEPS += NUTTXPATH

# Get the $(CFLAGSENV), $(CXXFLAGSENV), $(LDFLAGSENV)
include $(OPENMRNPATH)/etc/env.mk

# Get the $(TOOLPATH)
include $(OPENMRNPATH)/etc/armgcc.mk

PREFIX = $(TOOLPATH)/bin/arm-none-eabi-

CC = $(PREFIX)gcc
CXX = $(PREFIX)g++
AR = $(PREFIX)ar
LD = $(PREFIX)g++
SIZE = $(PREFIX)size
OBJCOPY = $(PREFIX)objcopy
OBJDUMP = $(PREFIX)objdump

STARTGROUP := -Wl,--start-group
ENDGROUP := -Wl,--end-group

INCLUDES += -I$(NUTTXPATH)/include \
            -I$(OPENMRNPATH)/include/nuttx

#ARCHOPTIMIZATION =
ARCHOPTIMIZATION = -O3 -fno-strict-aliasing -fno-strength-reduce -fomit-frame-pointer

CFLAGS = -c -g $(ARCHOPTIMIZATION) -Wall -MD -MP -std=gnu99 -D__nuttx__ \
         -fno-builtin -fpic -msingle-pic-base -mpic-register=r10 \
         -march=armv7-m -mthumb -mfloat-abi=soft -Wstrict-prototypes \
         -fno-stack-protector
CXXFLAGS = -c -g $(ARCHOPTIMIZATION) -Wall -MD -MP -D__nuttx__ \
           -fno-builtin -fpic -msingle-pic-base -mpic-register=r10 \
           -march=armv7-m -mthumb -mfloat-abi=soft -fpermissive \
           -fno-stack-protector -fno-exceptions -D__STDC_FORMAT_MACROS

LDFLAGS = -g
SYSLIBRARIES = 

EXTENTION =


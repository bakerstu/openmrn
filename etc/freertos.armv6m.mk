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
            -I$(FREERTOSPATH)/Source/portable/GCC/ARM_CM0 \
            -I$(OPENMRNPATH)/include/freertos \
            -I$(OPENMRNPATH)/src/freertos_drivers/common

ARCHOPTIMIZATION = 
ARCHOPTIMIZATION = -Os -D_REENT_SMALL
#ARCHOPTIMIZATION = -O3 -fno-strict-aliasing -fno-strength-reduce -fomit-frame-pointer

ASFLAGS = -c  -x assembler-with-cpp -D__NEWLIB__ -DDEBUG -D__CODE_RED -g -MD -MP \
           -mcpu=cortex-m0 -mthumb -mfloat-abi=soft

ifeq ($(TOOLPATH),/usr/local/lpcxpresso_5.1.2_2065/lpcxpresso/tools)
CLIBPATH=$(TOOLPATH)/lib/gcc/arm-none-eabi/4.6.2
CPPLIBPATH=$(TOOLPATH)/arm-none-eabi/include/c++/4.6.2
else
CLIBPATH=$(TOOLPATH)/lib/gcc/arm-none-eabi/4.7.3
CPPLIBPATH=$(TOOLPATH)/arm-none-eabi/include/c++/4.7.3
endif

DEPS += CMSIS_LPC11_PATH

INCLUDES += -I$(TOOLPATH)/arm-none-eabi/include -I$(CLIBPATH)/include-fixed -I$(CLIBPATH)/include -I$(CPPLIBPATH)/backward -I$(CPPLIBPATH)/arm-none-eabi -I$(CMSIS_LPC11_PATH)/inc


SHAREDCFLAGS = -DTARGET_LPC11Cxx -D__NEWLIB__ -DDEBUG \
        -D__USE_CMSIS=CMSISv2p00_LPC11xx -D__CODE_RED -D__FreeRTOS__  \
        -g3 -Wall -Werror -c -fmessage-length=0 -fno-builtin \
        -ffunction-sections -fdata-sections -fno-stack-protector \
        -mcpu=cortex-m0 -mthumb -mfloat-abi=soft \
        -MMD -MP -MF"$(@:%.o=%.d)" \
        $(CFLAGSENV)

#-MT"$(@:%.o=%.d)" 

CORECFLAGS = $(ARCHOPTIMIZATION) $(SHAREDCFLAGS) \
        -std=gnu99 -Wstrict-prototypes

CFLAGS = $(CORECFLAGS)

CXXFLAGS = $(ARCHOPTIMIZATION) $(SHAREDCFLAGS) \
        -fno-rtti -fno-exceptions -std=c++0x \
        -D__STDC_FORMAT_MACROS $(CXXFLAGSENV)

LDFLAGS = -g -nostdlib -L"$(CMSIS_LPC11_PATH)/Debug" -T target.ld \
        -Xlinker --gc-sections  -mcpu=cortex-m0 -mthumb \
        -Xlinker -Map="$(@:%.elf=%.map)"  \
          $(LDFLAGSEXTRA) $(LDFLAGSENV) \

#use this only if armgcc == arm gcc 4.7
#LDFLAGS += --specs=nano.specs


SYSLIBRARIES = \
        -lfreertos \
        -lfreertos_drivers  \
        -lCMSISv2p00_LPC11xx \
        $(SYSLIBRARIESEXTRA)

EXTENTION = .elf


# Get the $(FREERTOSPATH)
include $(OPENMRNPATH)/etc/freertos.mk

# Get the $(CFLAGSENV), $(CXXFLAGSENV), $(LDFLAGSENV)
include $(OPENMRNPATH)/etc/env.mk

# Get the $(TOOLPATH)
ifneq ($(FREERTOSPATH),)
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

STARTGROUP := -Wl,--start-group
ENDGROUP := -Wl,--end-group

INCLUDES += -I$(FREERTOSPATH)/Source/include \
            -I$(FREERTOSPATH)/Source/portable/GCC/ARM_CM0 \
            -I$(OPENMRNPATH)/include/freertos \
            -I$(OPENMRNPATH)/src/freertos_drivers/common

ARCHOPTIMIZATION = 
ARCHOPTIMIZATION += -D_REENT_SMALL
ARCHOPTIMIZATION += -Os -fno-strict-aliasing -fno-strength-reduce -fomit-frame-pointer

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

INCLUDES += -I$(TOOLPATH)/arm-none-eabi/include -I$(CLIBPATH)/include-fixed -I$(CLIBPATH)/include -I$(CPPLIBPATH)/backward -I$(CPPLIBPATH)/arm-none-eabi -I$(CMSIS_LPC11_PATH)/inc -I"$(MBEDSRCPATH)/cpp" -I"$(MBEDPATH)/mbed/vendor/NXP/capi" -I"$(MBEDPATH)/mbed/vendor/NXP/capi/LPC11U24" -I"$(MBEDSRCPATH)/capi"  #-I"$(MBEDPATH)/mbed/vendor/NXP/cmsis/LPC11U24"


SHAREDCFLAGS = -DTARGET_LPC11Cxx -D__NEWLIB__ -DDEBUG \
        -D__USE_CMSIS=CMSISv2p00_LPC11xx -D__CODE_RED -D__FreeRTOS__  \
        -g3 -Wall -Werror -c -fmessage-length=0 -fno-builtin \
        -ffunction-sections -fdata-sections -fno-stack-protector \
        -mcpu=cortex-m0 -mthumb -mfloat-abi=soft \
        -MMD -MP -MF"$(@:%.o=%.d)" -D_GLIBCXX_DEQUE_BUF_SIZE=32  \
	-D__LINEAR_MAP__ \
        $(CFLAGSENV)

#	-D__USE_LIBSTDCPP__ \


#-MT"$(@:%.o=%.d)" 

CORECFLAGS = $(ARCHOPTIMIZATION) $(SHAREDCFLAGS) \
        -std=gnu99 -Wstrict-prototypes

CFLAGS = $(CORECFLAGS)
ARM_CFLAGS = $(CFLAGS)

CXXFLAGS = $(ARCHOPTIMIZATION) $(SHAREDCFLAGS) \
        -fno-rtti -fno-exceptions -std=c++0x \
        -D__STDC_FORMAT_MACROS -D__STDC_VERSION__=199901 $(CXXFLAGSENV)

LDFLAGS = -g -nostdlib -L"$(CMSIS_LPC11_PATH)/Debug" -T target.ld \
        -Xlinker --gc-sections  -mcpu=cortex-m0 -mthumb --specs=nano.specs \
        -Xlinker -Map="$(@:%.elf=%.map)" -Wl,--wrap=__cxa_pure_virtual   \
	-Wl,--wrap=__cxa_atexit  -Wl,--wrap=exit \
          $(LDFLAGSEXTRA) $(LDFLAGSENV) \

#use this only if armgcc == arm gcc 4.7
#LDFLAGS += --specs=nano.specs

SYSLIB_SUBDIRS += mbed
SYSLIBRARIES += -lCMSISv2p00_LPC11xx -lmbed  $(SYSLIBRARIESEXTRA)

EXTENTION = .elf


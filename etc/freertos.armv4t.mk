include $(OPENMRNPATH)/etc/config.mk

# Get the $(FREERTOSPATH)
include $(OPENMRNPATH)/etc/freertos.mk

# Get the $(CFLAGSENV), $(CXXFLAGSENV), $(LDFLAGSENV)
include $(OPENMRNPATH)/etc/env.mk

# Get the $(TOOLPATH)
ifneq ($(FREERTOSPATH),)
include $(OPENMRNPATH)/etc/armgcc.mk
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
            -I$(FREERTOSPATH)/Source/portable/GCC/ARM7_LPC23xx \
            -I$(OPENMRNPATH)/include/freertos \
            -I$(OPENMRNPATH)/src/freertos_drivers/common

ARCHOPTIMIZATION = -Os
#ARCHOPTIMIZATION = -O3 -fno-strict-aliasing -fno-strength-reduce -fomit-frame-pointer

ASFLAGS = -c  -x assembler-with-cpp -D__NEWLIB__ -DDEBUG -D__CODE_RED -g -MD -MP \
           -mcpu=arm7tdmi -mfloat-abi=soft  -mthumb-interwork

ifeq ($(TOOLPATH),/usr/local/lpcxpresso_5.1.2_2065/lpcxpresso/tools)
CLIBPATH=$(TOOLPATH)/lib/gcc/arm-none-eabi/4.6.2
CPPLIBPATH=$(TOOLPATH)/arm-none-eabi/include/c++/4.6.2
else
CLIBPATH=$(TOOLPATH)/lib/gcc/arm-none-eabi/4.7.3
CPPLIBPATH=$(TOOLPATH)/arm-none-eabi/include/c++/4.7.3
endif


# -I"/home/bracz/lpc-workspace/libmbed_2387"
INCLUDES += -I$(TOOLPATH)/arm-none-eabi/include -I$(CLIBPATH)/include-fixed -I$(CLIBPATH)/include -I$(CPPLIBPATH)/backward -I$(CPPLIBPATH)/arm-none-eabi -I"$(MBEDSRCPATH)/cpp" -I"$(MBEDPATH)/mbed/vendor/NXP/capi" -I"$(MBEDPATH)/mbed/vendor/NXP/capi/LPC2368" -I"$(MBEDPATH)/mbed/vendor/NXP/cmsis/LPC2368" -I"$(MBEDPATH)/USBDevice/USBDevice" -I"$(MBEDPATH)/USBDevice/USBSerial" -I"$(MBEDSRCPATH)/capi" 


#-MT"$(@:%.o=%.d)" 
CORECFLAGS = $(ARCHOPTIMIZATION) -DTARGET_LPC2368 -D__NEWLIB__ -DDEBUG \
            -D__CODE_RED  -g3 -Wall -c -fmessage-length=0 -fno-builtin \
            -ffunction-sections -fdata-sections -mthumb-interwork \
            -mcpu=arm7tdmi -MMD -MP -MF"$(@:%.o=%.d)" \
            -Werror -D__FreeRTOS__ -mfloat-abi=soft \
           -fno-stack-protector -DTHUMB_INTERWORK \
            $(CFLAGSENV) -D__CR2_C___4_6_2_BITS_SHARED_PTR_H__

ARM_CFLAGS = $(CORECFLAGS)

CFLAGS = $(CORECFLAGS) -mthumb -Wstrict-prototypes -std=gnu99 

# -MT"$(@:%.o=%.d)"
CXXFLAGS = $(ARCHOPTIMIZATION) $(CORECFLAGS) -fno-rtti -fno-exceptions \
            -std=c++0x \
           -mthumb  \
            -D__STDC_FORMAT_MACROS \
           -D__STDC_VERSION__=199901  \
           $(CXXFLAGSENV)

LDFLAGS = -g -nostdlib -L"/home/bracz/lpc-workspace/libmbed_2387/Debug" \
          -T target.ld -mthumb -Xlinker --gc-sections -mcpu=arm7tdmi \
          -Xlinker -Map="$(@:%.elf=%.map)" -fmessage-length=0 -fno-builtin \
          -ffunction-sections -fdata-sections -fno-rtti -fno-exceptions \
          -Wl,--wrap=__cxa_pure_virtual   \
          -Wl,--wrap=__cxa_atexit  -Wl,--wrap=exit \
          -Wl,--wrap=_ZSt20__throw_length_errorPKc \
          -Wl,--defsym=__wrap__ZSt20__throw_length_errorPKc=abort \
          -Wl,--undefined=ignore_fn \
          -Wl,--defsym=__wrap___cxa_atexit=ignore_fn \
          -Wl,--defsym=__wrap___cxa_pure_virtual=abort \
          -Wl,--defsym=__wrap_exit=abort \
          $(LDFLAGSEXTRA) $(LDFLAGSENV)


SYSLIBRARIES += -llibmbed_2387 $(SYSLIBRARIESEXTRA)

EXTENTION = .elf


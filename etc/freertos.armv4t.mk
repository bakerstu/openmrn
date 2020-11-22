include $(OPENMRNPATH)/etc/config.mk

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
SIZE = $(PREFIX)size
OBJCOPY = $(PREFIX)objcopy
OBJDUMP = $(PREFIX)objdump

AROPTS=D

STARTGROUP := -Wl,--start-group
ENDGROUP := -Wl,--end-group

INCLUDES += -I$(FREERTOSPATH)/Source/include \
            -I$(FREERTOSPATH)/Source/portable/GCC/ARM7_LPC23xx \
            -I$(OPENMRNPATH)/include/freertos \
            -idirafter $(OPENMRNPATH)/include/freertos_select \
            -I$(OPENMRNPATH)/src/freertos_drivers/common

ARCHOPTIMIZATION = -Os -funwind-tables #-D_REENT_SMALL
#ARCHOPTIMIZATION = -O3 -fno-strict-aliasing -fno-strength-reduce -fomit-frame-pointer

ASFLAGS = -c  -x assembler-with-cpp -D__NEWLIB__ -DDEBUG -D__CODE_RED -g -MD -MP \
           -mcpu=arm7tdmi -mfloat-abi=soft  -mthumb-interwork -DTHUMB_INTERWORK

ifeq ($(TOOLPATH),/usr/local/lpcxpresso_5.1.2_2065/lpcxpresso/tools)
CLIBPATH=$(TOOLPATH)/lib/gcc/arm-none-eabi/4.6.2
CPPLIBPATH=$(TOOLPATH)/arm-none-eabi/include/c++/4.6.2
else
CLIBPATH=$(TOOLPATH)/lib/gcc/arm-none-eabi/4.7.3
CPPLIBPATH=$(TOOLPATH)/arm-none-eabi/include/c++/4.7.3
endif


# -I"/home/bracz/lpc-workspace/libmbed_2387"
INCLUDES += -I$(TOOLPATH)/arm-none-eabi/include -I$(CLIBPATH)/include-fixed -I$(CLIBPATH)/include -I$(CPPLIBPATH)/backward -I$(CPPLIBPATH)/arm-none-eabi 

#-MT"$(@:%.o=%.d)" 
CORECFLAGS = $(ARCHOPTIMIZATION) -DTARGET_LPC2368 -D__NEWLIB__ -DDEBUG \
            -D__CODE_RED  -g3 -Wall -c -fmessage-length=0 -fno-builtin \
            -ffunction-sections -fdata-sections -mthumb-interwork \
            -mcpu=arm7tdmi -MMD -MP -MF"$(@:%.o=%.d)" \
            -Werror -Wno-unknown-pragmas -D__FreeRTOS__ -mfloat-abi=soft \
           -fno-stack-protector -DTHUMB_INTERWORK \
            $(CFLAGSENV) -D__CR2_C___4_6_2_BITS_SHARED_PTR_H__

ARM_CFLAGS = $(CORECFLAGS)

CFLAGS = $(CORECFLAGS) -mthumb -Wstrict-prototypes -std=c99

EXCEPT_FLAG := # -fno-rtti # -fno-exceptions

# -MT"$(@:%.o=%.d)"
CXXFLAGS = $(ARCHOPTIMIZATION) $(CORECFLAGS) $(EXCEPT_FLAG) \
            -std=c++14 \
           -mthumb  \
            -D__STDC_FORMAT_MACROS \
           $(CXXFLAGSENV)

LDFLAGS = -g -nostdlib -L"/home/bracz/lpc-workspace/libmbed_2387/Debug" \
          -T target.ld -mthumb -mthumb-interwork -Xlinker --gc-sections -mcpu=arm7tdmi \
          -Xlinker -Map="$(@:%.elf=%.map)" -fmessage-length=0 -fno-builtin \
          -ffunction-sections -fdata-sections $(EXCEPT_FLAG) \
          -Wl,--wrap=__cxa_pure_virtual   \
          -Wl,--wrap=__cxa_atexit  -Wl,--wrap=exit \
          -Wl,--wrap=_ZSt20__throw_length_errorPKc \
          -Wl,--defsym=__wrap__ZSt20__throw_length_errorPKc=abort \
          -Wl,--undefined=ignore_fn \
          -Wl,--defsym=__wrap___cxa_atexit=ignore_fn \
          -Wl,--defsym=__wrap___cxa_pure_virtual=abort \
          -Wl,--defsym=__wrap_exit=abort \
          -Wl,--wrap=__cxa_throw   \
          -Wl,--defsym=__wrap___cxa_throw=abort \
          -Wl,--wrap=_ZSt9terminatev   \
          -Wl,--defsym=__wrap__ZSt9terminatev=abort \
          -Wl,--wrap=__cxa_end_cleanup   \
          -Wl,--defsym=__wrap___cxa_end_cleanup=abort \
          -Wl,--wrap=_ZSt10unexpectedv   \
          -Wl,--defsym=__wrap__ZSt10unexpectedv=abort \
          -Wl,--wrap=__cxa_begin_catch   \
          -Wl,--defsym=__wrap___cxa_begin_catch=abort \
          -Wl,--wrap=__cxa_call_unexpected   \
          -Wl,--defsym=__wrap___cxa_call_unexpected=abort \
          $(LDFLAGSEXTRA) $(LDFLAGSENV)

ifdef TRACE_MALLOC
LDFLAGS += -Wl,--wrap=malloc   
else 
LDFLAGS += -Wl,--defsym=__real_malloc=malloc 
endif


ifdef asdsajdflaskgd
          -Wl,--wrap=__gxx_personality_v0   \
          -Wl,--defsym=__wrap___gxx_personality_v0=abort \
          -Wl,--wrap=_ZSt19__throw_logic_errorPKc   \
          -Wl,--defsym=__wrap__ZSt19__throw_logic_errorPKc=abort \
          -Wl,--wrap=_ZSt9terminatev   \
          -Wl,--defsym=__wrap__ZSt9terminatev=abort \
          -Wl,--wrap=__cxa_rethrow   \
          -Wl,--defsym=__wrap___cxa_rethrow=abort \
          -Wl,--wrap=__cxa_end_cleanup   \
          -Wl,--defsym=__wrap___cxa_end_cleanup=abort \
          -Wl,--wrap=__cxa_begin_catch   \
          -Wl,--defsym=__wrap___cxa_begin_catch=abort \
          -Wl,--wrap=__cxa_end_catch   \
          -Wl,--defsym=__wrap___cxa_end_catch=abort \
          -Wl,--wrap=_ZSt20__throw_out_of_rangePKc   \
          -Wl,--defsym=__wrap__ZSt20__throw_out_of_rangePKc=abort \
          -Wl,--wrap=__aeabi_unwind_cpp_pr0   \
          -Wl,--defsym=__wrap___aeabi_unwind_cpp_pr0=abort \
          -Wl,--wrap=__aeabi_unwind_cpp_pr1   \
          -Wl,--defsym=__wrap___aeabi_unwind_cpp_pr1=abort \

          -Wl,--wrap=   \
          -Wl,--defsym=__wrap_=abort \

endif

SYSLIBRARIES += -llibmbed_2387 $(SYSLIBRARIESEXTRA)
#SYSLIB_SUBDIRS += mbed

EXTENTION = .elf


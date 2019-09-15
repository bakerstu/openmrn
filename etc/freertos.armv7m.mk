# Get the $(FREERTOSPATH)
include $(OPENMRNPATH)/etc/freertos.mk

# Get the $(CFLAGSENV), $(CXXFLAGSENV), $(LDFLAGSENV)
include $(OPENMRNPATH)/etc/env.mk

# Get the $(TOOLPATH)
ifeq ($(TOOLPATH),)
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
GDB = $(PREFIX)gdb

AROPTS=D
OBJDUMPOPTS=-C

STARTGROUP := -Wl,--start-group
ENDGROUP := -Wl,--end-group

INCLUDES += -I$(FREERTOSPATH)/Source/include \
            -I$(FREERTOSPATH)/Source/portable/GCC/ARM_CM3 \
            -I$(OPENMRNPATH)/include/freertos \
            -idirafter $(OPENMRNPATH)/include/freertos_select \
            -I$(OPENMRNPATH)/src/freertos_drivers/common

#ARCHOPTIMIZATION = -D__NEWLIB__
#ARCHOPTIMIZATION += -O3 -fno-strict-aliasing -fno-strength-reduce -fomit-frame-pointer
ARCHOPTIMIZATION += -Os -fno-strict-aliasing -fno-strength-reduce -fomit-frame-pointer -fdata-sections -ffunction-sections

ARCHFLAGS = -g -MD -MP -mcpu=cortex-m3 -mthumb -mfloat-abi=soft

ifdef DEBUG_MEMORY_USE
#warning: -funwind-tables adds 10k code size. Needed for malloc debugging.
ARCHFLAGS += -funwind-tables
endif

ASFLAGS = -c $(ARCHFLAGS)

CORECFLAGS = $(ARCHFLAGS) -Wall -Werror -Wno-unknown-pragmas \
             -fdata-sections -ffunction-sections \
             -fno-builtin -fno-stack-protector -mfix-cortex-m3-ldrd \
             -D__FreeRTOS__ -DGCC_ARMCM3 -specs=nano.specs

CFLAGS += -c $(ARCHOPTIMIZATION) $(CORECFLAGS) -std=c99 \
          -Wstrict-prototypes -D_REENT_SMALL \
          $(CFLAGSENV) $(CFLAGSEXTRA) \


CXXFLAGS += -c $(ARCHOPTIMIZATION) $(CORECFLAGS) -std=c++14  \
            -D_ISOC99_SOURCE -D__STDC_FORMAT_MACROS \
            -fno-exceptions -fno-rtti \
            $(CXXFLAGSENV) $(CXXFLAGSEXTRA) \

#            -Wsuggest-override \
	   # -D__LINEAR_MAP__
            #-D__USE_LIBSTDCPP__ #-D__STDC_VERSION__=199901


LDFLAGS += -g -fdata-sections -ffunction-sections -T target.ld \
           $(ARCHFLAGS) -Os \
           -Wl,-Map="$(@:%.elf=%.map)" -Wl,--gc-sections \
           -Wl,--undefined=ignore_fn $(LDFLAGSEXTRA) $(LDFLAGSENV) \
           --specs=nano.specs -Wl,--wrap=_malloc_r -Wl,--wrap=_free_r

SYSLIB_SUBDIRS +=
SYSLIBRARIES +=

# We disable linking against certain components from libc that we don't need
# and pull in a lot of code dependencies (typically 50-100 kbytes), like
# exception unwinding, or global destructor handling. On any exception
# triggering call we just crash. This way we simulate as if newlib had been
# compiled with -fno-exception. Most of these can be removed once we remove
# usage of std::string.
SYSLIBRARIES += $(SYSLIBRARIESEXTRA) \
          -Wl,--wrap=__cxa_pure_virtual \
          -Wl,--defsym=__wrap___cxa_pure_virtual=abort \
          -Wl,--wrap=__cxa_atexit \
          -Wl,--defsym=__wrap___cxa_atexit=ignore_fn \
          -Wl,--wrap=__aeabi_atexit \
          -Wl,--defsym=__wrap___aeabi_atexit=ignore_fn \
          -Wl,--wrap=_ZSt20__throw_length_errorPKc \
          -Wl,--defsym=__wrap__ZSt20__throw_length_errorPKc=abort \
          -Wl,--wrap=__cxa_throw   \
          -Wl,--defsym=__wrap___cxa_throw=abort \
          -Wl,--wrap=_ZSt19__throw_logic_errorPKc   \
          -Wl,--defsym=__wrap__ZSt19__throw_logic_errorPKc=abort \
          -Wl,--wrap=_ZSt17__throw_bad_allocv   \
          -Wl,--defsym=__wrap__ZSt17__throw_bad_allocv=abort \
          -Wl,--wrap=_ZSt20__throw_out_of_rangePKc   \
          -Wl,--defsym=__wrap__ZSt20__throw_out_of_rangePKc=abort \
          -Wl,--wrap=_ZSt24__throw_out_of_range_fmtPKcz   \
          -Wl,--defsym=__wrap__ZSt24__throw_out_of_range_fmtPKcz=abort \
          -Wl,--wrap=_ZSt25__throw_bad_function_callv   \
          -Wl,--defsym=__wrap__ZSt25__throw_bad_function_callv=abort \
          -Wl,--wrap=__cxa_allocate_exception   \
          -Wl,--defsym=__wrap___cxa_allocate_exception=abort \
          -Wl,--wrap=__gxx_personality_v0   \
          -Wl,--defsym=__wrap___gxx_personality_v0=abort \
          -Wl,--wrap=__cxa_begin_catch   \
          -Wl,--defsym=__wrap___cxa_begin_catch=abort \
          -Wl,--wrap=__cxa_end_cleanup   \
          -Wl,--defsym=__wrap___cxa_end_cleanup=abort \
          -Wl,--wrap=__cxa_end_catch   \
          -Wl,--defsym=__wrap___cxa_end_catch=abort \
          -Wl,--wrap=__cxa_free_exception   \
          -Wl,--defsym=__wrap___cxa_free_exception=abort \
          -Wl,--wrap=__cxa_call_unexpected   \
          -Wl,--defsym=__wrap___cxa_call_unexpected=abort \
          -Wl,--wrap=__aeabi_unwind_cpp_pr0   \
          -Wl,--defsym=__wrap___aeabi_unwind_cpp_pr0=abort \
          -Wl,--wrap=__aeabi_unwind_cpp_pr1   \
          -Wl,--defsym=__wrap___aeabi_unwind_cpp_pr1=abort \


ifeq (1,0)
          -Wl,--wrap=   \
          -Wl,--defsym=__wrap_=abort \

endif





EXTENTION = .elf


# Get the $(CFLAGSENV), $(CXXFLAGSENV), $(LDFLAGSENV)
include $(OPENMRNPATH)/etc/env.mk

# Get the path to all targets
include $(OPENMRNPATH)/etc/path.mk

DEPS += FREERTOSPATH ESPTOOLPATH
TOOLPATH := $(ESPTOOLPATH)
PREFIX = $(XTENSAGCCPATH)/bin/xtensa-lx106-elf-

AS = $(PREFIX)gcc
CC = $(PREFIX)gcc
CXX = $(PREFIX)g++
AR = $(PREFIX)ar
LD = $(PREFIX)g++
SIZE = $(PREFIX)size
OBJCOPY = $(PREFIX)objcopy
OBJDUMP = $(PREFIX)objdump
GDB = $(PREFIX)gdb
ESPTOOL = PATH=$(XTENSAGCCPATH)/bin:$$PATH $(ESPTOOLPATH)/esptool.py

AROPTS=D
OBJDUMPOPTS=-C

STARTGROUP := -Wl,--start-group
ENDGROUP := -Wl,--end-group

INCLUDES += 

#ARCHOPTIMIZATION = -D__NEWLIB__
#ARCHOPTIMIZATION += -O3 -fno-strict-aliasing -fno-strength-reduce -fomit-frame-pointer
ARCHOPTIMIZATION += -Os -fno-strict-aliasing -fno-strength-reduce -fomit-frame-pointer -fdata-sections -ffunction-sections -mlongcalls

ARCHFLAGS = -g -MD -MP -D__have_long32=0

ifdef DEBUG_MEMORY_USE
#warning: -funwind-tables adds 10k code size. Needed for malloc debugging.
ARCHFLAGS += -funwind-tables
endif

ASFLAGS = -c $(ARCHFLAGS)

CORECFLAGS = $(ARCHFLAGS) -Wall -Werror -Wno-unknown-pragmas \
             -fdata-sections -ffunction-sections \
             -fno-builtin -fno-stack-protector \
             -D_REENT_SMALL -DESP_NONOS -DICACHE_FLASH

CFLAGS += -c $(ARCHOPTIMIZATION) $(CORECFLAGS) -std=gnu99 \
          -Wstrict-prototypes -D_REENT_SMALL \
          $(CFLAGSENV) $(CFLAGSEXTRA) \


CXXFLAGS += -c $(ARCHOPTIMIZATION) $(CORECFLAGS) -std=gnu++0x  \
            -D_ISOC99_SOURCE -D__STDC_FORMAT_MACROS \
            -fno-exceptions -fno-rtti \
            $(CXXFLAGSENV) $(CXXFLAGSEXTRA) \

#            -Wsuggest-override \


	   # -D__LINEAR_MAP__
            #-D__USE_LIBSTDCPP__ #-D__STDC_VERSION__=199901

LDFLAGS += -g -fdata-sections -ffunction-sections -T $(LDSCRIPT) -L $(ESPOPENSDKPATH)/sdk/ld \
           -Wl,-Map="$(@:%.elf=%.map)" -Wl,--gc-sections \
           -nostdlib \
           -Wl,--undefined=ignore_fn $(LDFLAGSEXTRA) $(LDFLAGSENV)  \

SYSLIB_SUBDIRS += 
SYSLIBRARIES += -Wl,--start-group -lmain -lnet80211 -lwpa -llwip -lpp -lphy -Wl,--end-group -lgcc -lstdc++ -lc

SYSLIBRARIES += $(SYSLIBRARIESEXTRA) \
          -Wl,--defsym=snprintf=ets_snprintf \
          -Wl,--defsym=printf=ets_printf \
          -Wl,--defsym=__assert_func=abort \
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


#	-Wl,--undefined=os_snprintf \


ifeq (1,0)

# We disable linking against certain components from libc that we don't need
# and pull in a lot of code dependencies (typically 50-100 kbytes), like
# exception unwinding, or global destructor handling. On any exception
# triggering call we just crash. This way we simulate as if newlib had been
# compiled with -fno-exception. Most of these can be removed once we remove
# usage of std::string.
SYSLIBRARIES += $(SYSLIBRARIESEXTRA) \

endif

ifeq (1,0)
          -Wl,--wrap=   \
          -Wl,--defsym=__wrap_=abort \

endif





EXTENTION = .elf


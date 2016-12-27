# Get the $(FREERTOSPATH)
include $(OPENMRNPATH)/etc/freertos.mk

# Get the $(CFLAGSENV), $(CXXFLAGSENV), $(LDFLAGSENV)
include $(OPENMRNPATH)/etc/env.mk

# Get the $(TOOLPATH)
ifeq ($(TOOLPATH),)
TOOLPATH=$(MIPSGCCPATH)
endif

DEPS+= MIPSGCCPATH PIC32MXLIBPATH PIC32MXLEGACYPLIBPATH

PREFIX = $(TOOLPATH)/bin/mips-sde-elf-

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
            -I$(FREERTOSPATH)/Source/portable/MPLAB/PIC32MX \
            -I$(OPENMRNPATH)/include/freertos \
            -idirafter $(OPENMRNPATH)/include/freertos_select \
            -I$(OPENMRNPATH)/src/freertos_drivers/common

ARCH = -mips16
ARCHOPTIMIZATION = $(ARCH) -Os -fno-strict-aliasing
#ARCHOPTIMIZATION = -O3 -fno-strict-aliasing -fno-strength-reduce -fomit-frame-pointer

BASEDEFS= -D__PIC32MX__ -D__XC__ -D__XC32 -D__XC -D__FreeRTOS__

# @TODO(balazs.racz) consider moving this to the drivers compile makefile. It
# is important to search this dir after the system include directories, because
# it contains a full libc system header set that is incompatible with vanilla
# GCC.
INCLUDES += -idirafter $(PIC32MXLIBPATH)/pic32mx/include \
	-idirafter $(PIC32MXLEGACYPLIBPATH)/pic32mx/include \
	-D_DISABLE_OPENADC10_CONFIGPORT_WARNING 

BASEDEFS += -DTARGET_PIC32MX  -D__32MX795F512H__ \
	-D__PIC32_FEATURE_SET__=795

# This will create macros for the functions __builtin_mfc0 et al.
INCLUDES += -include freertos_drivers/pic32mx/builtins.h

ASFLAGS = -c -g -EL -MD -MP $(BASEDEFS) -D__LANGUAGE_ASSEMBLY__ -fdollars-in-identifiers -msoft-float -DTARGET_PIC32MX $(ARCH) $(INCLUDES)

#           -march=armv7-m -mthumb -mfloat-abi=soft

CORECFLAGS = -c -EL -g -msoft-float -march=mips32r2 $(ARCHOPTIMIZATION) \
	     -Wall -Werror -Wno-unknown-pragmas -MD -MP \
             -fno-stack-protector -DTARGET_PIC32MX \
             -D_POSIX_C_SOURCE=200112 $(BASEDEFS) -D__LANGUAGE_C__ \
             -ffunction-sections -fdata-sections

#             -march=armv7-m -mthumb -mfloat-abi=soft -mfix-cortex-m3-ldrd \
#             -DINTERRUPT_ATTRIBUTE=   


CFLAGS =  $(CORECFLAGS) -std=gnu99 -Wstrict-prototypes  $(CFLAGSENV)

CXXFLAGS = $(CORECFLAGS)  -std=c++0x  -D_ISOC99_SOURCE -fno-exceptions  \
           -fno-rtti -D__STDC_FORMAT_MACROS $(CXXFLAGSENV) -U__STRICT_ANSI__ # -D__STDC_VERSION__=199902L

LDFLAGS = -EL $(ARCH) -g -T target.ld -fdata-sections -ffunction-sections  -Xlinker \
	-Map="$(@:%.elf=%.map)"  \
	-msoft-float -Wl,--defsym,__cs3_mips_float_type=2 \
	-Wl,--gc-sections -Wl,--undefined=ignore_fn \
          $(LDFLAGSEXTRA) $(LDFLAGSENV)

ifdef TRACE_MALLOC
LDFLAGS += \
          -Wl,--wrap=malloc   \

endif

SYSLIBRARIES +=  $(SYSLIBRARIESEXTRA)
SYSLIB_SUBDIRS += 

EXTENTION = .elf


# We disable linking against certain components from libc that we don't need
# and pull in a lot of code dependencies (typically 50-100 kbytes), like
# exception unwinding, or global destructor handling. On any exception
# triggering call we just crash. This way we simulate as if newlib had been
# compiled with -fno-exception. Most of these can be removed once we remove
# usage of std::string.

#	-L$(MIPSGCCPATH)/lib/gcc/mips-sde-e
#          -Wl,--wrap=   \
#          -Wl,--defsym=__wrap_=abort \


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
          -Wl,--wrap=_Unwind_Resume   \
          -Wl,--defsym=__wrap__Unwind_Resume=abort \
          -Wl,--wrap=__gnu_compact_pr2   \
          -Wl,--defsym=__wrap___gnu_compact_pr2=abort \



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

AROPTS=D

STARTGROUP := -Wl,--start-group
ENDGROUP := -Wl,--end-group

INCLUDES += -I$(FREERTOSPATH)/Source/include \
            -I$(FREERTOSPATH)/Source/portable/GCC/ARM_CM0 \
            -I$(OPENMRNPATH)/include/freertos \
            -I$(OPENMRNPATH)/src/freertos_drivers/common

ARCHOPTIMIZATION += -Os -fno-strict-aliasing -fno-strength-reduce \
                    -fomit-frame-pointer -fdata-sections -ffunction-sections

# Uncomment following line for debugging without optimization
#ARCHOPTIMIZATION += -O0

ARCHFLAGS = -g -MD -MP -march=armv6-m -mthumb -mfloat-abi=soft

ASFLAGS = -c $(ARCHFLAGS)

CORECFLAGS = $(ARCHOPTIMIZATION) $(ARCHFLAGS) -Wall -Werror \
             -Wno-unknown-pragmas -fdata-sections -ffunction-sections \
             -fno-builtin -fno-stack-protector -D__FreeRTOS__ -DGCC_ARMCM0

CFLAGS += -c $(CORECFLAGS) -std=gnu99 -Wstrict-prototypes  \
          $(CFLAGSENV) $(CFLAGSEXTRA)

CXXFLAGS += -c $(CORECFLAGS) -std=gnu++0x -D_ISOC99_SOURCE \
            -D__USE_LIBSTDCPP__ -D__STDC_FORMAT_MACROS -D__LINEAR_MAP__ \
            -fno-exceptions -fno-rtti $(CXXFLAGSENV) $(CXXFLAGSEXTRA)

LDFLAGS += -g -fdata-sections -ffunction-sections -T target.ld \
           -march=armv6-m -mthumb -L$(TOOLPATH)/arm-none-eabi/lib/armv6-m \
           -Wl,-Map="$(@:%.elf=%.map)" -Wl,--gc-sections \
           -Wl,--undefined=ignore_fn $(LDFLAGSEXTRA) $(LDFLAGSENV) 

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

EXTENTION = .elf


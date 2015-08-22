ifndef TOOLPATH
TOOLPATH := $(shell \
sh -c "if [ -d /usr/include/linux ]; then echo /usr/bin; \
      else echo; fi" \
)
endif

# Get the $(CFLAGSENV), $(CXXFLAGSENV), $(LDFLAGSENV)
include $(OPENMRNPATH)/etc/env.mk
include $(OPENMRNPATH)/etc/path.mk

DEPS += CLANGPPPATH

CC = clang
CXX = clang++
AR = llvm-ar
LD = clang++
OBJDUMP = llvm-objdump

OBJDUMPOPTS=-symbolize 

HOST_TARGET := 1

STARTGROUP := -Wl,--start-group
ENDGROUP := -Wl,--end-group

ARCHSELECT = -m64
ARCHOPTIMIZATION = -g -O0 $(ARCHSELECT)

CSHAREDFLAGS = -c $(ARCHOPTIMIZATION) -Wall -Werror -MP -fno-stack-protector -D_GNU_SOURCE

CFLAGS = $(CSHAREDFLAGS) -std=gnu99

CXXFLAGS = $(CSHAREDFLAGS) -std=c++0x -D__STDC_FORMAT_MACROS \
           -D__STDC_LIMIT_MACROS #-D__LINEAR_MAP__

LDFLAGS = -g $(ARCHSELECT) -Wl,-Map="$(@:%=%.map)"
SYSLIB_SUBDIRS += console
SYSLIBRARIES = -lrt -lpthread -lconsole

EXTENTION =


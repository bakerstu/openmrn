include $(OPENMRNPATH)/etc/path.mk

ifneq ($(strip $(EMSDKPATH)),)
HAVE_EMSCRIPTEN = 1
TOOLPATH:=$(EMSDKPATH)
else
TOOLPATH=
endif


# Get the $(CFLAGSENV), $(CXXFLAGSENV), $(LDFLAGSENV)
include $(OPENMRNPATH)/etc/env.mk

CC = $(EMSDKPATH)/emcc
CXX = $(EMSDKPATH)/em++
AR = $(EMLLVMPATH)/llvm-ar
LD = $(EMSDKPATH)/em++
OBJDUMP = echo $(EMLLVMPATH)/llvm-objdump

EMU := nodejs

DEPS += NODEJSPATH

STARTGROUP := -Wl,--start-group
ENDGROUP := -Wl,--end-group

ARCHOPTIMIZATION = -g -O0 -m32

CSHAREDFLAGS = -c $(ARCHOPTIMIZATION) -Wall -Werror -MP -m32 -fno-stack-protector -D_GNU_SOURCE -Wno-warn-absolute-paths --em-config $(EMSDKPATH)/../../.emscripten

CFLAGS = $(CSHAREDFLAGS) -std=gnu99

CXXFLAGS = $(CSHAREDFLAGS) -std=c++0x -D__STDC_FORMAT_MACROS \
           -D__STDC_LIMIT_MACROS #-D__LINEAR_MAP__

LDFLAGS = -g -m32 -Wl,-Map="$(@:%=%.map)" --em-config $(EMSDKPATH)/../../.emscripten
SYSLIB_SUBDIRS += console
SYSLIBRARIES = -lconsole

EXTENTION ?= .js


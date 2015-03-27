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
AR = $(EMSDKPATH)/llvm-ar
LD = $(EMSDKPATH)/em++
OBJDUMP = $(EMSDKPATH)/llvm-objdump

EMU := nodejs

STARTGROUP := -Wl,--start-group
ENDGROUP := -Wl,--end-group

ARCHOPTIMIZATION = -g -O0 -m32

CSHAREDFLAGS = -c $(ARCHOPTIMIZATION) -Wall -Werror -MP -m32 -fno-stack-protector -D_GNU_SOURCE -Wno-warn-absolute-paths --em-config $(EMSDKPATH)/../../.emscripten

CFLAGS = $(CSHAREDFLAGS) -std=gnu99

CXXFLAGS = $(CSHAREDFLAGS) -std=c++0x -D__STDC_FORMAT_MACROS \
           -D__STDC_LIMIT_MACROS #-D__LINEAR_MAP__

LDFLAGS = -g -m32 -pg -Wl,-Map="$(@:%=%.map)"
SYSLIB_SUBDIRS += console
SYSLIBRARIES = -lconsole

EXTENTION = .js


ifndef TOOLPATH
TOOLPATHCOMMAND := $(shell \
sh -c "which arm-linux-gnueabihf-gcc" \
)
TOOLPATH := $(dir $(TOOLPATHCOMMAND))
endif

$(info mach toolpath '$(TOOLPATH)')

# Get the $(CFLAGSENV), $(CXXFLAGSENV), $(LDFLAGSENV)
include $(OPENMRNPATH)/etc/env.mk

CC = arm-linux-gnueabihf-gcc
CXX = arm-linux-gnueabihf-g++
AR = arm-linux-gnueabihf-ar
LD = arm-linux-gnueabihf-g++
OBJDUMP = arm-linux-gnueabihf-objdump

AROPTS=D

HOST_TARGET := 1

STARTGROUP := -Wl,--start-group
ENDGROUP := -Wl,--end-group

ARCHOPTIMIZATION = -g -O0 -march=armv7-a

CSHAREDFLAGS = -c $(ARCHOPTIMIZATION) -Wall -Werror -Wno-unknown-pragmas \
               -MD -MP -fno-stack-protector -D_GNU_SOURCE

CFLAGS = $(CSHAREDFLAGS) -std=gnu99

CXXFLAGS = $(CSHAREDFLAGS) -std=c++0x -D__STDC_FORMAT_MACROS \
           -D__STDC_LIMIT_MACROS #-D__LINEAR_MAP__

LDFLAGS = $(ARCHOPTIMIZATION) -pg -Wl,-Map="$(@:%=%.map)"
SYSLIB_SUBDIRS += console
SYSLIBRARIES = -lrt -lpthread -lconsole

EXTENTION =


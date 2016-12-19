# Get the toolchain paths for openmrn
include $(OPENMRNPATH)/etc/path.mk


ifndef TOOLPATH
#TOOLPATHCOMMAND := $(shell \
#sh -c "which arm-linux-gnueabihf-gcc" \
#)
TOOLPATH := $(ARMLINUXGCCPATH)
endif

$(info armv7alinux toolpath '$(TOOLPATH)')

# Get the $(CFLAGSENV), $(CXXFLAGSENV), $(LDFLAGSENV)
include $(OPENMRNPATH)/etc/env.mk

CC = $(TOOLPATH)/arm-linux-gnueabihf-gcc
CXX = $(TOOLPATH)/arm-linux-gnueabihf-g++
AR = $(TOOLPATH)/arm-linux-gnueabihf-ar
LD = $(TOOLPATH)/arm-linux-gnueabihf-g++
OBJDUMP = $(TOOLPATH)/cd .arm-linux-gnueabihf-objdump

AROPTS=D

HOST_TARGET := 1

STARTGROUP := -Wl,--start-group
ENDGROUP := -Wl,--end-group

ARCHOPTIMIZATION = -g3 -O0 -march=armv7-a

CSHAREDFLAGS = -c $(ARCHOPTIMIZATION) -Wall -Werror -Wno-unknown-pragmas \
               -MD -MP -fno-stack-protector -D_GNU_SOURCE

CFLAGS = $(CSHAREDFLAGS) -std=gnu99

CXXFLAGS = $(CSHAREDFLAGS) -std=c++0x -D__STDC_FORMAT_MACROS \
           -D__STDC_LIMIT_MACROS -D__USE_LIBSTDCPP__

LDFLAGS = $(ARCHOPTIMIZATION) -Wl,-Map="$(@:%=%.map)"
SYSLIB_SUBDIRS +=
SYSLIBRARIES = -lrt -lpthread

EXTENTION =


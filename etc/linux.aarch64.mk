# Get the toolchain paths for openmrn
include $(OPENMRNPATH)/etc/path.mk


ifndef TOOLPATH
#TOOLPATHCOMMAND := $(shell \
#sh -c "which aarch64-linux-gnu-gcc" \
#)
TOOLPATH := $(AARCH64LINUXGCCPATH)
endif

$(info armv7alinux toolpath '$(TOOLPATH)')

# Get the $(CFLAGSENV), $(CXXFLAGSENV), $(LDFLAGSENV)
include $(OPENMRNPATH)/etc/env.mk

CC = $(TOOLPATH)/aarch64-linux-gnu-gcc
CXX = $(TOOLPATH)/aarch64-linux-gnu-g++
AR = $(TOOLPATH)/aarch64-linux-gnu-ar
LD = $(TOOLPATH)/aarch64-linux-gnu-g++
OBJDUMP = $(TOOLPATH)/aarch64-linux-gnu-objdump

AROPTS=D

HOST_TARGET := 1

STARTGROUP := -Wl,--start-group
ENDGROUP := -Wl,--end-group

ARCHOPTIMIZATION = -g3 -O0 -march=armv8-a

CSHAREDFLAGS = -c $(ARCHOPTIMIZATION) -Wall -Werror -Wno-unknown-pragmas \
               -MD -MP -fno-stack-protector -D_GNU_SOURCE

CFLAGS = $(CSHAREDFLAGS) -std=gnu99

CXXFLAGS = $(CSHAREDFLAGS) -std=c++0x -D__STDC_FORMAT_MACROS \
           -D__STDC_LIMIT_MACROS -D__USE_LIBSTDCPP__

LDFLAGS = $(ARCHOPTIMIZATION) -Wl,-Map="$(@:%=%.map)"
SYSLIB_SUBDIRS +=
SYSLIBRARIES = -lrt -lpthread

EXTENTION =


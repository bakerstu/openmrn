TOOLPATH ?= $(shell \
sh -c "if [ -d /usr/include/linux ]; then echo /usr/bin; \
      else echo; fi" \
)

# Get the $(CFLAGSENV), $(CXXFLAGSENV), $(LDFLAGSENV)
include $(OPENMRNPATH)/etc/env.mk

CC = gcc
CXX = g++
AR = ar
LD = g++

HOST_TARGET := 1

STARTGROUP := -Wl,--start-group
ENDGROUP := -Wl,--end-group

ARCHOPTIMIZATION = -g -O0 -m32

CSHAREDFLAGS = -c $(ARCHOPTIMIZATION) -Wall -Werror -MD -MP -m32 -fno-stack-protector -D_GNU_SOURCE

CFLAGS = $(CSHAREDFLAGS) -std=gnu99

CXXFLAGS = $(CSHAREDFLAGS) -std=c++0x -D__STDC_FORMAT_MACROS #-D__LINEAR_MAP__

LDFLAGS = -g -m32 -pg
SYSLIBRARIES = -lrt -lpthread

EXTENTION =


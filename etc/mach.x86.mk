ifndef TOOLPATH
TOOLPATH := $(shell \
sh -c "if [ -d /usr/include/mach ]; then echo /usr/bin; \
      else echo; fi" \
)
endif

# Get the $(CFLAGSENV), $(CXXFLAGSENV), $(LDFLAGSENV)
include $(OPENMRNPATH)/etc/env.mk

CC = gcc
CXX = g++
AR = ar
LD = g++

STARTGROUP :=
ENDGROUP :=

INCLUDES += -I$(OPENMRNPATH)/include/mach

CFLAGS = -c -g -O0 -Wall -Werror -MD -MP -std=gnu99 -m32 -fno-stack-protector \
         -D_GNU_SOURCE
CXXFLAGS = -c -g -O0 -Wall -Werror -MD -MP -std=c++0x -m32 -fno-stack-protector \
           -D_GNU_SOURCE -D__STDC_FORMAT_MACROS

LDFLAGS = -g -m32
SYSLIBRARIES = -lpthread

EXTENTION =


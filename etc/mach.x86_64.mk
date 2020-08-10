# Get the toolchain path
include $(OPENMRNPATH)/etc/path.mk

ifeq ($(shell uname -sm),Darwin x86_64)
TOOLPATH := $(HOSTCLANGPPPATH)
endif

$(info mach toolpath '$(TOOLPATH)')

# Get the $(CFLAGSENV), $(CXXFLAGSENV), $(LDFLAGSENV)
include $(OPENMRNPATH)/etc/env.mk

CC = clang
CXX = clang++
AR = ar
LD = clang++

STARTGROUP :=
ENDGROUP :=

INCLUDES += -I$(OPENMRNPATH)/include/mach

CFLAGS = -c -g -O0 -Wall -Werror -MD -MP -std=c99 -fno-stack-protector \
         -D_GNU_SOURCE
CXXFLAGS = -c -g -O0 -Wall -Werror -MD -MP -std=c++14 -fno-stack-protector \
           -D_GNU_SOURCE

LDFLAGS = -g
SYSLIBRARIES = -lpthread

EXTENTION =


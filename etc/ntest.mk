TOOLPATH ?= /usr/bin

# Get the $(CFLAGSENV), $(CXXFLAGSENV), $(LDFLAGSENV)
include $(OPENMRNPATH)/etc/env.mk

# Define this variable if you want to use a specific (suffixed) GCC version
# instead of the system default.
#GCCVERSION=-8

CC = $(shell $(OPENMRNPATH)/bin/find_distcc.sh gcc$(GCCVERSION))
CXX = $(shell $(OPENMRNPATH)/bin/find_distcc.sh g++$(GCCVERSION))
AR = ar
LD = g++$(GCCVERSION)
OBJDUMP = objdump

AROPTS=D

HOST_TARGET := 1

STARTGROUP := -Wl,--start-group
ENDGROUP := -Wl,--end-group

TESTOPTIMIZATION=-O0

ARCHOPTIMIZATION = -g $(TESTOPTIMIZATION) -fdata-sections -ffunction-sections -fPIC -O0

CSHAREDFLAGS = -c -frandom-seed=$(shell echo $(abspath $<) | md5sum  | sed 's/\(.*\) .*/\1/') \
    $(ARCHOPTIMIZATION) $(INCLUDES) -Wall -Werror -Wno-unknown-pragmas -MD -MP \
    -fno-stack-protector -D_GNU_SOURCE -DGTEST

CFLAGS = $(CSHAREDFLAGS) -std=gnu99 $(CFLAGSENV) $(CFLAGSEXTRA)

CXXFLAGS = $(CSHAREDFLAGS) -std=c++14 -D__STDC_FORMAT_MACROS \
           -D__STDC_LIMIT_MACROS $(CXXFLAGSENV) $(CXXFLAGSEXTRA) \

LDFLAGS = $(ARCHOPTIMIZATION) -pg -Wl,-Map="$(@:%=%.map)" -Wl,--undefined=ignore_fn

SYSLIB_SUBDIRS +=
SYSLIBRARIES = -lrt -lpthread -lavahi-client -lavahi-common $(SYSLIBRARIESEXTRA)

EXTENTION =


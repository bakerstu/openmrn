ifndef TOOLPATH
TOOLPATH := $(shell \
sh -c "if [ -d /usr/include/linux ]; then echo /usr/bin; \
      else echo; fi" \
)
endif

$(info mach toolpath '$(TOOLPATH)')

# Get the $(CFLAGSENV), $(CXXFLAGSENV), $(LDFLAGSENV)
include $(OPENMRNPATH)/etc/env.mk

# Define this variable if you want to use a specific (suffixed) GCC version
# instead of the system default.
#GCCVERSION=-8

CC = gcc$(GCCVERSION)
CXX = g++$(GCCVERSION)
AR = ar
LD = g++$(GCCVERSION)
OBJDUMP = objdump

AROPTS=D

HOST_TARGET := 1

STARTGROUP := -Wl,--start-group
ENDGROUP := -Wl,--end-group

ARCHOPTIMIZATION = -g -fdata-sections -ffunction-sections

CSHAREDFLAGS = -c $(ARCHOPTIMIZATION) -Wall -Werror -Wno-unknown-pragmas -MD -MP -fno-stack-protector -D_GNU_SOURCE

CFLAGS = $(CSHAREDFLAGS) -std=gnu99

CXXFLAGS = $(CSHAREDFLAGS) -std=c++0x -D__STDC_FORMAT_MACROS \
           -D__STDC_LIMIT_MACROS #-D__LINEAR_MAP__

LDFLAGS = $(ARCHOPTIMIZATION) -pg -Wl,-Map="$(@:%=%.map)" -Wl,--undefined=ignore_fn

SYSLIB_SUBDIRS +=
SYSLIBRARIES = -lrt -lpthread $(SYSLIBRARIESEXTRA)

EXTENTION =


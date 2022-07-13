# Special target for building with the host GCC in coverage collecting mode.

TOOLPATH := /usr/bin
# Get the $(CFLAGSENV), $(CXXFLAGSENV), $(LDFLAGSENV)
include $(OPENMRNPATH)/etc/env.mk

# Define this variable if you want to use a specific (suffixed) GCC version
# instead of the system default.
# GCCVERSION=-8

CC = $(shell $(OPENMRNPATH)/bin/find_distcc.sh gcc$(GCCVERSION))
CXX = $(shell $(OPENMRNPATH)/bin/find_distcc.sh g++$(GCCVERSION))
AR = ar
LD = g++$(GCCVERSION)

HOST_TARGET := 1

STARTGROUP := -Wl,--start-group
ENDGROUP := -Wl,--end-group

TESTOPTIMIZATION=-O0

ifdef SKIP_COVERAGE
ARCHOPTIMIZATION = -g $(TESTOPTIMIZATION)
else
ARCHOPTIMIZATION = -g $(TESTOPTIMIZATION) -fprofile-arcs -ftest-coverage
endif

CSHAREDFLAGS = -c -frandom-seed=$(shell echo $(abspath $<) | md5sum  | sed 's/\(.*\) .*/\1/') $(ARCHOPTIMIZATION) $(INCLUDES) -Wall -Werror -Wno-unknown-pragmas -MD -MP -fno-stack-protector -D_GNU_SOURCE -DGTEST

CFLAGS = $(CSHAREDFLAGS) -std=gnu99 $(CFLAGSEXTRA)

CXXFLAGS = $(CSHAREDFLAGS) -std=c++14 -D__STDC_FORMAT_MACROS \
           -D__STDC_LIMIT_MACROS $(CXXFLAGSEXTRA) #-D__LINEAR_MAP__


LDFLAGS = $(ARCHOPTIMIZATION) -Wl,-Map="$(@:%=%.map)"
SYSLIB_SUBDIRS +=
SYSLIBRARIES = -lrt -lpthread -lavahi-client -lavahi-common $(SYSLIBRARIESEXTRA)

ifdef RUN_GPERF
CXXFLAGS += -DWITHGPERFTOOLS
LDFLAGS += -DWITHGPERFTOOLS
SYSLIBRARIES += -lprofiler
TESTOPTIMIZATION = -O3
SKIP_COVERAGE = 1
endif


ifndef SKIP_COVERAGE
LDFLAGS += -pg
SYSLIBRARIES += -lgcov
endif

ifdef RUN_TSAN
ARCHOPTIMIZATION += -fsanitize=thread
LDFLAGS += -fsanitize=thread
endif


EXTENTION =


# Special target for building with the host GCC in coverage collecting mode.

TOOLPATH := /usr/bin
# Get the $(CFLAGSENV), $(CXXFLAGSENV), $(LDFLAGSENV)
include $(OPENMRNPATH)/etc/env.mk

CC = gcc
CXX = g++
AR = ar
LD = g++

HOST_TARGET := 1

STARTGROUP := -Wl,--start-group
ENDGROUP := -Wl,--end-group

ARCHOPTIMIZATION = -g -O0 -m32 -fprofile-arcs -ftest-coverage

CSHAREDFLAGS = -c -frandom-seed=$(shell echo $(abspath $<) | md5sum  | sed 's/\(.*\) .*/\1/') $(ARCHOPTIMIZATION) $(INCLUDES) -Wall -Werror -Wno-unknown-pragmas -MD -MP -fno-stack-protector -D_GNU_SOURCE

CFLAGS = $(CSHAREDFLAGS) -std=gnu99

CXXFLAGS = $(CSHAREDFLAGS) -std=c++0x -D__STDC_FORMAT_MACROS \
           -D__STDC_LIMIT_MACROS #-D__LINEAR_MAP__

LDFLAGS = $(ARCHOPTIMIZATION) -pg -Wl,-Map="$(@:%=%.map)"
SYSLIB_SUBDIRS += console
SYSLIBRARIES = -lrt -lpthread -lconsole -lgcov

EXTENTION =


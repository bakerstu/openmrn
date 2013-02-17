TOOLPATH ?= $(shell \
sh -c "if [ -d /usr/i686-w64-mingw32 ]; then echo /usr/bin; \
      else echo; fi" \
)

PREFIX ?= $(shell \
sh -c "if [ -d /usr/i686-w64-mingw32 ]; then echo i686-w64-mingw32-; \
      else echo; fi" \
)

# Get the $(CFLAGSENV), $(CXXFLAGSENV), $(LDFLAGSENV)
include $(OPENMRNPATH)/etc/env.mk

CC  = $(TOOLPATH)/$(PREFIX)gcc
CXX = $(TOOLPATH)/$(PREFIX)g++
AR  = $(TOOLPATH)/$(PREFIX)ar
LD  = $(TOOLPATH)/$(PREFIX)g++

INCLUDES += -I/usr/mingw-pthreads/mingw32/i686-w64-mingw32/include \
            -I$(OPENMRNPATH)/include/windows

CFLAGS = -c -g -O0 -Wall -Werror -MD -MP -std=gnu99 -m32 -fno-stack-protector \
         -D_GNU_SOURCE
CXXFLAGS = -c -g -O0 -Wall -Werror -MD -MP -m32 -fno-stack-protector \
           -D_GNU_SOURCE -D__STDC_FORMAT_MACROS

LDFLAGS = -g -m32 -L/usr/mingw-pthreads/mingw32/bin
SYSLIBRARIES = -lpthreadGC2-w32 -lwsock32

EXTENTION = .exe


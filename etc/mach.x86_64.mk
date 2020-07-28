ifndef TOOLPATH
TOOLPATH := $(shell \
sh -c "if [ -d /Applications/Xcode.app/Contents/Developer ]; then echo /usr/bin; \
      else echo; fi" \
)
endif

$(info mach toolpath '$(TOOLPATH)')

# Get the $(CFLAGSENV), $(CXXFLAGSENV), $(LDFLAGSENV)
include $(OPENMRNPATH)/etc/env.mk

CC = gcc
CXX = g++
AR = ar
LD = g++

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


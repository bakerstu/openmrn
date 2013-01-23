TOOLPATH ?= $(shell \
sh -c "if [ -d /usr/include/mach ]; then echo /usr/bin; \
      else echo; fi" \
)

# Get the $(CFLAGSENV), $(CXXFLAGSENV), $(LDFLAGSENV)
include $(OPENMRNPATH)/etc/env.mk

CC = gcc
CXX = g++
AR = ar
LD = g++

INCLUDES += -I$(OPENMRNPATH)/include/mach

CFLAGS = -c -g -O0 -Wall -Werror -MD -MP -std=gnu99 -fno-stack-protector \
         -D_GNU_SOURCE
CXXFLAGS = -c -g -O0 -Wall -Werror -MD -MP -fno-stack-protector \
           -D_GNU_SOURCE

LDFLAGS = -g
SYSLIBRARIES = -lpthread

EXTENTION =






findmatch=$(firstword $(foreach dir,$(1),$(if $(wildcard $(dir)),$(wildcard $(dir)))))

MINGWSEARCHPATH := \
	/usr/bin/i686-w64-mingw32-g++ \
	/usr/bin/x86_64-w64-mingw32-g++ \


TRYPATH:=$(call findmatch,$(MINGWSEARCHPATH))
ifneq ($(TRYPATH),)
TOOLPATH:=$(subst g++,,$(TRYPATH))
endif
$(info $(TRYPATH) toolpath $(TOOLPATH))
# Get the $(CFLAGSENV), $(CXXFLAGSENV), $(LDFLAGSENV)
include $(OPENMRNPATH)/etc/env.mk

CC  = $(TOOLPATH)gcc
CXX = $(TOOLPATH)g++
AR  = $(TOOLPATH)ar
LD  = $(TOOLPATH)g++

STARTGROUP := -Wl,--start-group
ENDGROUP := -Wl,--end-group

INCLUDES += -I/usr/mingw-pthreads/mingw32/i686-w64-mingw32/include \
            -I$(OPENMRNPATH)/include/windows

CFLAGS = -c -g -O0 -Wall -Werror -MD -MP -std=gnu99 -m32 -fno-stack-protector \
         -D_GNU_SOURCE -Wno-unknown-pragmas 
CXXFLAGS = -c -g -O0 -Wall -Werror -MD -MP -m32 -fno-stack-protector \
           -D_GNU_SOURCE -D__STDC_FORMAT_MACROS -std=c++0x -Wno-unknown-pragmas 

LDFLAGS = -g -m32 -L/usr/mingw-pthreads/mingw32/bin
SYSLIBRARIES = -lpthreadGC2-w32 -lwsock32

EXTENTION = .exe


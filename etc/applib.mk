ifeq ($(TARGET),)
# if the target is so far undefined
TARGET := $(notdir $(realpath $(CURDIR)/..))
endif
BASENAME := $(notdir $(CURDIR))
SRCDIR = $(abspath ../../../$(BASENAME))
VPATH = $(SRCDIR)

INCLUDES += -I./ -I../ -I../include 
INCLUDES += -I$(OPENMRNPATH)/include
INCLUDES += -I$(OPENMRNPATH)/src
include $(OPENMRNPATH)/etc/$(TARGET).mk

exist := $(wildcard $(SRCDIR)/sources)
ifneq ($(strip $(exist)),)
include $(VPATH)/sources
else
exist := $(wildcard sources)
ifneq ($(strip $(exist)),)
include sources
else
FULLPATHCSRCS = $(wildcard $(VPATH)/*.c)
FULLPATHCXXSRCS = $(wildcard $(VPATH)/*.cxx)
FULLPATHCPPSRCS = $(wildcard $(VPATH)/*.cpp)
CSRCS = $(notdir $(FULLPATHCSRCS))
CXXSRCS = $(notdir $(FULLPATHCXXSRCS))
CPPSRCS = $(notdir $(FULLPATHCPPSRCS))
endif
endif

ifdef APP_PATH
INCLUDES += -I$(APP_PATH)
endif

OBJS = $(CXXSRCS:.cxx=.o) $(CPPSRCS:.cpp=.o) $(CSRCS:.c=.o)
LIBNAME = lib$(BASENAME).a

ifdef BOARD
INCLUDES += -D$(BOARD)
endif
CFLAGS += $(INCLUDES)
CXXFLAGS += $(INCLUDES)

DEPS += TOOLPATH
MISSING_DEPS:=$(call find_missing_deps,$(DEPS))

ifneq ($(MISSING_DEPS),)
all docs clean veryclean tests mksubdirs:
	@echo "** Ignoring target $(TARGET), because the following libraries are not installed: $(MISSING_DEPS). This is not an error, so please do not report as a bug. If you care about target $(TARGET), make sure the quoted libraries are installed. For most libraries you can check $(OPENMRNPATH)/etc/path.mk to see where we looked for these dependencies."

else
.PHONY: all
all: $(LIBNAME)

-include $(OBJS:.o=.d)

.SUFFIXES:
.SUFFIXES: .o .c .cxx .cpp

.cpp.o:
	$(CXX) $(CXXFLAGS) $< -o $@
	$(CXX) -MM $(CXXFLAGS) $< > $*.d

.cxx.o:
	$(CXX) $(CXXFLAGS) $< -o $@
	$(CXX) -MM $(CXXFLAGS) $< > $*.d

.c.o:
	$(CC) $(CFLAGS) $< -o $@
	$(CC) -MM $(CFLAGS) $< > $*.d

$(LIBNAME): $(OBJS)
	$(AR) cr $(LIBNAME) $(OBJS)
	mkdir -p ../lib
ifeq ($(OS),Windows_NT)
	cp -f $(LIBNAME) ../lib/
else
	(cd ../lib ; ln -sf ../$(BASENAME)/$(LIBNAME) . )	
endif
	touch ../lib/timestamp


.PHONY: clean
clean:
	rm -rf $(wildcard *.o *.d *.a *.so *.dll) timestamp

.PHONY: veryclean
veryclean: clean

.PHONY: tests
tests:

.PHONY: mksubdirs
mksubdirs:


endif

ifeq ($(TARGET),)
# if the target is so far undefined
TARGET := $(shell basename `cd ../; pwd`)
endif
BASENAME = $(shell basename `pwd`)
SRCDIR = $(abspath ../../../$(BASENAME))
VPATH = $(SRCDIR)

INCLUDES += -I./ -I../ -I../include 
ifdef APP_PATH
INCLUDES += -I$(APP_PATH)
endif
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
	@echo "******************************************************************"
	@echo "*"
	@echo "*   Unable to build for $(TARGET), missing dependencies: $(MISSING_DEPS)"
	@echo "*"
	@echo "******************************************************************"
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
	ln -sf -t ../lib ../$(BASENAME)/$(LIBNAME)
	touch ../lib/timestamp


.PHONY: clean
clean:
	rm -rf *.o *.d *.a *.so *.dll

.PHONY: veryclean
veryclean: clean

.PHONY: tests
tests:

.PHONY: mksubdirs
mksubdirs:


endif

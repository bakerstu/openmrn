ifeq ($(TARGET),)
# if the target is so far undefined
TARGET := $(shell basename `cd ../; pwd`)
endif
BASENAME = $(shell basename `pwd`)
SRCDIR = ../../../$(BASENAME)
VPATH = $(SRCDIR)

INCLUDES += -I./ -I../ -I../include
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

CFLAGS += $(INCLUDES)

ifeq ($(TOOLPATH),)
all docs clean veryclean:
	@echo "******************************************************************"
	@echo "*"
	@echo "*   Unable to build for $(TARGET), no toolchain available"
	@echo "*"
	@echo "******************************************************************"
else
.PHONY: all
all: $(LIBNAME)

export: $(LIBNAME)

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
	cp $(LIBNAME) ../lib/$(LIBNAME)

.PHONY: clean
clean:
	rm -rf *.o *.d *.a *.so *.dll

.PHONY: veryclean
veryclean: clean

endif

TARGET := $(shell basename `cd ../; pwd`)
BASENAME = $(shell basename `pwd`)
SRCDIR = $(LEVEL)src/$(BASENAME)
VPATH = $(SRCDIR)

INCLUDES += -I./ -I$(LEVEL)src/ -I $(LEVEL)include
include $(LEVEL)etc/$(TARGET).mk

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
CSRCS = $(notdir $(FULLPATHCSRCS))
CXXSRCS = $(notdir $(FULLPATHCXXSRCS))
endif
endif

OBJS = $(CXXSRCS:.cxx=.o) $(CSRCS:.c=.o)
LIBNAME = lib$(BASENAME).a

CFLAGS += $(INCLUDES)

.PHONY: all
all: $(LIBNAME)

-include $(OBJS:.o=.d)

.SUFFIXES:
.SUFFIXES: .o .c .cxx

.cxx.o:
	$(CXX) $(CXXFLAGS) $< -o $@
	$(CXX) -MM $(CXXFLAGS) $< > $*.d

.c.o:
	$(CC) $(CFLAGS) $< -o $@
	$(CC) -MM $(CFLAGS) $< > $*.d

$(LIBNAME): $(OBJS)
	$(AR) cr $(LIBNAME) $(OBJS)
	cp $(LIBNAME) ../lib/$(LIBNAME)

.PHONY: clean
clean:
	rm -rf *.o *.d *.a *.so *.dll

.PHONY: veryclean
veryclean: clean


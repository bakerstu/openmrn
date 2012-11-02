TARGET := $(shell basename `cd ../; pwd`)
BASENAME = $(shell basename `pwd`)
VPATH = $(LEVEL)src/$(BASENAME)
INCLUDES = -I./ -I$(LEVEL)src/ -I $(LEVEL)include
FULLPATHCSRCS = $(wildcard $(VPATH)/*.c)
FULLPATHCXXSRCS = $(wildcard $(VPATH)/*.cxx)
CSRCS = $(notdir $(FULLPATHCSRCS))
CXXSRCS = $(notdir $(FULLPATHCXXSRCS))
OBJS = $(CXXSRCS:.cxx=.o) $(CSRCS:.c=.o)
LIBNAME = lib$(BASENAME).a

include $(LEVEL)etc/$(TARGET).mk

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


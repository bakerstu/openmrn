TARGET := $(shell basename `pwd`)
LEVEL = $(BASEPATH)
include $(BASEPATH)etc/$(TARGET).mk
VPATH = ../../
INCLUDES += -I$(BASEPATH)src/ -I $(BASEPATH)include
FULLPATHCSRCS = $(wildcard $(VPATH)/*.c)
FULLPATHCXXSRCS = $(wildcard $(VPATH)/*.cxx)
CSRCS = $(notdir $(FULLPATHCSRCS))
CXXSRCS = $(notdir $(FULLPATHCXXSRCS))
OBJS = $(CXXSRCS:.cxx=.o) $(CSRCS:.c=.o)
LIBNAME = lib$(BASENAME).a
LIBDIR = $(BASEPATH)targets/$(TARGET)/lib
FULLPATHLIBS = $(wildcard $(LIBDIR)/*.a)
LIBS = -lif -los -lcore
LDFLAGS += -L./ -L$(LIBDIR)
CFLAGS += $(INCLUDES)
CXXFLAGS += $(INCLUDES)

EXECUTABLE = $(shell basename `cd ../../; pwd`)

all: $(EXECUTABLE)$(EXTENTION)

$(EXECUTABLE)$(EXTENTION): $(OBJS) $(FULLPATHLIBS)
	$(LD) -o $@ $(OBJS) $(LDFLAGS) $(LIBS) $(SYSLIBRARIES)

-include $(OBJS:.o=.d)

.SUFFIXES:
.SUFFIXES: .o .c .cxx

.cxx.o:
	$(CXX) $(CXXFLAGS) $< -o $@
	$(CXX) -MM $(CXXFLAGS) $< > $*.d

.c.o:
	$(CC) $(CFLAGS) $< -o $@
	$(CC) -MM $(CFLAGS) $< > $*.d

clean:
	rm -rf *.o *.d *.a *.so $(EXECUTABLE)$(EXTENTION)

verclean: clean


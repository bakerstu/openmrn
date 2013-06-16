ifeq ($(TARGET),)
# if the target is so far undefined
TARGET := $(shell basename `pwd`)
endif
include ../../subdirs
include $(OPENMRNPATH)/etc/$(TARGET).mk

VPATH = ../../

FULLPATHASMSRCS = $(wildcard $(VPATH)/*.S)
FULLPATHCSRCS = $(wildcard $(VPATH)/*.c)
FULLPATHCXXSRCS = $(wildcard $(VPATH)/*.cxx)
FULLPATHCPPSRCS = $(wildcard $(VPATH)/*.cpp)
ASMSRCS = $(notdir $(FULLPATHASMSRCS)) $(wildcard *.S)
CSRCS = $(notdir $(FULLPATHCSRCS)) $(wildcard *.c)
CXXSRCS = $(notdir $(FULLPATHCXXSRCS)) $(wildcard *.cxx)
CPPSRCS = $(notdir $(FULLPATHCPPSRCS)) $(wildcard *.cpp)
OBJS = $(CXXSRCS:.cxx=.o) $(CPPSRCS:.cpp=.o) $(CSRCS:.c=.o) $(ASMSRCS:.S=.o)

LIBDIR = $(OPENMRNPATH)/targets/$(TARGET)/lib
FULLPATHLIBS = $(wildcard $(LIBDIR)/*.a) $(wildcard lib/*.a)
LIBDIRS := $(SUBDIRS)
LIBS = $(STARTGROUP) \
       $(foreach lib,$(LIBDIRS),-l$(lib)) \
       $(ENDGROUP) \
       -lif -lcore -los 

SUBDIRS += lib
INCLUDES += -I$(OPENMRNPATH)/src/ -I$(OPENMRNPATH)/include
CFLAGS += $(INCLUDES)
CXXFLAGS += $(INCLUDES)
LDFLAGS += -Llib -L$(LIBDIR)

EXECUTABLE = $(shell basename `cd ../../; pwd`)

ifeq ($(TOOLPATH),)
all docs clean veryclean:
	@echo "******************************************************************"
	@echo "*"
	@echo "*   Unable to build for $(TARGET), no toolchain available"
	@echo "*"
	@echo "******************************************************************"
else

include $(OPENMRNPATH)/etc/recurse.mk

all: $(EXECUTABLE)$(EXTENTION)

# Makes sure the subdirectory builds are done before linking the binary.
# The targets and variable BUILDDIRS are defined in recurse.mk.
$(FULLPATHLIBS): $(BUILDDIRS)

$(EXECUTABLE)$(EXTENTION): $(OBJS) $(FULLPATHLIBS)
	$(LD) -o $@ $(OBJS) $(OBJEXTRA) $(LDFLAGS) $(LIBS) $(SYSLIBRARIES)

-include $(OBJS:.o=.d)

.SUFFIXES:
.SUFFIXES: .o .c .cxx .cpp .S

.S.o:
	$(AS) $(ASFLAGS) $< -o $@
	$(AS) -MM $(ASFLAGS) $< > $*.d

.cpp.o:
	$(CXX) $(CXXFLAGS) $< -o $@
	$(CXX) -MM $(CXXFLAGS) $< > $*.d

.cxx.o:
	$(CXX) $(CXXFLAGS) $< -o $@
	$(CXX) -MM $(CXXFLAGS) $< > $*.d

.c.o:
	$(CC) $(CFLAGS) $< -o $@
	$(CC) -MM $(CFLAGS) $< > $*.d

clean: clean-local

clean-local:
	rm -rf *.o *.d *.a *.so $(EXECUTABLE)$(EXTENTION) $(EXECUTABLE).bin $(EXECUTABLE).lst

veryclean: clean-local

endif

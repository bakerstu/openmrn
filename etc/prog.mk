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
LIBS = -Wl,--start-group \
       $(foreach lib,$(LIBDIRS),-l$(lib)) \
       -Wl,--end-group \
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
all: subdirs $(EXECUTABLE)$(EXTENTION)

.PHONY: subdirs
subdirs:
	$(foreach dir, $(SUBDIRS), $(MAKE) -C $(dir) all || exit 1;)

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

clean:
	$(foreach dir, $(SUBDIRS), $(MAKE) -C $(dir) clean || exit 1;)
	rm -rf *.o *.d *.a *.so $(EXECUTABLE)$(EXTENTION)

verclean: clean
	$(foreach dir, $(SUBDIRS), $(MAKE) -C $(dir) veryclean || exit 1;)

endif

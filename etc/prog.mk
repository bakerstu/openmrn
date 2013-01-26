ifeq ($(TARGET),)
# if the target is so far undefined
TARGET := $(shell basename `pwd`)
endif
include $(OPENMRNPATH)/etc/$(TARGET).mk
VPATH = ../../
INCLUDES += -I$(OPENMRNPATH)/src/ -I$(OPENMRNPATH)/include
FULLPATHASMSRCS = $(wildcard $(VPATH)/*.S)
FULLPATHCSRCS = $(wildcard $(VPATH)/*.c)
FULLPATHCXXSRCS = $(wildcard $(VPATH)/*.cxx)
FULLPATHCPPSRCS = $(wildcard $(VPATH)/*.cpp)
ASMSRCS = $(notdir $(FULLPATHASMSRCS)) $(wildcard *.S)
CSRCS = $(notdir $(FULLPATHCSRCS)) $(wildcard *.c)
CXXSRCS = $(notdir $(FULLPATHCXXSRCS)) $(wildcard *.cxx)
CPPSRCS = $(notdir $(FULLPATHCPPSRCS)) $(wildcard *.cpp)
OBJS = $(CXXSRCS:.cxx=.o) $(CPPSRCS:.cpp=.o) $(CSRCS:.c=.o) $(ASMSRCS:.S=.o)
LIBNAME = lib$(BASENAME).a
LIBDIR = $(OPENMRNPATH)/targets/$(TARGET)/lib
FULLPATHLIBS = $(wildcard $(LIBDIR)/*.a)
LIBS = -lif -los -lcore
LDFLAGS += -L./ -L$(LIBDIR)
CFLAGS += $(INCLUDES)
CXXFLAGS += $(INCLUDES)

EXECUTABLE = $(shell basename `cd ../../; pwd`)

ifeq ($(TOOLPATH),)
all docs clean veryclean:
	@echo "******************************************************************"
	@echo "*"
	@echo "*   Unable to build for $(TARGET), no toolchain available"
	@echo "*"
	@echo "******************************************************************"
else
all: $(EXECUTABLE)$(EXTENTION)

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
	rm -rf *.o *.d *.a *.so $(EXECUTABLE)$(EXTENTION) lib

verclean: clean

endif

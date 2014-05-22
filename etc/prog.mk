ifeq ($(TARGET),)
# if the target is so far undefined
TARGET := $(shell basename `pwd`)
endif

include $(OPENMRNPATH)/etc/config.mk

exist := $(wildcard ../../subdirs)
ifneq ($(strip $(exist)),)
include ../../subdirs
endif

include $(OPENMRNPATH)/etc/$(TARGET).mk

include $(OPENMRNPATH)/etc/path.mk


VPATH = $(abspath ../../)

-include $(VPATH)/tests/sources

FULLPATHASMSRCS  = $(wildcard $(VPATH)/*.S)
FULLPATHCSRCS    = $(wildcard $(VPATH)/*.c)
FULLPATHCXXSRCS  = $(wildcard $(VPATH)/*.cxx)
FULLPATHCPPSRCS  = $(wildcard $(VPATH)/*.cpp)
FULLPATHXMLSRCS  = $(wildcard $(VPATH)/*.xml)
FULLPATHTESTSRCS ?= $(wildcard $(VPATH)/tests/*_test.cc)

ASMSRCS  = $(notdir $(FULLPATHASMSRCS)) $(wildcard *.S)
CSRCS    = $(notdir $(FULLPATHCSRCS))   $(wildcard *.c)
CXXSRCS  = $(notdir $(FULLPATHCXXSRCS)) $(wildcard *.cxx)
CPPSRCS  = $(notdir $(FULLPATHCPPSRCS)) $(wildcard *.cpp)
XMLSRCS  = $(notdir $(FULLPATHXMLSRCS)) $(wildcard *.xml)
TESTSRCS = $(notdir $(FULLPATHTESTSRCS)) $(wildcard *_test.cc)

$(info fullptest=$(FULLPATHTESTSRCS) test=$(TESTSRCS))

OBJS := $(CXXSRCS:.cxx=.o) $(CPPSRCS:.cpp=.o) $(CSRCS:.c=.o) $(ASMSRCS:.S=.o) \
       $(XMLSRCS:.xml=.o)
TESTOBJS := $(TESTSRCS:.cc=.o)

LIBDIR = $(OPENMRNPATH)/targets/$(TARGET)/lib
FULLPATHLIBS = $(wildcard $(LIBDIR)/*.a) $(wildcard lib/*.a)
LIBDIRS := $(SUBDIRS)
LIBS = $(STARTGROUP) \
       $(foreach lib,$(LIBDIRS),-l$(lib)) \
       $(ENDGROUP) \
       $(LINKCORELIBS)

#we don't have to recurse into lib, because there are no sources there. We don't need a liblib.a
#SUBDIRS += lib
INCLUDES += -I$(OPENMRNPATH)/src/ -I$(OPENMRNPATH)/include
ifdef APP_PATH
INCLUDES += -I$(APP_PATH)
else
#$(error no APP_PATH found)
endif
ifdef BOARD
INCLUDES += -D$(BOARD)
endif
CFLAGS += $(INCLUDES)
CXXFLAGS += $(INCLUDES)
LDFLAGS += -Llib -L$(LIBDIR)

EXECUTABLE = $(shell basename `cd ../../; pwd`)

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

# This defines how to create nonexistant directories.
MKSUBDIR_OPENMRNINCLUDE=applib.mk

ifneq ($(SUBDIRS),)
include $(OPENMRNPATH)/etc/recurse.mk
endif

all: $(EXECUTABLE)$(EXTENTION)

# Makes sure the subdirectory builds are done before linking the binary.
# The targets and variable BUILDDIRS are defined in recurse.mk.
$(FULLPATHLIBS): $(BUILDDIRS)

# This file acts as a guard describing when the last libsomething.a was remade
# in the application libraries.
lib/timestamp : FORCE $(BUILDDIRS)
	if [ ! -d lib ] ; then mkdir lib ; fi  # creates the lib directory
	if [ ! -f $@ ] ; then touch $@ ; fi  # in case there are not applibs.

# This file acts as a guard describing when the last libsomething.a was remade
# in the core target libraries.
$(LIBDIR)/timestamp: FORCE $(BUILDDIRS)
	make -C $(OPENMRNPATH)/targets/$(TARGET) all

# We cannot make lib/timestamp a phony target or else every test will always be
# remade.
FORCE:

$(EXECUTABLE)$(EXTENTION): $(OBJS) $(FULLPATHLIBS) $(LIBDIR)/timestamp lib/timestamp
	$(LD) -o $@ $(OBJS) $(OBJEXTRA) $(LDFLAGS) $(LIBS) $(SYSLIBRARIES)
ifdef SIZE
	$(SIZE) $@
endif
ifdef OBJDUMP
	$(OBJDUMP) -h $@
endif

$(EXECUTABLE).lst: $(EXECUTABLE)$(EXTENTION)
	$(OBJDUMP) -C -d $< > $@

ifndef CGMINSIZE
CGMINSIZE=300
endif

cg.svg: $(EXECUTABLE).lst $(OPENMRNPATH)/bin/callgraph.py
	$(OPENMRNPATH)/bin/callgraph.py --min_size $(CGMINSIZE) --map $(EXECUTABLE).map < $(EXECUTABLE).lst 2> cg.debug.txt | tee cg.dot | dot -Tsvg > cg.svg

-include $(OBJS:.o=.d)
-include $(TESTOBJS:.o=.d)

.SUFFIXES:
.SUFFIXES: .o .c .cxx .cpp .S .xml .cout .cxxout

.xml.o:
	$(OPENMRNPATH)/bin/build_cdi.py -i $< -o $*.cxxout
	$(CXX) $(CXXFLAGS) -x c++ $*.cxxout -o $@
	$(CXX) -MM $(CXXFLAGS) $*.cxxout > $*.d

.S.o:
	$(AS) $(ASFLAGS) $< -o $@
	$(AS) -MM $(ASFLAGS) $< > $*.d

.cpp.o:
	$(CXX) $(CXXFLAGS) $< -o $@
	$(CXX) -MM $(CXXFLAGS) $< > $*.d

.cxx.o:
	$(CXX) $(CXXFLAGS) $(realpath $<) -o $@
	$(CXX) -MM $(CXXFLAGS) $< > $*.d

.c.o:
	$(CC) $(CFLAGS) $< -o $@
	$(CC) -MM $(CFLAGS) $< > $*.d

clean: clean-local

clean-local:
	rm -rf *.o *.d *.a *.so *.output *.cout *.cxxout $(TESTOBJS:.o=) $(EXECUTABLE)$(EXTENTION) $(EXECUTABLE).bin $(EXECUTABLE).lst $(EXECUTABLE).map cg.debug.txt cg.dot cg.svg
	rm -rf $(XMLSRCS:.xml=.c)

veryclean: clean-local

TEST_MISSING_DEPS:=$(call find_missing_deps,HOST_TARGET GTESTPATH GTESTSRCPATH GMOCKPATH GMOCKSRCPATH)

ifneq ($(TEST_MISSING_DEPS),)
tests:
	@echo "***Not building tests at target $(TARGET), because missing: $(TEST_MISSING_DEPS) ***"

else
VPATH:=$(VPATH):$(GTESTPATH)/src:$(GTESTSRCPATH):$(GMOCKPATH)/src:$(GMOCKSRCPATH):$(abspath ../../tests)
INCLUDES += -I$(GTESTPATH)/include -I$(GTESTPATH) -I$(GMOCKPATH)/include -I$(GMOCKPATH)

TEST_OUTPUTS=$(TESTOBJS:.o=.output)

TEST_EXTRA_OBJS += gtest-all.o gmock-all.o

.cc.o:
	$(CXX) $(CXXFLAGS) $< -o $@
	$(CXX) -MM $(CXXFLAGS) $< > $*.d

gtest-all.o : %.o : $(GTESTSRCPATH)/src/%.cc
	$(CXX) $(CXXFLAGS) -I$(GTESTPATH) -I$(GTESTSRCPATH)  $< -o $@
	$(CXX) -MM $(CXXFLAGS) -I$(GTESTPATH) -I$(GTESTSRCPATH) $< > $*.d

gmock-all.o : %.o : $(GMOCKSRCPATH)/src/%.cc
	$(CXX) $(CXXFLAGS) -I$(GMOCKPATH) -I$(GMOCKSRCPATH)  $< -o $@
	$(CXX) -MM $(CXXFLAGS) -I$(GMOCKPATH) -I$(GMOCKSRCPATH) $< > $*.d

#.PHONY: $(TEST_OUTPUTS)

$(TEST_OUTPUTS) : %_test.output : %_test
	./$*_test --gtest_death_test_style=threadsafe
	touch $@

$(TESTOBJS:.o=) : %_test : %_test.o $(TEST_EXTRA_OBJS) $(FULLPATHLIBS) $(LIBDIR)/timestamp lib/timestamp
	$(LD) -o $*_test$(EXTENTION) $*_test.o $(TEST_EXTRA_OBJS) $(OBJEXTRA) $(LDFLAGS)  $(LIBS) $(SYSLIBRARIES) -lstdc++

$(info test deps: $(FULLPATHLIBS) )

%_test.o : %_test.cc
	$(CXX) $(CXXFLAGS:-Werror=) -DTESTING -fpermissive  $< -o $*_test.o
	$(CXX) -MM $(CXXFLAGS) $< > $*_test.d

#$(TEST_OUTPUTS) : %_test.output : %_test.cc gtest-all.o gtest_main.o
#	$(CXX) $(CXXFLAGS) $< -o $*_test.o
#	$(CXX) -MM $(CXXFLAGS) $< > $*_test.d
#	$(LD) -o $*_test$(EXTENTION) $+ $(OBJEXTRA) $(LDFLAGS) $(LIBS) $(SYSLIBRARIES) -lstdc++
#	./$*_test

tests : all $(TEST_OUTPUTS)

mksubdirs:
	[ -d lib ] || mkdir lib

endif  # if we are able to run tests

endif

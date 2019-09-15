
ifeq ($(TARGET),)
# if the target is so far undefined
TARGET := $(notdir $(CURDIR))
endif

include $(OPENMRNPATH)/etc/config.mk

exist := $(wildcard ../../subdirs)
ifneq ($(strip $(exist)),)
include ../../subdirs
endif

include $(OPENMRNPATH)/etc/$(TARGET).mk

include $(OPENMRNPATH)/etc/path.mk
include $(OPENMRNPATH)/etc/make_utils.mk

GITREPOS += $(OPENMRNPATH)

VPATH = $(abspath ../../)

-include $(VPATH)/tests/sources

FULLPATHASMSRCS  := $(wildcard $(VPATH)/*.S)
FULLPATHCSRCS    := $(wildcard $(VPATH)/*.c)
FULLPATHCXXSRCS  := $(wildcard $(VPATH)/*.cxx)
FULLPATHCPPSRCS  := $(wildcard $(VPATH)/*.cpp)
FULLPATHXMLSRCS  := $(wildcard $(VPATH)/*.xml)

ASMSRCS  = $(notdir $(FULLPATHASMSRCS)) $(wildcard *.S)
CSRCS    = $(notdir $(FULLPATHCSRCS))   $(wildcard *.c)
CXXSRCS  = $(notdir $(FULLPATHCXXSRCS)) $(wildcard *.cxx)
CPPSRCS  = $(notdir $(FULLPATHCPPSRCS)) $(wildcard *.cpp)
XMLSRCS  = $(notdir $(FULLPATHXMLSRCS)) $(wildcard *.xml)

OBJS = $(CXXSRCS:.cxx=.o) $(CPPSRCS:.cpp=.o) $(CSRCS:.c=.o) $(ASMSRCS:.S=.o) \
       $(XMLSRCS:.xml=.o)

LIBDIR = $(OPENMRNPATH)/targets/$(TARGET)/lib
FULLPATHLIBS = $(wildcard $(LIBDIR)/*.a) $(wildcard lib/*.a)
LIBDIRS := $(SUBDIRS)
LIBS = $(STARTGROUP) \
       $(foreach lib,$(LIBDIRS),-l$(lib)) \
       $(LINKCORELIBS) \
       $(ENDGROUP) \


all:


# This ensures that link targets that depend on lib/libfoo.a will recurse into
# the directory foo and rebuild stuff that's there. However, the dependency is
# phrased in a way that if recursing does not change the library (when it's
# up-to-date) then the .elf linking is not re-done.
$(foreach lib,$(LIBDIRS),$(eval $(call DEP_helper_template,lib/lib$(lib).a,build-$(lib))))

CDIEXTRA := -I.
INCLUDES += -I.

#we don't have to recurse into lib, because there are no sources there. We don't need a liblib.a
#SUBDIRS += lib
INCLUDES += -I$(OPENMRNPATH)/src/ -I$(OPENMRNPATH)/include
ifdef APP_PATH
INCLUDES += -I$(APP_PATH)
CDIEXTRA += -I$(APP_PATH)
else
#$(error no APP_PATH found)
endif
ifdef BOARD
INCLUDES += -D$(BOARD)
CDIEXTRA += -D$(BOARD)
endif
CFLAGS += $(INCLUDES)
CXXFLAGS += $(INCLUDES)
ifeq ($(TARGET),bare.pruv3)
LDFLAGS += -ilib -i$(LIBDIR)
else
LDFLAGS += -Llib -L$(LIBDIR)
endif

EXECUTABLE ?= $(notdir $(realpath $(CURDIR)/../..))


ifeq ($(OS),Windows_NT)
include $(OPENMRNPATH)/etc/path_windows.mk
else
    ifeq ($(shell uname -s),Darwin)
        CDIEXTRA += -I$(OPENMRNPATH)/include/mach
    endif
endif

DEPS += TOOLPATH
MISSING_DEPS:=$(call find_missing_deps,$(DEPS))

ifneq ($(MISSING_DEPS),)
all docs clean veryclean tests mksubdirs: print_error_deps


print_error_deps:
	@echo "** Ignoring target $(TARGET), because the following libraries are not installed: $(MISSING_DEPS). This is not an error, so please do not report as a bug. If you care about target $(TARGET), make sure the quoted libraries are installed. For most libraries you can check $(OPENMRNPATH)/etc/path.mk to see where we looked for these dependencies."

else

# This defines how to create nonexistant directories.
MKSUBDIR_OPENMRNINCLUDE=applib.mk

ifneq ($(SUBDIRS),)
include $(OPENMRNPATH)/etc/recurse.mk
else
all: mksubdirs

mksubdirs:

endif

# handle the revision metadata
all: $(EXECUTABLE)$(EXTENTION)

-include *.d

.PHONY: FORCE
Revision.hxxout: FORCE
	$(OPENMRNPATH)/bin/revision.py $(REVISIONFLAGS) -t -i "$(GITREPOS)" -g "`$(CC) -dumpversion`"

# This part detects whether we have a config.hxx defining CDI data and if yes,
# then compiles it into an xml and object file.
HAVE_CONFIG_CDI := $(shell grep ConfigDef config.hxx 2>/dev/null)
ifneq ($(HAVE_CONFIG_CDI),)
ifeq ($(SKIP_CONFIG_CDI),)
OBJS += cdi.o
endif

$(EXECUTABLE)$(EXTENTION): cdi.o

cdi.o : compile_cdi
	./compile_cdi > cdi.cxx
	$(CXX) $(CXXFLAGS) -x c++ cdi.cxx -o $@
	mv cdi.cxx cdi.cxxout
	rm -f cdi.d

compile_cdi: config.hxx $(OPENMRNPATH)/src/openlcb/CompileCdiMain.cxx
	g++ -o $@ -I. -I$(OPENMRNPATH)/src -I$(OPENMRNPATH)/include $(CDIEXTRA)  --std=c++11 -MD -MF $@.d $(CXXFLAGSEXTRA) $(OPENMRNPATH)/src/openlcb/CompileCdiMain.cxx

config.hxx: Revision.hxxout

$(OBJS): compile_cdi

clean: clean_cdi

.PHONY: clean_cdi
clean_cdi:
	rm -f cdi.xmlout cdi.nxml cdi.cxxout compile_cdi
endif

# Makes sure the subdirectory builds are done before linking the binary.
# The targets and variable BUILDDIRS are defined in recurse.mk.
#$(FULLPATHLIBS): $(BUILDDIRS)

# This file acts as a guard describing when the last libsomething.a was remade
# in the application libraries.
lib/timestamp : FORCE $(BUILDDIRS)
#	 creates the lib directory
	@[ -d lib ] || mkdir lib
# in case there are not applibs.
	@[ -f $@ ] || touch $@  

# Detect when we have a compound toplevel build and use the toplevel build
# timestamp to decide whether we need to recurse into the target
# directory. This saves a lot of makefile recursion when there are multiple
# application targets built together that refer of the same openmrn lib target.
ifdef HAVE_BUILD_TIMESTAMP
LIBBUILDDEP:=$(HAVE_BUILD_TIMESTAMP)
else
LIBBUILDDEP:=FORCE
endif

# This file acts as a guard describing when the last libsomething.a was remade
# in the core target libraries.
$(LIBDIR)/timestamp: $(LIBBUILDDEP) $(BUILDDIRS)
ifdef FLOCKPATH
	@$(FLOCKPATH)/flock $(OPENMRNPATH)/targets/$(TARGET) -c "if [ $< -ot $(LIBBUILDDEP) -o ! -f $(LIBBUILDDEP) ] ; then $(MAKE) -C $(OPENMRNPATH)/targets/$(TARGET) all ; else echo short-circuiting core target build ; fi"
else
	@echo warning: no flock support. If you use make -jN then you can run into occasional compilation errors when multiple makes are progressing in the same directory. Usually re-running make solved them.
	$(MAKE) -C $(OPENMRNPATH)/targets/$(TARGET) all
endif

# We cannot make lib/timestamp a phony target or else every test will always be
# remade.
FORCE:


rclean: clean
	$(MAKE) -C $(OPENMRNPATH)/targets/$(TARGET) clean


$(EXECUTABLE)$(EXTENTION): $(OBJS) $(FULLPATHLIBS) $(LIBDIR)/timestamp lib/timestamp $(OPENMRNPATH)/etc/$(TARGET).mk 
	$(LD) $(OBJS) $(OBJEXTRA) $(LDFLAGS) $(LIBS) $(STARTGROUP) $(SYSLIBRARIES) $(ENDGROUP) -o $@ 
ifdef SIZE
	$(SIZE) $@
endif

# Makes the executable recompiled if the linker script has changed.
ifneq ($(strip $(wildcard target.ld)),)
$(EXECUTABLE)$(EXTENTION): target.ld
endif
ifneq ($(strip $(wildcard memory_map.ld)),)
$(EXECUTABLE)$(EXTENTION): memory_map.ld
endif

ifdef OBJDUMP
all:  $(EXECUTABLE).lst

ifndef OBJDUMPOPTS
OBJDUMPOPTS=-C
endif

$(EXECUTABLE).lst: $(EXECUTABLE)$(EXTENTION)
	$(OBJDUMP) $(OBJDUMPOPTS) -d -h $< > $@

$(EXECUTABLE).slst: $(EXECUTABLE)$(EXTENTION)
	$(OBJDUMP) $(OBJDUMPOPTS) -d -S -h $< > $@

$(EXECUTABLE).ndlst: $(EXECUTABLE)$(EXTENTION)
	$(OBJDUMP) -d $< > $@

$(EXECUTABLE).bin: $(EXECUTABLE)$(EXTENTION)
	$(OBJCOPY) -O binary $< $@

endif


ifndef CGMINSIZE
CGMINSIZE=300
endif

# You can also try make cg.svg CGARGS=--focus=appl_main
cg.svg: $(EXECUTABLE).ndlst $(OPENMRNPATH)/bin/callgraph.py
	$(OPENMRNPATH)/bin/callgraph.py --max_indep 6 --min_size $(CGMINSIZE) $(CGARGS) --map $(EXECUTABLE).map < $(EXECUTABLE).ndlst 2> cg.debug.txt | tee cg.dot | dot -Tsvg > cg.svg

-include $(OBJS:.o=.d)
-include $(TESTOBJS:.o=.d)

.SUFFIXES:
.SUFFIXES: .o .c .cxx .cpp .S .xml .cout .cxxout

.xml.o: $(OPENMRNPATH)/bin/build_cdi.py
	$(OPENMRNPATH)/bin/build_cdi.py -i $< -o $*.cxxout
	$(CXX) $(CXXFLAGS) -x c++ $*.cxxout -o $@
	$(CXX) -MM $(CXXFLAGS) -x c++ $*.cxxout > $*.d

ifeq ($(TARGET),bare.pruv3)
.cpp.o:
	$(CXX) $(CXXFLAGS) $<

.cxx.o:
	$(CXX) $(CXXFLAGS) $<

.S.o:
	$(AS) $(ASFLAGS) $<

.c.o:
	$(CC) $(CFLAGS) $<

else
.cpp.o:
	$(CXX) $(CXXFLAGS) -MD -MF $*.d $(abspath $<) -o $@

.cxx.o:
	$(CXX) $(CXXFLAGS) -MD -MF $*.d $(abspath $<) -o $@

.S.o:
	$(AS) $(ASFLAGS) -MD -MF $*.d $(abspath $<) -o $@

.c.o:
	$(CC) $(CFLAGS) -MD -MF $*.d $(abspath $<) -o $@
endif

clean: clean-local

clean-local:
	rm -f $(wildcard *.o *.d *.a *.so *.output *.cout *.cxxout *.hxxout *.stripped lib/*.stripped lib/*.lst) $(TESTOBJS:.o=) $(EXECUTABLE)$(EXTENTION) $(EXECUTABLE).bin $(EXECUTABLE).lst $(EXECUTABLE).map cg.debug.txt cg.dot cg.svg gmon.out $(OBJS) demangled.txt $(EXECUTABLE).ndlst objcopy.params
	rm -rf $(XMLSRCS:.xml=.c)

veryclean: clean-local clean

TEST_MISSING_DEPS:=$(call find_missing_deps,HOST_TARGET GTESTPATH GTESTSRCPATH GMOCKPATH GMOCKSRCPATH)

ifneq ($(TEST_MISSING_DEPS),)
tests:
	@echo "***Not building tests at target $(TARGET), because missing: $(TEST_MISSING_DEPS) ***"

else
ifeq (1,1)

SRCDIR=$(abspath ../../)
#old code from prog.mk
#$(TEST_EXTRA_OBJS) $(OBJEXTRA) $(LDFLAGS)  $(LIBS) $(SYSLIBRARIES)
#new code in core_test.mk
#$(LDFLAGS) -los  $< $(TESTOBJSEXTRA) $(LINKCORELIBS) $(SYSLIBRARIES) 
#TESTOBJSEXTRA += $(TEST_EXTRA_OBJS)
SYSLIBRARIES += $(LIBS)
TESTEXTRADEPS += lib/timestamp
include $(OPENMRNPATH)/etc/core_test.mk

else
FULLPATHTESTSRCS ?= $(wildcard $(VPATH)/tests/*_test.cc)
TESTSRCS = $(notdir $(FULLPATHTESTSRCS)) $(wildcard *_test.cc)
TESTOBJS := $(TESTSRCS:.cc=.o)

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
endif # old testrunner code

endif  # if we are able to run tests

endif

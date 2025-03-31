# Helper makefile for generating test execution targets for a core_target.mk
#
# Prerequisites:
# - target.mk is loaded
# - HOST_TARGET or EMU is defined
# - there is an $(SRCDIR) symbol defined with the location of the source files.

ifneq ($(HOST_TARGET)$(EMU),0)

include $(OPENMRNPATH)/etc/make_utils.mk

FULLPATHCXXTESTSRCS := $(foreach DIR,$(SUBDIRS) tests,$(wildcard $(SRCDIR)/$(DIR)/*.cxxtest))

TESTOBJSEXTRA += gtest-all.o gmock-all.o
TESTSRCS ?= $(patsubst $(SRCDIR)/%,%,$(FULLPATHCXXTESTSRCS))
ifdef TESTBLACKLIST
TESTSRCS := $(filter-out $(TESTBLACKLIST),$(TESTSRCS))
endif
TESTOBJS = $(TESTSRCS:.cxxtest=.test.o)
TESTBINS = $(TESTSRCS:.cxxtest=.test$(EXTENTION))
TESTOUTPUTS = $(TESTSRCS:.cxxtest=.testout)
TESTMD5 = $(TESTSRCS:.cxxtest=.testmd5)

alltest-%: 
	+$(MAKE) $(filter $(@:alltest-%=%)/%,$(TESTOUTPUTS))

INCLUDES += -I$(GTESTPATH)/include -I$(GMOCKPATH)/include -I$(GMOCKPATH) \
            -I$(OPENMRNPATH)/src -I$(OPENMRNPATH)/include

CFLAGS += $(INCLUDES)
CXXFLAGS += $(INCLUDES)

.SUFFIXES: .o .otest .c .cxx .cxxtest .test .testmd5 .testout

ifdef LIBDIR
# we are under prog.mk
TESTLIBDEPS += $(foreach lib,$(SUBDIRS),lib/lib$(lib).a)

# This ensures that link targets that depend on lib/libfoo.a will recurse into
# the directory foo and rebuild stuff that's there. However, the dependency is
# phrased in a way that if recursing does not change the library (when it's
# up-to-date) then the .elf linking is not re-done.
$(foreach lib,$(SUBDIRS),$(eval $(call SUBDIR_helper_template,$(lib))))

# Ensures that when the core target is clean (missing lib/libfoo.a), make knows
# how to build its libraries. This also ensures that the text.executable files
# are remade when something changes in the openmrn codebase.
$(foreach lib,$(CORELIBS),$(LIBDIR)/lib$(lib).a): $(LIBDIR)/timestamp

else
LIBDIR = lib

# This ensures that link targets that depend on lib/libfoo.a will recurse into
# the directory foo and rebuild stuff that's there. However, the dependency is
# phrased in a way that if recursing does not change the library (when it's
# up-to-date) then the .elf linking is not re-done.
$(foreach lib,$(CORELIBS),$(eval $(call SUBDIR_helper_template,$(lib))))

endif
TESTLIBDEPS += $(foreach lib,$(CORELIBS),$(LIBDIR)/lib$(lib).a)

LDFLAGS      += -L$(LIBDIR)

$(LIBDIR)/timestamp: $(BUILDDIRS)


$(info test deps $(TESTOBJSEXTRA) $(LIBDIR)/timestamp | $(BUILDDIRS) )
$(TESTBINS): %.test$(EXTENTION) : %.test.o $(TESTOBJSEXTRA) $(LIBDIR)/timestamp lib/timestamp $(TESTLIBDEPS) $(TESTEXTRADEPS) | $(BUILDDIRS)
	$(LD) -o $@ $(LDFLAGS) -los  $< $(TESTOBJSEXTRA) $(LIBS) $(STARTGROUP) $(LINKCORELIBS) $(ENDGROUP) $(SYSLIBRARIES) 

-include $(TESTOBJS:.test.o=.dtest)
-include $(TESTOBJSEXTRA:.o=.d)

$(TESTOBJS): %.test.o : $(SRCDIR)/%.cxxtest
	$(CXX) $(CXXFLAGS) -MMD -MF $*.dtest -MT $@ -x c++ $< -o $@

gtest-all.o : %.o : $(GTESTSRCPATH)/src/%.cc
	$(CXX) $(CXXFLAGS) -Wno-uninitialized -Wno-maybe-uninitialized -I$(GTESTPATH) -I$(GTESTSRCPATH) -MMD -MF $*.d   $< -o $@

gmock-all.o : %.o : $(GMOCKSRCPATH)/src/%.cc
	$(CXX) $(CXXFLAGS) -I$(GMOCKPATH) -I$(GMOCKSRCPATH) -MMD -MF $*.d  $< -o $@

# This target takes the test binary output and compares the md5sum against the
# md5sum of the previous run. If the md5sum of the test binary didn't change,
# we do not re-run the test. We tell this to make by not updating the timestmap
# of the .md5sum even though these commands run. Make will then not run the
# dependent commands (the actual test run) and assumes that the .testout is
# also up-to-date.
%.testmd5 : %.test$(EXTENTION)
	@SM="$$(md5sum $<)" ; if [ ! -f $@ ] || [ "$$SM" != "$$(<$@)" ] ; then echo replacing md5 file. old: $$(<$@) new $$SM ; echo "$$SM" > $@ ; else echo test output up-to-date for $(TARGET):$@ ; fi

ifndef CUSTOM_EXEC
# This target actually runs the test. We jump through some hoops to collect the
# coverage files into a separate directory. Since they are in a separate directory, we need to put the original .gcno files there as well.
%.testout : %.testmd5
	$(EMU) $(<:.testmd5=.test$(EXTENTION)) --gtest_death_test_style=threadsafe $(TESTARGS)
	touch $@

endif

run-tests: $(TESTOUTPUTS) $(TESTMD5)

run-tests-single:
	$(MAKE) $(TESTMD5)
	$(MAKE) -j1 $(TESTOUTPUTS)

tests: run-tests

tests-single: run-tests-single

clean-gtest:
	rm -f {gtest-all,gmock-all}.{d,o,gcno}
	rm -rf $(TESTBINS) $(TESTOBJS) $(TESTOUTPUTS) $(TESTMD5)

clean veryclean: clean-gtest

endif

# Helper makefile for generating test execution targets for a core_target.mk
#
# Prerequisites:
# - target.mk is loaded
# - HOST_TARGET or EMU is defined
# - there is an $(SRCDIR) symbol defined with the location of the source files.

ifneq ($(HOST_TARGET)$(EMU),0)

FULLPATHCXXTESTSRCS := $(foreach DIR,$(SUBDIRS),$(wildcard $(SRCDIR)/$(DIR)/*.cxxtest))

TESTOBJSEXTRA = gtest-all.o gmock-all.o

TESTSRCS ?= $(patsubst $(SRCDIR)/%,%,$(FULLPATHCXXTESTSRCS))
TESTBINS = $(TESTSRCS:.cxxtest=.test)
TESTOBJS = $(TESTSRCS:.cxxtest=.otest)
TESTOUTPUTS = $(TESTSRCS:.cxxtest=.testout)
TESTMD5 = $(TESTSRCS:.cxxtest=.testmd5)

INCLUDES += -I$(GTESTPATH)/include -I$(GMOCKPATH)/include -I$(GMOCKPATH) \
            -I$(OPENMRNPATH)/src -I$(OPENMRNPATH)/include

CFLAGS += $(INCLUDES)
CXXFLAGS += $(INCLUDES)

.SUFFIXES: .o .otest .c .cxx .cxxtest .test .testmd5 .testout

LIBDIR = lib
LDFLAGS      += -L$(LIBDIR)

$(LIBDIR)/timestamp: $(BUILDDIRS)

$(TESTBINS): %.test : %.otest $(TESTOBJSEXTRA) $(LIBDIR)/timestamp | $(BUILDDIRS)
	$(LD) -o $@ $(LDFLAGS) -los  $< $(TESTOBJSEXTRA) $(LINKCORELIBS) $(SYSLIBRARIES) 

-include $(TESTOBJS:.otest=.dtest)
-include $(TESTOBJSEXTRA:.o=.d)

$(TESTOBJS): %.otest : $(SRCDIR)/%.cxxtest
	$(CXX) $(CXXFLAGS) -MD -MF $*.dtest -x c++ $< -o $@

gtest-all.o : %.o : $(GTESTSRCPATH)/src/%.cc
	$(CXX) $(CXXFLAGS) -I$(GTESTPATH) -I$(GTESTSRCPATH)  $< -o $@
	$(CXX) -MM $(CXXFLAGS) -I$(GTESTPATH) -I$(GTESTSRCPATH) $< > $*.d

gmock-all.o : %.o : $(GMOCKSRCPATH)/src/%.cc
	$(CXX) $(CXXFLAGS) -I$(GMOCKPATH) -I$(GMOCKSRCPATH)  $< -o $@
	$(CXX) -MM $(CXXFLAGS) -I$(GMOCKPATH) -I$(GMOCKSRCPATH) $< > $*.d

# This target takes the test binary output and compares the md5sum against the
# md5sum of the previous run. If the md5sum of the test binary didn't change,
# we do not re-run the test. We tell this to make by not updating the timestmap
# of the .md5sum even though these commands run. Make will then not run the
# dependent commands (the actual test run) and assumes that the .testout is
# also up-to-date.
%.testmd5 : %.test
	@SM="$$(md5sum $<)" ; if [ ! -f $@ ] || [ "$$SM" != "$$(<$@)" ] ; then echo replacing md5 file. old: $$(<$@) new $$SM ; echo "$$SM" > $@ ; else echo test output up-to-date for $(TARGET):$@ ; fi

ifndef CUSTOM_EXEC
# This target actually runs the test. We jump through some hoops to collect the
# coverage files into a separate directory. Since they are in a separate directory, we need to put the original .gcno files there as well.
%.testout : %.testmd5
	$(EMU) $(<:.testmd5=.test) --gtest_death_test_style=threadsafe
	touch $@

endif

run-tests: $(TESTOUTPUTS) $(TESTMD5)

tests: run-tests

clean-gtest:
	rm -f {gtest-all,gmock-all}.{d,o,gcno}
	rm -rf $(TESTBINS) $(TESTOBJS) $(TESTOUTPUTS) $(TESTMD5)

clean veryclean: clean-gtest

endif

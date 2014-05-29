include ../../etc/core_target.mk

include $(OPENMRNPATH)/etc/cov.mk
include $(OPENMRNPATH)/etc/path.mk

SRCDIR = $(OPENMRNPATH)/src
FULLPATHCXXTESTSRCS := $(foreach DIR,$(SUBDIRS),$(wildcard $(SRCDIR)/$(DIR)/*.cxxtest))

TESTOBJSEXTRA = gtest-all.o gmock-all.o

TESTSRCS ?= $(patsubst $(SRCDIR)/%,%,$(FULLPATHCXXTESTSRCS))
TESTBINS = $(TESTSRCS:.cxxtest=.test)
TESTOBJS = $(TESTSRCS:.cxxtest=.otest)
TESTOUTPUTS = $(TESTSRCS:.cxxtest=.testout)
TESTMD5 = $(TESTSRCS:.cxxtest=.testmd5)
TESTDIRS = $(TESTSRCS:.cxxtest=.covdir)

INCLUDES     += -I$(GTESTPATH)/include -I$(GMOCKPATH)/include -I$(GMOCKPATH) \
                -I$(OPENMRNPATH)/src -I$(OPENMRNPATH)/include
LIBDIR = lib
LDFLAGS      += -L$(LIBDIR)


.SUFFIXES: .o .otest .c .cxx .cxxtest .test .testmd5 .testout

$(TESTBINS): %.test : %.otest $(TESTOBJSEXTRA) | $(BUILDDIRS)
	$(LD) -o $@ $(LDFLAGS) -los  $< $(TESTOBJSEXTRA) $(LINKCORELIBS) $(SYSLIBRARIES) 

-include $(TESTOBJS:.otest=.dtest)
-include $(TESTOBJSEXTRA:.o=.d)

$(TESTOBJS): %.otest : $(SRCDIR)/%.cxxtest
	$(CXX) $(CXXFLAGS) -MF $*.dtest -x c++ $< -o $@

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
	SM="$$(md5sum $<)" ; if [ ! -f $@ ] || [ "$$SM" != "$$(<$@)" ] ; then echo replacing md5 file. old: $$(<$@) new $$SM ; echo "$$SM" > $@ ; else echo not replacing md5 file $@ ; fi

# This target actually runs the test. We jump through some hoops to collect the
# coverage files into a separate directory. Since they are in a separate directory, we need to put the original .gcno files there as well.
%.testout : %.testmd5
	mkdir -p $*.covdir
	rm -f $*.covdir/app.info
	lcov --directory $*.covdir/ -z
	GCOV_PREFIX=$(abspath $*.covdir) GCOV_PREFIX_STRIP=$(shell echo $(realpath .) | tr -C -d '/' | wc -c) $(<:.testmd5=.test) --gtest_death_test_style=threadsafe
	cd $*.covdir/ ; for i in $$(find . -name \*.gcda) ; do cp --remove-destination $(realpath .$)/$${i%%.gcda}.gcno $${i%%.gcda}.gcno ; done && lcov --directory . --capture --output-file app.info
	touch $@

COVAGGRDIRS = $(dir $(TESTBINS))
COVAGGRFILES_DUP = $(foreach DIR,$(dir $(TESTBINS)),$(DIR)cov.info)
COVAGGRFILES = $(shell echo $(COVAGGRFILES_DUP) | tr ' ' '\n' | sort | uniq)

# Helper function for aggregating coverage files into per-directory templates.
#
# arg 1: a .testout file
#
# Adds the per-test coverage.info file to the dependencies to the directory's
# coverage file.
define DIRCOV_template
$(dir $(1))cov.info: $(1:.testout=.covdir/app.info)
endef

# Calls the directory template for every .testout file we have.
$(foreach testout,$(TESTOUTPUTS),$(eval $(call DIRCOV_template,$(testout))))

$(COVAGGRFILES):
	rm -f $@
	lcov --output-file $@ $(foreach F,$^, -a $F)

cov: lcovdir/index.html

lcovdir/index.html: $(TESTOUTPUTS) $(TESTMD5) $(COVAGGRFILES)
	mkdir lcovdir ; \
	cd lcovdir ; rm app.info; \
	lcov --output-file app.info $(foreach F,$(COVAGGRFILES), -a ../$F) ; \
	lcov -r app.info "/usr/include/*" "*.cxxtest" "*gtest*" "*gmock*" -o app.info; \
	genhtml -o . app.info

run-tests: $(TESTOUTPUTS) $(TESTMD5)

tests: run-tests

veryclean-cov:
	rm -rf lcovdir/

veryclean: veryclean-cov clean-cov

clean-cov:
	rm -rf $(TESTBINS) $(TESTOBJS) $(TESTOUTPUTS) $(TESTMD5) $(TESTDIRS) $(COVAGGRFILES)

clean: clean-cov

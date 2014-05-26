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

INCLUDES     += -I$(GTESTPATH)/include -I$(GMOCKPATH)/include -I$(GMOCKPATH) \
                -I$(OPENMRNPATH)/src -I$(OPENMRNPATH)/include
LIBDIR = lib
LDFLAGS      += -L$(LIBDIR) #-Wl,--undefined=appl_main


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

%.testmd5 : %.test
	SM="$$(md5sum $<)" ; if [ ! -f $@ ] || [ "$$SM" != "$$(<$@)" ] ; then echo replacing md5 file. old: $$(<$@) new $$SM ; echo "$$SM" > $@ ; else echo not replacing md5 file $@ ; fi

%.testout : %.testmd5
	mkdir -p $*.covdir
	rm -f $*.covdir/app.info
	lcov --directory $*.covdir/ -z
	GCOV_PREFIX=$(abspath $*.covdir) GCOV_PREFIX_STRIP=$(shell echo $(realpath .) | tr -C -d '/' | wc -c) $(<:.testmd5=.test) --gtest_death_test_style=threadsafe
	cd $*.covdir/ ; for i in $$(find . -name \*.gcda) ; do cp --remove-destination $(realpath .$)/$${i%%.gcda}.gcno $${i%%.gcda}.gcno ; done && lcov --directory . --capture --output-file app.info
	touch $@

FORCE:

cov: lcovdir/index.html


lcovdir/index.html: $(TESTOUTPUTS) $(TESTMD5)
	mkdir lcovdir ; \
	cd lcovdir ; rm app.info; \
	lcov --output-file app.info $(foreach F,$(TESTBINS:.test=.covdir/app.info), -a ../$F) ; \
	lcov -r app.info "/usr/include/*" "*.cxxtest" "*gtest*" -o app.info; \
	genhtml -o . app.info

run-tests: $(TESTOUTPUTS) $(TESTMD5)

$(info $(TESTBINS))

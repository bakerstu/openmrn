ifeq ($(BASENAME),)
# if the basename is so far undefined
BASENAME := $(shell basename `pwd`)
endif

include $(OPENMRNPATH)/etc/config.mk

include $(OPENMRNPATH)/etc/linux.x86.mk

include $(OPENMRNPATH)/etc/path.mk

SRCDIR = $(OPENMRNPATH)/src/$(BASENAME)/
VPATH = $(SRCDIR):$(GMOCKPATH)/src:$(GMOCKSRCPATH)

exist := $(wildcard $(SRCDIR)/sources)
ifneq ($(strip $(exist)),)
include $(SRCDIR)/sources
else
exist := $(wildcard sources)
ifneq ($(strip $(exist)),)
include sources
else
FULLPATHCSRCS        = $(wildcard $(VPATH)*.c)
FULLPATHCXXSRCS      = $(wildcard $(VPATH)*.cxx)
FULLPATHCXXTESTSRCS  = $(wildcard $(VPATH)*.cxxtest)

CSRCS       = $(notdir $(FULLPATHCSRCS))       $(wildcard *.c)
CXXSRCS     = $(notdir $(FULLPATHCXXSRCS))     $(wildcard *.cxx)
CXXTESTSRCS = $(notdir $(FULLPATHCXXTESTSRCS)) $(wildcard *.cxxtest)
endif
endif

OBJS = $(CXXSRCS:.cxx=.o) $(CSRCS:.c=.o)
TESTOBJS = $(CXXTESTSRCS:.cxxtest=.otest)
TESTOBJSEXTRA = gtest-all.o gmock-all.o

TESTBINS = $(CXXTESTSRCS:.cxxtest=.test)
TESTOUTPUTS = $(CXXTESTSRCS:.cxxtest=.testout)

LIBDIR = $(OPENMRNPATH)/targets/linux.x86/lib
FULLPATHLIBS = $(wildcard $(LIBDIR)/*.a) $(wildcard lib/*.a)
LIBDIRS := $(SUBDIRS)
LIBS = $(STARTGROUP) \
       $(foreach lib,$(LIBDIRS),-l$(lib)) \
       $(ENDGROUP) \
       $(LINKCORELIBS)

INCLUDES     += -I$(GTESTPATH)/include -I$(GMOCKPATH)/include -I$(GMOCKPATH) \
                -I$(OPENMRNPATH)/src -I$(OPENMRNPATH)/include
CFLAGS       += -DGTEST $(INCLUDES) -Wno-unused-but-set-variable -fprofile-arcs -ftest-coverage -O0
CXXFLAGS     += -DGTEST $(INCLUDES) -Wno-unused-but-set-variable -fprofile-arcs -ftest-coverage -O0
SYSLIBRARIES += -lgcov -fprofile-arcs -ftest-coverage -O0
LDFLAGS      += -L$(LIBDIR)

.SUFFIXES:
.SUFFIXES: .o .otest .c .cxx .cxxtest .test

all: $(TESTBINS)

-include $(OBJS:.o=.d) $(TESTOBJS:.otest=.dtest)

$(TESTBINS): $(OBJS) $(TESTOBJS) $(FULLPATHLIBS) $(TESTOBJSEXTRA)
	$(LD) -o $@ $*.otest $(TESTOBJSEXTRA) \
	$(filter $(@:.test=.o),$(OBJS)) $(OBJSEXTRA) \
	$(LDFLAGS) $(LIBS) $(SYSLIBRARIES)

$(TESTOUTPUTS): %.testout : %.test
	(cd lcovdir; ../$< --gtest_death_test_style=threadsafe && touch $@)

gtest-all.o : %.o : $(GTESTSRCPATH)/src/%.cc
	$(CXX) $(CXXFLAGS) -I$(GTESTPATH) -I$(GTESTSRCPATH)  $< -o $@
	$(CXX) -MM $(CXXFLAGS) -I$(GTESTPATH) -I$(GTESTSRCPATH) $< > $*.d

gmock-all.o : %.o : $(GMOCKSRCPATH)/src/%.cc
	$(CXX) $(CXXFLAGS) -I$(GMOCKPATH) -I$(GMOCKSRCPATH)  $< -o $@
	$(CXX) -MM $(CXXFLAGS) -I$(GMOCKPATH) -I$(GMOCKSRCPATH) $< > $*.d

.cxx.o:
	$(CXX) $(CXXFLAGS) $< -o $@

.c.o:
	$(CC) $(CFLAGS) $< -o $@

.cxxtest.otest:
	$(CXX) $(CXXFLAGS) -MF $*.dtest -x c++ $< -o $@
#	ln -s $@.gcno $*.gcno

#.otest.test:
#	$(LD) -o $@ $< $(GTESTPATH)/src/gtest-all.o \
#	$(GTESTPATH)/src/gtest_main.o $(filter $(<:.otest=.o),$(OBJS)) \
#	$(LDFLAGS) $(SYSLIBRARIES)

tests: all
	echo $(foreach TESTBINS,$(TESTBINS),gcov $(VPATH)$(TESTBINS:.test=.cxxtest))
	[ -z "$(TESTBINS)" ] || (mkdir -p lcovdir; cd lcovdir; \
	lcov --directory ../ --no-recursion -z; cd .. ; \
	$(MAKE) run-tests ; cd lcovdir ; \
	lcov --directory ../ --no-recursion --capture --output-file app.info; \
	lcov -r app.info "/usr/include/*" -o app.info; \
	lcov -r app.info "*.cxxtest" -o app.info; \
	lcov -r app.info "*gtest*" -o app.info; \
	genhtml -o . app.info )

run-tests: $(TESTOUTPUTS)

clean: clean-local

clean-local:
	rm -rf *.o *.otest *.d *.dtest *.test *.gcda *.gcno *.png *.info bits ext home i686-linux-gnu opt usr *.html *.css lcovdir *.map

veryclean: clean-local

#nothing to do
mksubdirs:

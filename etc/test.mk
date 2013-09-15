ifeq ($(BASENAME),)
# if the basename is so far undefined
BASENAME := $(shell basename `pwd`)
endif

include $(OPENMRNPATH)/etc/linux.x86.mk

include $(OPENMRNPATH)/etc/path.mk

VPATH = $(OPENMRNPATH)/src/$(BASENAME)/

FULLPATHCSRCS        = $(wildcard $(VPATH)*.c)
FULLPATHCXXSRCS      = $(wildcard $(VPATH)*.cxx)
FULLPATHCXXTESTSRCS  = $(wildcard $(VPATH)*.cxxtest)

CSRCS       = $(notdir $(FULLPATHCSRCS))       $(wildcard *.c)
CXXSRCS     = $(notdir $(FULLPATHCXXSRCS))     $(wildcard *.cxx)
CXXTESTSRCS = $(notdir $(FULLPATHCXXTESTSRCS)) $(wildcard *.cxxtest)

OBJS = $(CXXSRCS:.cxx=.o) $(CSRCS:.c=.o)
TESTOBJS = $(CXXTESTSRCS:.cxxtest=.otest)

TESTOUTPUTS = $(CXXTESTSRCS:.cxxtest=.test)

INCLUDES     += -I$(GTESTPATH)/include -I$(OPENMRNPATH)/src -I$(OPENMRNPATH)/include
CFLAGS       += -DGTEST $(INCLUDES) -Wno-unused-but-set-variable -fprofile-arcs -ftest-coverage -O0
CXXFLAGS     += -DGTEST $(INCLUDES) -Wno-unused-but-set-variable -fprofile-arcs -ftest-coverage -O0
SYSLIBRARIES += -lgcov -fprofile-arcs -ftest-coverage -O0

.SUFFIXES:
.SUFFIXES: .o .otest .c .cxx .cxxtest .test

all: $(TESTOUTPUTS)

$(TESTOUTPUTS): $(OBJS) $(TESTOBJS)
	$(LD) -o $@ $*.otest $(GTESTPATH)/src/gtest-all.o \
	$(GTESTPATH)/src/gtest_main.o $(filter $(@:.test=.o),$(OBJS)) $(OBJSEXTRA) \
	$(LDFLAGS) $(SYSLIBRARIES)

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
	echo $(foreach TESTOUTPUTS,$(TESTOUTPUTS),gcov $(VPATH)$(TESTOUTPUTS:.test=.cxxtest))
	mkdir -p lcovdir; cd lcovdir; \
	lcov --directory ../ --no-recursion -z; \
	$(foreach TESTOUTPUTS,$(TESTOUTPUTS),../$(TESTOUTPUTS);) \
	lcov --directory ../ --no-recursion --capture --output-file app.info; \
	lcov -r app.info "/usr/include/*" -o app.info; \
	lcov -r app.info "*.cxxtest" -o app.info; \
	lcov -r app.info "*gtest*" -o app.info; \
	genhtml -o . app.info

clean: clean-local

clean-local:
	rm -rf *.o *.otest *.d *.dtest *.test *.gcda *.gcno *.png *.info bits ext home i686-linux-gnu opt usr *.html *.css lcovdir

veryclean: clean-local


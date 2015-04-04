include ../../etc/core_target.mk

#include $(OPENMRNPATH)/etc/core_target.mk

SRCDIR = $(OPENMRNPATH)/src

CUSTOM_EXEC=1

include $(OPENMRNPATH)/etc/core_test.mk

TESTDIRS = $(TESTSRCS:.cxxtest=.covdir)

# This target actually runs the test. We jump through some hoops to collect the
# coverage files into a separate directory. Since they are in a separate directory, we need to put the original .gcno files there as well.
%.testout : %.testmd5
	mkdir -p $*.covdir
	rm -f $*.covdir/app.info
	lcov --directory $*.covdir/ -z
	GCOV_PREFIX=$(abspath $*.covdir) GCOV_PREFIX_STRIP=$(shell echo $(realpath .) | tr -C -d '/' | wc -c) $(<:.testmd5=.test$(EXTENTION)) --gtest_death_test_style=threadsafe
	touch $@


%.covdir/app.info : %.testout
	cd $*.covdir/ ; for i in $$(find . -name \*.gcda) ; do cp --remove-destination $(realpath .$)/$${i%%.gcda}.gcno $${i%%.gcda}.gcno ; done && lcov --directory . --capture --output-file app.info


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

veryclean-cov:
	rm -rf lcovdir/

veryclean: veryclean-cov clean-cov

clean-cov:
	rm -rf $(TESTDIRS) $(COVAGGRFILES)

clean veryclean: clean-cov clean-gtest

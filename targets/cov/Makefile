OPENMRNPATH ?= $(abspath ../..)
include $(OPENMRNPATH)/etc/core_target.mk

SRCDIR = $(OPENMRNPATH)/src

CUSTOM_EXEC=1

include $(OPENMRNPATH)/etc/core_test.mk

TESTDIRS = $(TESTSRCS:.cxxtest=.covdir)

utils/OpenSSLAesCcm.test: SYSLIBRARIESEXTRA+=-lcrypto

# This target actually runs the test. We jump through some hoops to collect the
# coverage files into a separate directory. Since they are in a separate directory, we need to put the original .gcno files there as well.
%.testout : %.testmd5
	mkdir -p $*.covdir
	rm -f $*.covdir/app.info
	lcov --rc branch_coverage=1 --directory $*.covdir/ -z
	GCOV_PREFIX=$(abspath $*.covdir) GCOV_PREFIX_STRIP=$(shell echo $(realpath .) | tr -C -d '/' | wc -c) $(<:.testmd5=.test$(EXTENTION)) $(TESTARGS) --gtest_death_test_style=threadsafe
	touch $@


%.covdir/app.info : %.testout
	cd $*.covdir/ ; \
	for i in $$(find . -name \*.gcda) ; do \
	  echo i $$i base $${i%%.gcda} ; \
	  ln -sf $(realpath .)/$${i%%.gcda}.gcno $${i%%.gcda}.gcno ; \
	done ; \
	lcov --rc branch_coverage=1 --ignore-errors mismatch --directory . --capture --output-file app.info ; \
	rm -f $$(find . -name \*.gcno -type l)


COVAGGRDIRS = $(dir $(TESTBINS))
COVAGGRDIRS_UNIQ = $(shell echo $(COVAGGRDIRS) | tr ' ' '\n' | sort | uniq)
COVAGGRFILES_DUP = $(foreach DIR,$(dir $(TESTBINS)),$(DIR)cov.info)
COVAGGRFILES = $(shell echo $(COVAGGRFILES_DUP) | tr ' ' '\n' | sort | uniq)

# Helper function for aggregating coverage files into per-directory templates.
#
# arg 1: a .testout file
#
# Adds the per-test coverage.info file to the dependencies to the directory's
# coverage file.
define TEST_TO_DIR_COV_template
$(dir $(1))cov.info: $(1:.testout=.covdir/app.info)

$(dir $(1))tests: $(1)
endef

# Calls the directory template for every .testout file we have.
$(foreach testout,$(TESTOUTPUTS),$(eval $(call TEST_TO_DIR_COV_template,$(testout))))

$(COVAGGRFILES):
	rm -f $@
	lcov --rc branch_coverage=1 --ignore-errors mismatch --ignore-errors empty --output-file $@ $(foreach F,$^, -a $F)

cov: lcovdir/index.html

lcovdir/index.html: $(TESTOUTPUTS) $(TESTMD5) $(COVAGGRFILES)
	mkdir -p lcovdir ; \
	rm -f lcovdir/app_base.info; \
	lcov --rc branch_coverage=1 --ignore-errors mismatch -c -i -d . -o lcovdir/app_base.info; \
	cd lcovdir ; rm -f app.info; \
	lcov --rc branch_coverage=1 --ignore-errors mismatch -o app.info $(foreach F,$(COVAGGRFILES), -a ../$F) ; \
	lcov --rc branch_coverage=1 -r app.info "/usr/include/*" "*.cxxtest" "*gtest*" "*gmock*" -o app.info; \
	lcov --rc branch_coverage=1 -r app_base.info "/usr/include/*" "*.cxxtest" "*gtest*" "*gmock*" -o app_base.info; \
	lcov --rc branch_coverage=1 -a app_base.info -a app.info -o app_total.info; \
	genhtml --branch-coverage --ignore-errors unmapped -o . app_total.info

# Helper function to create per-direcotry coverage summaries.
#
# arg 1: a directory name (basename, without anything, on trailing slash)
define PER_DIR_COV_template
$(1)cov: $(1)lcovdir/index.html

$(1)lcovdir/index.html: $(1)cov.info
	cd $(1) ; \
	mkdir -p lcovdir ; \
	cd lcovdir ; \
	lcov --rc branch_coverage=1 -r ../../$$< "/usr/include/*" "*.cxxtest" "*gtest*" "*gmock*" -o app.info ; \
	genhtml --branch-coverage --ignore-errors unmapped -o . app.info
endef

$(foreach dirname,$(COVAGGRDIRS_UNIQ),$(eval $(call PER_DIR_COV_template,$(dirname))))

veryclean-cov:
	rm -rf lcovdir/
	rm -rf $(foreach dirname,$(COVAGGRDIRS_UNIQ),$(dirname)lcovdir/)


veryclean: veryclean-cov clean-cov

clean-cov:
	rm -rf $(TESTDIRS) $(COVAGGRFILES)

clean veryclean: clean-cov clean-gtest

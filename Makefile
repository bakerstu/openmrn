# Ensures all recursive directories get OPENMRNPATH appropriately.
OPENMRNPATH:=$(realpath .)
export OPENMRNPATH

$(shell touch $(OPENMRNPATH)/build_timestamp)
export HAVE_BUILD_TIMESTAMP=$(OPENMRNPATH)/build_timestamp

SUBDIRS = targets doc applications #test

include $(OPENMRNPATH)/etc/recurse.mk

# Makes sure all the targets are compiled before building the test application.
build-test: build-targets

tests-test: build-targets

build-applications: build-targets

tests-applications: build-targets

.PHONY: docs cov
docs:
	$(MAKE) -C doc docs || exit 1;

docsw:
	cat $(OPENMRNPATH)/doc/warnings

cov:
	$(MAKE) -C targets/cov cov

tests:
	$(MAKE) -C targets/cov tests

llvm-tests:
	$(MAKE) -C targets/linux.llvm run-tests

js-tests:
	$(MAKE) -C targets/js.emscripten run-tests

alltests: tests llvm-tests

# Ensures all recursive directories get OPENMRNPATH appropriately.
OPENMRNPATH:=$(realpath .)
export OPENMRNPATH

SUBDIRS = targets doc applications

include $(OPENMRNPATH)/etc/recurse.mk

# Makes sure all the targets are compiled before building the test application.
build-test: build-targets

tests-test: build-targets

build-applications: build-targets

tests-applications: build-targets

.PHONY: docs cov
docs:
	$(MAKE) -C doc docs || exit 1;

cov:
	$(MAKE) -C targets/cov cov

tests:
	$(MAKE) -C targets/cov tests

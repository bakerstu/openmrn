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

release-clean:
	$(MAKE) -C targets/linux.x86 clean

RELNAME=$(shell uname -sm | tr ' A-Z' '.a-z')
RELDIR=$(OPENMRNPATH)/bin/release/staging-$(RELNAME)

include $(OPENMRNPATH)/etc/release.mk

# These are the applications that are packaged into the binary release.
$(call RELEASE_BIN_template,hub,applications/hub/targets/linux.x86)
$(call RELEASE_BIN_template,memconfig_utils,applications/memconfig_utils/targets/linux.x86)
$(call RELEASE_BIN_template,bootloader_client,applications/bootloader_client/targets/linux.x86)
$(call RELEASE_BIN_template,send_datagram,applications/send_datagram/targets/linux.x86)

release-bin:
	rm -rf $(RELDIR)/*
	mkdir -p $(RELDIR)
	+$(MAKE) -C . release-bin-all
	cd $(RELDIR); zip -9r ../release-$(RELNAME).zip .

# Helper targets for recusing into subdirectories. The caller has to define the
# variable SUBDIRS, then include recurse.mk. THis file defines targets all,
# clean and veryclean, with them recursing into all the subdirectories in
# parallel.
#
# To ensure one directory gets built before another, do this:
# #the utils need the libraries in dev built first
# build-utils: build-dev

ifeq ($(SUBDIRS),)
$(error Must define SUBDIRS for using recurse.mk)
endif

# Fake targets for recursing
BUILDDIRS = $(SUBDIRS:%=build-%)
CLEANDIRS = $(SUBDIRS:%=clean-%)
VERYCLEANDIRS = $(SUBDIRS:%=veryclean-%)
TESTDIRS = $(SUBDIRS:%=tests-%)
MKSUBDIRSDIRS = $(SUBDIRS:%=mksubdirs-%)

all: $(BUILDDIRS)
$(SUBDIRS): $(BUILDDIRS)
$(BUILDDIRS):
	@[ -d $(@:build-%=%) ] || make mksubdirs
	+$(MAKE) -C $(@:build-%=%) all

clean: $(CLEANDIRS)
$(CLEANDIRS): 
	@[ -d $(@:clean-%=%) ] || make mksubdirs
	+$(MAKE) -C $(@:clean-%=%) clean

veryclean: $(VERYCLEANDIRS)
$(VERYCLEANDIRS): 
	@[ -d $(@:veryclean-%=%) ] || make mksubdirs
	+$(MAKE) -C $(@:veryclean-%=%) veryclean

tests: $(TESTDIRS)
$(TESTDIRS): 
	@[ -d $(@:tests-%=%) ] || make mksubdirs
	+$(MAKE) -C $(@:tests-%=%) tests

mksubdirs: $(MKSUBDIRSDIRS)

# Some strategies on how to create nonexistant directories.

ifdef MKSUBDIR_OPENMRNINCLUDE
HAVE_MKSUBDIR=1
$(MKSUBDIRSDIRS):
	if [ ! -d $(@:mksubdirs-%=%) ] ; then mkdir $(@:mksubdirs-%=%) ; echo 'include $$(OPENMRNPATH)/etc/$(MKSUBDIR_OPENMRNINCLUDE)' > $(@:mksubdirs-%=%)/Makefile ; fi
	+$(MAKE) -C $(@:mksubdirs-%=%) mksubdirs
endif

# Fallback in case we didn't have a subdiretory create strategy.
ifndef HAVE_MKSUBDIR
$(MKSUBDIRSDIRS):
	$(MAKE) -C $(@:mksubdirs-%=%) mksubdirs
endif



.PHONY: subdirs $(SUBDIRS)
.PHONY: subdirs $(BUILDDIRS)
.PHONY: subdirs $(CLEANDIRS)
.PHONY: subdirs $(VERYCLEANDIRS)
.PHONY: subdirs $(TESTDIRS)
.PHONY: all install clean veryclean tests

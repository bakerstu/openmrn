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

all: $(BUILDDIRS)
$(SUBDIRS): $(BUILDDIRS)
$(BUILDDIRS):
	+$(MAKE) -C $(@:build-%=%) all

clean: $(CLEANDIRS)
$(CLEANDIRS): 
	+$(MAKE) -C $(@:clean-%=%) clean

veryclean: $(VERYCLEANDIRS)
$(VERYCLEANDIRS): 
	+$(MAKE) -C $(@:veryclean-%=%) veryclean

tests: $(TESTDIRS)
$(TESTDIRS): 
	+$(MAKE) -C $(@:tests-%=%) tests


.PHONY: subdirs $(SUBDIRS)
.PHONY: subdirs $(BUILDDIRS)
.PHONY: subdirs $(CLEANDIRS)
.PHONY: subdirs $(VERYCLEANDIRS)
.PHONY: subdirs $(TESTDIRS)
.PHONY: all install clean veryclean test

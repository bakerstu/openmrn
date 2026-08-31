-include ../../openmrnpath.mk
ifndef OPENMRNPATH
OPENMRNPATH := $(realpath ../..)
endif
export OPENMRNPATH

SUBDIRS = targets
-include config.mk
include $(OPENMRNPATH)/etc/recurse.mk

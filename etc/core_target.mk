ifeq ($(strip $(TARGET)),)
TARGET := $(notdir $(CURDIR))
endif
ifndef OPENMRNPATH
OPENMRNPATH:=$(realpath ../..)
endif
export OPENMRNPATH

#GRABDIRLOCK=$(shell $(OPENMRNPATH)/etc/dirguard.sh )
#$(info dirlock $(GRABDIRLOCK))

include $(OPENMRNPATH)/etc/config.mk
include $(OPENMRNPATH)/etc/path.mk
include $(OPENMRNPATH)/etc/$(TARGET).mk

SUBDIRS = $(CORELIBS) $(SYSLIB_SUBDIRS)

# This defines how to create nonexistant directories.
MKSUBDIR_OPENMRNINCLUDE=lib.mk

include $(OPENMRNPATH)/etc/recurse.mk

# lib/timestamp is a dependency to all test binaries. This rule has to have a
# body to execute, or else make will take the mtime of this file too early in
# the execution process. The body shall not actually touch the file, as the
# touches happen in the BUILDDIRS recursions. This workaround ensures that when
# a source file changes in openmrn, the tests are all re-linked and
# re-run. Otherwise you need to run tests twice to actually execute them.
lib/timestamp: $(BUILDDIRS)
	true

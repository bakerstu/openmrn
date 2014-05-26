ifeq ($(strip $(TARGET)),)
TARGET := $(shell basename `pwd`)
endif
ifndef OPENMRNPATH
OPENMRNPATH:=$(realpath ../..)
endif
export OPENMRNPATH

include $(OPENMRNPATH)/etc/config.mk

include $(OPENMRNPATH)/etc/$(TARGET).mk

# lib here is only needed for clean to work properly. Libraries are copied
# there by the original build rules.
SUBDIRS = $(CORELIBS) $(SYSLIB_SUBDIRS) lib

# This defines how to create nonexistant directories.
MKSUBDIR_OPENMRNINCLUDE=lib.mk

include $(OPENMRNPATH)/etc/recurse.mk

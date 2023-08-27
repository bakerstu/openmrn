OPENMRNPATH ?= $(realpath ../../)

export SKIP_COVERAGE := 1
export TARGET:=linux.x86
export HOST_TARGET := 1
SRCDIR = $(OPENMRNPATH)/src

#TARGET := linux.x86
#

include $(OPENMRNPATH)/etc/core_target.mk
include $(OPENMRNPATH)/etc/core_test.mk
CXXFLAGSEXTRA+="-Wno-maybe-uninitialized "



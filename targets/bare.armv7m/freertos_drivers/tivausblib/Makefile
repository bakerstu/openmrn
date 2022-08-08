OPENMRNPATH ?= $(realpath ../../../..)

DEPS  += TIVAWAREPATH
include $(OPENMRNPATH)/etc/tivaware.mk
VPATH := $(TIVAWAREPATH)/usblib

include $(OPENMRNPATH)/etc/lib.mk
CFLAGS += -Dcodered

SUBDIRS = device host
include $(OPENMRNPATH)/etc/recurse.mk


OPENMRNPATH ?= $(realpath ../../../..)

DEPS  += TIVAWAREPATH
include $(OPENMRNPATH)/etc/tivaware.mk
VPATH := $(TIVAWAREPATH)/usblib
CFLAGS += -Dcodered

include $(OPENMRNPATH)/etc/lib.mk

SUBDIRS = device host
include $(OPENMRNPATH)/etc/recurse.mk


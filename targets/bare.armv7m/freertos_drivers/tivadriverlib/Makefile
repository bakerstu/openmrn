OPENMRNPATH ?= $(realpath ../../../..)

DEPS += TIVAWAREPATH
include $(OPENMRNPATH)/etc/tivaware.mk
VPATH := $(TIVAWAREPATH)/driverlib

include $(OPENMRNPATH)/etc/lib.mk

CFLAGS += -Dgcc

emac.o : CFLAGS += -Wno-address-of-packed-member

OPENMRNPATH ?= $(realpath ../../../..)

DEPS += TIVAWAREPATH
include $(OPENMRNPATH)/etc/tivaware.mk
VPATH := $(TIVAWAREPATH)/driverlib
CFLAGS += -Dcodered

include $(OPENMRNPATH)/etc/lib.mk

emac.o : CFLAGS += -Wno-address-of-packed-member

OPENMRNPATH ?= $(realpath ../../../..)

DEPS += TIVAWAREPATH
include $(OPENMRNPATH)/etc/tivaware.mk
VPATH := $(TIVAWAREPATH)/grlib
CFLAGS += -Dgcc

FULLPATHCSRCS = $(wildcard $(VPATH)/*.c) $(wildcard $(VPATH)/fonts/*.c)

include $(OPENMRNPATH)/etc/lib.mk


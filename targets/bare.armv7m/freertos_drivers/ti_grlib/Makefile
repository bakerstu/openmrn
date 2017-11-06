OPENMRNPATH ?= $(realpath ../../../..)

DEPS += TIVAWAREPATH
INCLUDES += -I$(TIVAWAREPATH) -Dgcc
VPATH := $(TIVAWAREPATH)/grlib

FULLPATHCSRCS = $(wildcard $(VPATH)/*.c) $(wildcard $(VPATH)/fonts/*.c)

include $(OPENMRNPATH)/etc/lib.mk


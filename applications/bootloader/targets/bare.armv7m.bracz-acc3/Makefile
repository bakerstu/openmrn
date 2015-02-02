export BOARD := BOARD_BRACZ_CS_TIVA
APP_PATH ?= $(realpath ../..)
-include $(APP_PATH)/config.mk
-include $(OPENMRNPATH)/etc/config.mk
LINKCORELIBS = -lnmranet -lutils

TARGET := bare.armv7m
include hardware.mk

SYSLIBRARIES=
SYSLIB_SUBDIRS=
OBJEXTRA=
LDFLAGSEXTRA+= --specs=nano.specs
ifdef ADDRESS
CXXFLAGS+= -DNODEID_LOW_BITS=$(ADDRESS)
else
$(error Must define address with ADDRESS=0x4c)
endif

# This will force re-building NodeId.cxx every invocation, because the address
# might have changed.
NodeId.o: FORCE

FORCE:


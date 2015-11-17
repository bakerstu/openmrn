export BOARD := BOARD_BRACZ_RAILCOM
APP_PATH ?= $(realpath ../..)
-include $(APP_PATH)/config.mk
-include $(OPENMRNPATH)/etc/config.mk
LINKCORELIBS = -lnmranet -lutils

TARGET := bare.armv7m

ifndef ADDRESS
ADDRESS=0x62
$(warning Using default address 0x62 override with ADDRESS=0x3f)
endif

include hardware.mk

ARCHOPTIMIZATION += -Os


SYSLIBRARIES=
SYSLIB_SUBDIRS=
OBJEXTRA=
LDFLAGSEXTRA+= --specs=nano.specs

CXXFLAGS+= -DNODEID_LOW_BITS=$(ADDRESS)

# This will force re-building NodeId.cxx every invocation, because the address
# might have changed.
NodeId.o: FORCE

FORCE:


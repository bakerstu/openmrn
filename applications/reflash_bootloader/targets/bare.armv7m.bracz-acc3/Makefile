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
CXXFLAGS+= -DNODEID_LOW_BITS=0x04
$(warning Using default address 0x04 override with ADDRESS=0x4c)
endif

# This will force re-building NodeId.cxx every invocation, because the address
# might have changed.
NodeId.o: FORCE

payload.cxx: FORCE
	$(MAKE) -C $(realpath bootloader_bin) bootloader.bin
	$(OPENMRNPATH)/bin/build_bootloader_img.py -i bootloader_bin/bootloader.bin -o $@

FORCE:


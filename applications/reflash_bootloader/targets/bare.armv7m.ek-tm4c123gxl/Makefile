export BOARD := BOARD_EK_TM4C123
APP_PATH ?= $(realpath ../..)
-include $(APP_PATH)/config.mk
-include $(OPENMRNPATH)/etc/config.mk
LINKCORELIBS = -lopenlcb -lutils

TARGET := bare.armv7m
include hardware.mk
include $(OPENMRNPATH)/etc/node_id.mk

ARCHOPTIMIZATION += -Os

SYSLIBRARIES=
SYSLIB_SUBDIRS=
OBJEXTRA=
LDFLAGSEXTRA+= --specs=nano.specs


bootloader_bin/bootloader.bin: FORCE
	$(MAKE) -C $(realpath bootloader_bin) bootloader.bin

payload.cxxout: bootloader_bin/bootloader.bin $(OPENMRNPATH)/bin/build_bootloader_img.py
	$(OPENMRNPATH)/bin/build_bootloader_img.py -i bootloader_bin/bootloader.bin -o $@

.cxxout.o:
	$(CXX) $(CXXFLAGS) -x c++ -MD -MF $*.d $(abspath $<) -o $@

OBJS += payload.o

$(EXECUTABLE)$(EXTENTION): payload.o

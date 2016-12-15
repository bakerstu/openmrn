export BOARD := BOARD_EK_TM4C123
APP_PATH ?= $(realpath ../..)
-include $(APP_PATH)/config.mk
-include $(OPENMRNPATH)/etc/config.mk
LINKCORELIBS = -lnmranet -lutils

TARGET := bare.armv7m
include hardware.mk
include $(OPENMRNPATH)/etc/node_id.mk

SYSLIBRARIES=
SYSLIB_SUBDIRS=
OBJEXTRA=
LDFLAGSEXTRA+= --specs=nano.specs

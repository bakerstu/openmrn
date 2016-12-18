export BOARD := BOARD_BRACZ_CS_TIVA
APP_PATH ?= $(realpath ../..)
-include $(APP_PATH)/config.mk
-include $(OPENMRNPATH)/etc/config.mk
LINKCORELIBS = -lopenlcb -lutils

TARGET := bare.armv7m
include hardware.mk

SYSLIBRARIES=
SYSLIB_SUBDIRS=
OBJEXTRA=
LDFLAGSEXTRA+= --specs=nano.specs

DEFAULT_ADDRESS=0x4c
include $(OPENMRNPATH)/etc/node_id.mk

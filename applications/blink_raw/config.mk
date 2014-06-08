BACKUP_OPENMRN := $(realpath $(dir $(lastword $(MAKEFILE_LIST)))/../..)/
-include $(APP_PATH)/openmrnpath.mk
OPENMRNPATH ?= $(BACKUP_OPENMRN)
export OPENMRNPATH

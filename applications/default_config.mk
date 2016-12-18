ifndef APP_PATH
APP_PATH := $(realpath $(dir $(lastword $(MAKEFILE_LIST))))
endif
export APP_PATH

-include $(APP_PATH)/openmrnpath.mk
ifndef OPENMRNPATH
OPENMRNPATH := $(realpath $(APP_PATH)/../..)
endif
export OPENMRNPATH

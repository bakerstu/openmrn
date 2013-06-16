# This makefile contains the lookup path for all the external dependencies to
# toolchains and libraries.


################# helper functions ##############

# Finds the first occurence of a file under a search path. Returns the first
# entry in the directories list that has file under it, or empty string if none
# found.
# $(1): file or dir to look for, $(2); list of paths
findfirst=$(firstword $(foreach dir,$(2),$(if $(wildcard $(dir)/$(1)),$(wildcard $(dir)))))

################# mbed library ##################

ifndef MBEDPATH
SEARCHPATH := \
  $(HOME)/lpc-workspace/libmbed_2387/mbed \
  /opt/mbed/default/libraries \


TRYPATH:=$(call findfirst,USBDevice,$(SEARCHPATH))
ifneq ($(TRYPATH),)
MBEDPATH:=$(TRYPATH)
endif
endif #ifndef MBED_PATH

################### FreeRTOS ####################
ifndef FREERTOSPATH
SEARCHPATH := \
  /opt/FreeRTOS \
  /opt/FreeRTOS/default \
  $(HOME)/FreeRTOS \

TRYPATH:=$(call findfirst,Source,$(SEARCHPATH))
ifneq ($(TRYPATH),)
FREERTOSPATH:=$(TRYPATH)
endif
endif #FREERTOSPATH

################# lpcxpresso ####################
ifndef LPCXPRESSOPATH
SEARCHPATH := \
  /usr/local/lpcxpresso_*/lpcxpresso \

TRYPATH:=$(call findfirst,tools/bin,$(SEARCHPATH))
ifneq ($(TRYPATH),)
LPCXPRESSOPATH:=$(TRYPATH)
endif
endif #LPCXPRESSOPATH

################## checksum #####################
ifndef CHECKSUM
SEARCHPATH := \
  $(LPCXPRESSOPATH)/bin \

TRYPATH:=$(call findfirst,checksum,$(SEARCHPATH))
ifneq ($(TRYPATH),)
CHECKSUM:=$(TRYPATH)/checksum
endif
endif #CHECKSUM

ifndef CHECKSUM
CHECKSUM:=@echo No CHECKSUM binary available - skipping writing header checksum. It is possible that the MCU will drop into bootloader when using this binary. \#
endif

################### ARM-GCC #####################
ifndef ARMGCCPATH
SEARCHPATH := \
  /opt/armgcc/default \

TRYPATH:=$(call findfirst,bin/arm-none-eabi-g++,$(SEARCHPATH))
ifneq ($(TRYPATH),)
ARMGCCPATH:=$(TRYPATH)
endif
endif #ARMGCCPATH


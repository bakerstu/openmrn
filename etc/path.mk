# This makefile contains the lookup path for all the external dependencies to
# toolchains and libraries.


################# helper functions ##############

# Finds the first occurence of a file under a search path. Returns the first
# entry in the directories list that has file under it, or empty string if none
# found.
# $(1): file or dir to look for, $(2); list of paths
findfirst=$(firstword $(foreach dir,$(2),$(if $(wildcard $(dir)/$(1)),$(wildcard $(dir)))))

# Finds missing dependencies from a list.
#
# Accepts as $(1) a list of variable NAMEs. Returns a string containing the
# name of those variables that have no or empty value. Returns an empty string
# if all dependencies are met.
#
# Usage:
# DEPS += TOOLPATH FREERTOSPATH
# MISSING_DEPS:=$(call find_missing_deps,$(DEPS))
# ifneq (,$(MISSING_DEPS))
# all:
# 	@echo missing dependencies: $(MISSING_DEPS)
# else
# all: build-deps
# endif
find_missing_deps=$(strip $(foreach depvar,$(1),$(if $(value $(depvar)),,$(depvar))))

################ tivaware ##################
ifndef TIVAWAREPATH
SEARCHPATH := \
  /opt/ti/TivaWare/default \
  /opt/TivaWare/default \
  /opt/TivaWare \
  $(HOME)/TivaWare

TRYPATH:=$(call findfirst,driverlib,$(SEARCHPATH))
ifneq ($(TRYPATH),)
TIVAWAREPATH:=$(TRYPATH)
endif
endif #TIVAWAREPATH

################ lpcopen_18xx_43xx ##################
ifndef LPCOPENPATH_18XX_43XX
SEARCHPATH := \
  /opt/nxp/lpcopen_18xx_43xx/default 

TRYPATH:=$(call findfirst,driverlib,$(SEARCHPATH))
ifneq ($(TRYPATH),)
LPCOPENPATH_18XX_43XX:=$(TRYPATH)
endif
endif #LPCOPENPATH_18XX_43XX

################ nxpusblib ##################
ifndef NXPUSBLIBPATH
SEARCHPATH := \
  /opt/nxp/nxpUSBlib/default 

TRYPATH:=$(call findfirst,driverlib,$(SEARCHPATH))
ifneq ($(TRYPATH),)
NXPUSBLIBPATH:=$(TRYPATH)
endif
endif #NXPUSBLIBPATH

################# mbed library ##################

ifndef MBEDPATH
SEARCHPATH := \
  $(HOME)/lpc-workspace/libmbed_2387/mbed \
  $(HOME)/train/libmbed_2387/mbed \
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
  /opt/lpcxpresso/default/lpcxpresso/tools \
  /usr/local/lpcxpresso_*/lpcxpresso/tools \
  /opt/CodeSourcery/Sourcery_CodeBench_Lite_for_ARM_EABI \
  /opt/armgcc/default \

TRYPATH:=$(call findfirst,bin/arm-none-eabi-g++,$(SEARCHPATH))
ifneq ($(TRYPATH),)
ARMGCCPATH:=$(TRYPATH)
endif
endif #ARMGCCPATH

############### CMSIS-LPC11xx ###################
ifndef CMSIS_LPC11_PATH
SEARCHPATH := \
  $(HOME)/lpc-workspace/CMSISv2p00_LPC11xx \

TRYPATH:=$(call findfirst,inc/LPC11xx.h,$(SEARCHPATH))
ifneq ($(TRYPATH),)
CMSIS_LPC11_PATH:=$(TRYPATH)
endif
endif #CMSIS_LPC11_PATH


############### GMOCK ###################
ifndef GMOCKPATH
SEARCHPATH := \
  /opt/gmock/default \
  /usr \

TRYPATH:=$(call findfirst,include/gmock/gmock.h,$(SEARCHPATH))
ifneq ($(TRYPATH),)
GMOCKPATH:=$(TRYPATH)
endif
endif #GMOCKPATH

ifndef GMOCKSRCPATH
SEARCHPATH := \
  $(GMOCKPATH) \
  /usr/src/gmock \
  /opt/gmock/default \

TRYPATH:=$(call findfirst,src/gmock-all.cc,$(SEARCHPATH))
ifneq ($(TRYPATH),)
GMOCKSRCPATH:=$(TRYPATH)
endif
endif #GMOCKSRCPATH


############### GTEST ###################
ifndef GTESTPATH
SEARCHPATH := \
  $(GMOCKPATH)/gtest \
  /opt/gmock/default/gtest \
  /opt/gtest/gtest \
  /usr \

TRYPATH:=$(call findfirst,include/gtest/gtest.h,$(SEARCHPATH))
ifneq ($(TRYPATH),)
GTESTPATH:=$(TRYPATH)
endif
endif #GTESTPATH

ifndef GTESTSRCPATH
SEARCHPATH := \
  $(GTESTPATH) \
  /opt/gmock/default/gtest \
  /usr/src/gtest \
  /opt/gtest/default \

TRYPATH:=$(call findfirst,src/gtest-all.cc,$(SEARCHPATH))
ifneq ($(TRYPATH),)
GTESTSRCPATH:=$(TRYPATH)
endif
endif #GTESTSRCPATH

################### MIPS-ELF-GCC #####################
ifndef MIPSGCCPATH
SEARCHPATH := \
  /opt/CodeSourcery/Sourcery_CodeBench_Lite_for_MIPS_ELF \
  /opt/MentorGraphics/default_mips_elf
  

TRYPATH:=$(call findfirst,bin/mips-sde-elf-g++,$(SEARCHPATH))
ifneq ($(TRYPATH),)
MIPSGCCPATH:=$(TRYPATH)
endif
endif #MIPSGCCPATH

################### PIC32MXLIB #####################
ifndef PIC32MXLIBPATH
SEARCHPATH := \
  $(HOME)/train/git/pic32/includes \
  /opt/microchip/xc32/default/pic32mx/include

TRYPATH:=$(call findfirst,p32xxxx.h,$(SEARCHPATH))
ifneq ($(TRYPATH),)
PIC32MXLIBPATH:=$(TRYPATH)
endif
endif #PIC32MXLIBPATH


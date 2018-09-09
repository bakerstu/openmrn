# This makefile contains the lookup path for all the external dependencies to
# toolchains and libraries.

ifndef OPENMRN_PATH_MK
OPENMRN_PATH_MK:=1

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

ifeq ($(OS),Windows_NT)
include $(OPENMRNPATH)/etc/path_windows.mk
else

################ shell ##################
# Various commands in the makefiles are using the bash syntax. We ignore the
# user's login shell preferences and use a specific shell instead.
export SHELL :=/bin/bash

################ flock ##################
ifndef FLOCKPATH
SEARCHPATH := \
  /usr/bin \

TRYPATH:=$(call findfirst,flock,$(SEARCHPATH))
ifneq ($(TRYPATH),)
FLOCKPATH:=$(TRYPATH)
endif
endif #FLOCKPATH


ifndef OPENMRN_EXPLICIT_DEPS_ONLY

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
TRYPATH:=$(call findfirst,inc/hw_onewire.h,$(TRYPATH))
ifneq ($(TRYPATH),)
BUILDTIVAWARE:=$(TRYPATH)
endif
endif
endif #TIVAWAREPATH

################ STM32Cube_F0 ##################
ifndef STM32CUBEF0PATH
SEARCHPATH := \
  /opt/st/STM32Cube_FW_F0/default

TRYPATH:=$(call findfirst,Drivers,$(SEARCHPATH))
ifneq ($(TRYPATH),)
STM32CUBEF0PATH:=$(TRYPATH)
endif
endif #STM32CUBEF0PATH

################ STM32Cube_F1 ##################
ifndef STM32CUBEF1PATH
SEARCHPATH := \
  /opt/st/STM32Cube_FW_F1/default

TRYPATH:=$(call findfirst,Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h,$(SEARCHPATH))
ifneq ($(TRYPATH),)
STM32CUBEF1PATH:=$(TRYPATH)
endif
endif #STM32CUBEF1PATH

################ STM32Cube_F3 ##################
ifndef STM32CUBEF3PATH
SEARCHPATH := \
  /opt/st/STM32Cube_FW_F3/default

TRYPATH:=$(call findfirst,Drivers,$(SEARCHPATH))
ifneq ($(TRYPATH),)
STM32CUBEF3PATH:=$(TRYPATH)
endif
endif #STM32CUBEF3PATH

################ lpcopen_18xx_43xx ##################
ifndef LPCOPENPATH_18XX_43XX
SEARCHPATH := \
  /opt/nxp/lpcopen_18xx_43xx/default 

TRYPATH:=$(call findfirst,driverlib,$(SEARCHPATH))
ifneq ($(TRYPATH),)
LPCOPENPATH_18XX_43XX:=$(TRYPATH)
endif
endif #LPCOPENPATH_18XX_43XX

################ lpc_chip_17xx_40xx ##################
ifndef LPCCHIPPATH_17XX_40XX
SEARCHPATH := \
  /opt/nxp/lpc_chip/lpc_chip_17xx_40xx \
  /opt/nxp/lpc_chip/lpc_chip_175x_6x \

TRYPATH:=$(call findfirst,inc/can_17xx_40xx.h,$(SEARCHPATH))
ifneq ($(TRYPATH),)
LPCCHIPPATH_17XX_40XX:=$(TRYPATH)
endif
endif #LPCCHIPPATH_17XX_40XX

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

########### libmaple library source #############
ifndef LIBMAPLEPATH
SEARCHPATH := \
  $(HOME)/libmaple \
  /opt/libmaple/default \

TRYPATH:=$(call findfirst,support/ld/stm32/mem/sram_20k_flash_128k/mem-flash.inc,$(SEARCHPATH))
ifneq ($(TRYPATH),)
LIBMAPLEPATH:=$(TRYPATH)
endif
endif #ifndef LIBMAPLEPATH

######## STM32 peripheral library source ########
ifndef STM32PLIBPATH
SEARCHPATH := \
  /opt/st/stm32_plib/default \

TRYPATH:=$(call findfirst,Libraries/STM32F10x_StdPeriph_Driver/inc/stm32f10x_can.h,$(SEARCHPATH))
ifneq ($(TRYPATH),)
STM32PLIBPATH:=$(TRYPATH)
endif
endif #ifndef STM32PLIBPATH

################### FreeRTOS ####################
ifndef FREERTOSPATH
SEARCHPATH := \
  /opt/FreeRTOS \
  /opt/FreeRTOS/default \
  /opt/FreeRTOS/default/FreeRTOS \
  $(HOME)/FreeRTOS \
  /d/FreeRTOS/default

TRYPATH:=$(call findfirst,Source,$(SEARCHPATH))
ifneq ($(TRYPATH),)
FREERTOSPATH:=$(TRYPATH)
endif
endif #FREERTOSPATH

################### FreeRTOS+TCP ################
ifndef FREERTOSTCPPATH
SEARCHPATH := \
  /opt/FreeRTOSPlus/TCP \
  /opt/FreeRTOSPlus/default/TCP \
  /opt/FreeRTOS/plus-tcp \
  $(HOME)/FreeRTOSPlus/Source/FreeRTOS-Plus-TCP \
  /d/FreeRTOSPlus/default/TCP \

TRYPATH:=$(call findfirst,include/FreeRTOS_DNS.h,$(SEARCHPATH))
ifneq ($(TRYPATH),)
FREERTOSTCPPATH:=$(TRYPATH)
endif
endif #FREERTOSTCPPATH

################# lpcxpresso ####################
ifndef LPCXPRESSOPATH
SEARCHPATH := \
  /opt/lpcxpresso/default/lpcxpresso \
  /opt/lpcxpresso/lpcxpresso_*/lpcxpresso \
  /usr/local/lpcxpresso_*/lpcxpresso \

TRYPATH:=$(call findfirst,bin/LPCXpressoWIN.enc,$(SEARCHPATH))
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
  /opt/lpcxpresso/default/lpcxpresso/tools \
  /usr/local/lpcxpresso_*/lpcxpresso/tools \
  /opt/CodeSourcery/Sourcery_CodeBench_Lite_for_ARM_EABI \

TRYPATH:=$(call findfirst,bin/arm-none-eabi-g++,$(SEARCHPATH))
ifneq ($(TRYPATH),)
ARMGCCPATH:=$(TRYPATH)
endif
endif #ARMGCCPATH

################### TI-LINUX-SDK #####################
ifndef TILINUXSDKPATH
SEARCHPATH := \
  /opt/ti/ti-processor-sdk-linux/default \
  ~/ti-processor-sdk-linux-am335x-evm-03.00.00.04 \
  ~/ti-processor-sdk-linux-am335x-evm-02.00.01.07 \
  /opt/ti-processor-sdk-linux-am335x-evm-02.00.01.07 \
  /opt/ti/ti-processor-sdk-linux-am335x-evm-02.00.01.07 \



TRYPATH:=$(call findfirst,setup.sh,$(SEARCHPATH))
ifneq ($(TRYPATH),)
TILINUXSDKPATH:=$(TRYPATH)
endif
endif #TILINUXSDKPATH

################### ARM-LINUX GCC PATH #####################
ifndef ARMLINUXGCCPATH
SEARCHPATH := \
    $(TILINUXSDKPATH)/linux-devkit/sysroots/x86_64-arago-linux/usr/bin \
    /usr/bin \

TRYPATH:=$(call findfirst,arm-linux-gnueabihf-gcc,$(SEARCHPATH))
ifneq ($(TRYPATH),)
ARMLINUXGCCPATH:=$(TRYPATH)
endif
endif #ARMLINUXGCCPATH

################### TI-CC3200-SDK #####################
ifndef TICC3200SDKPATH
SEARCHPATH := \
  /opt/ti/CC3200SDK/default/cc3200-sdk


TRYPATH:=$(call findfirst,readme.txt,$(SEARCHPATH))
ifneq ($(TRYPATH),)
TICC3200SDKPATH:=$(TRYPATH)
endif
endif #TICC3200SDKPATH

################### TI-CC3220-SDK #####################
ifndef TICC3220SDKPATH
SEARCHPATH := \
  /opt/ti/CC3220SDK/default  \
  /opt/ti/CC3220SDK/simplelink_cc32xx_sdk_1_30_01_03 \


TRYPATH:=$(call findfirst,source/ti/devices/cc32xx/driverlib/gpio.c,$(SEARCHPATH))
ifneq ($(TRYPATH),)
TICC3220SDKPATH:=$(TRYPATH)
endif
endif #TICC3220SDKPATH

################### TI-UNIFLASH-V4 #####################
ifndef TIUNIFLASH4PATH
SEARCHPATH := \
  /opt/ti/uniflash/v4-default  \
  /opt/ti/uniflash/uniflash_4.1  \

TRYPATH:=$(call findfirst,dslite.sh,$(SEARCHPATH))
ifneq ($(TRYPATH),)
TIUNIFLASH4PATH:=$(TRYPATH)
endif
endif #TIUNIFLASH4PATH

################### PRU-ICSS #####################
ifndef PRUICSSPATH
SEARCHPATH := \
  $(TILINUXSDKPATH)/example-applications/pru-icss-4.0.2 \
  $(TILINUXSDKPATH)/example-applications/pru-icss-4.0.1

TRYPATH:=$(call findfirst,ReadMe.txt,$(SEARCHPATH))
ifneq ($(TRYPATH),)
PRUICSSPATH:=$(TRYPATH)
endif
endif #PRUICSSPATH

################### PRU-CGT #####################
ifndef PRUCGTPATH
SEARCHPATH := \
  /opt/ti/ccsv6/tools/compiler/pru \
  /opt/ti/ccsv6/tools/compiler/ti-cgt-pru_2.1.2 \
  /opt/ti/ccsv6/tools/compiler/ti-cgt-pru_2.1.1 \
  /opt/ti/ccsv6/tools/compiler/ti-cgt-pru_2.1.0 \
  $(TILINUXSDKPATH)/linux-devkit/sysroots/x86_64-arago-linux/usr/share/ti/cgt-pru

TRYPATH:=$(call findfirst,bin/clpru,$(SEARCHPATH))
ifneq ($(TRYPATH),)
PRUCGTPATH:=$(TRYPATH)
endif
endif #PRUCGTPATH

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
  /opt/gtest/default \
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
  /opt/CodeSourcery/default_mips_elf \
  /opt/CodeSourcery/Sourcery_CodeBench_Lite_for_MIPS_ELF \
  /opt/MentorGraphics/default_mips_elf \

# Master page for releases is https://sourcery.mentor.com/GNUToolchain/subscription3537?lite=MIPS
# To download latest release (2016-05) go here https://sourcery.mentor.com/GNUToolchain/release3215

TRYPATH:=$(call findfirst,bin/mips-sde-elf-g++,$(SEARCHPATH))
ifneq ($(TRYPATH),)
MIPSGCCPATH:=$(TRYPATH)
endif
endif #MIPSGCCPATH

################### MIPS-ELF-NEWLIB #####################
ifndef MIPSNEWLIBPATH
SEARCHPATH := \
  /opt/newlib/mips-sde-elf \
  $(MIPSGCCPATH)/mips-sde-elf

TRYPATH:=$(call findfirst,lib/el/mips16/sof/libc.a,$(SEARCHPATH))
ifneq ($(TRYPATH),)
MIPSNEWLIBPATH:=$(TRYPATH)
endif
endif #MIPSNEWLIBPATH

################### PIC32MXLIB #####################
ifndef PIC32MXLIBPATH
SEARCHPATH := \
  /opt/microchip/xc32/default \
  $(HOME)/train/git/pic32/includes \

TRYPATH:=$(call findfirst,pic32mx/include/p32xxxx.h,$(SEARCHPATH))
ifneq ($(TRYPATH),)
PIC32MXLIBPATH:=$(TRYPATH)
endif
endif #PIC32MXLIBPATH

################### PIC32MXLEGACYPLIB #####################
ifndef PIC32MXLEGACYPLIBPATH
SEARCHPATH := \
  $(PIC32MXLIBPATH) \
  /opt/microchip/pic32-plib/default \
  /opt/microchip/xc32/v1.32 \
  $(HOME)/train/git/pic32/includes \

TRYPATH:=$(call findfirst,pic32mx/include/peripheral/CAN.h,$(SEARCHPATH))
ifneq ($(TRYPATH),)
PIC32MXLEGACYPLIBPATH:=$(TRYPATH)
endif
endif #PIC32MXLEGACYPLIBPATH

################### PIC32HARMONY #####################
ifndef PIC32HARMONYPATH
SEARCHPATH := \
  /opt/microchip/harmony/default \

TRYPATH:=$(call findfirst,framework/usb/usb_cdc.h,$(SEARCHPATH))
ifneq ($(TRYPATH),)
PIC32HARMONYPATH:=$(TRYPATH)
endif
endif #PIC32HARMONYPATH

##################### OPENOCD ######################
ifndef OPENOCDPATH
SEARCHPATH := \
  /opt/openocd/default/openocd/src \
  /opt/openocd/default/bin \
  /usr/local/bin \
  /usr/bin \

TRYPATH:=$(call findfirst,openocd,$(SEARCHPATH))
ifneq ($(TRYPATH),)
OPENOCDPATH:=$(TRYPATH)
endif
endif #OPENOCDPATH

##################### OPENOCDSCRIPTS ######################
ifndef OPENOCDSCRIPTSPATH
SEARCHPATH := \
  /opt/openocd/default/openocd/tcl \
  /opt/openocd/default/tcl \
  /opt/openocd/default/scripts \
  /opt/openocd/default/share/openocd/scripts \
  /usr/local/share/openocd/scripts \
  /usr/share/openocd/scripts \


TRYPATH:=$(call findfirst,target/stellaris_icdi.cfg,$(SEARCHPATH))
ifneq ($(TRYPATH),)
OPENOCDSCRIPTSPATH:=$(TRYPATH)
endif
endif #OPENOCDSCRIPTSPATH

##################### EMSDK ######################
ifndef EMSDKPATH
SEARCHPATH := \
  /opt/emscripten/default/emscripten/master \
  /opt/emscripten/emsdk_portable/emscripten/master \
  $(wildcard /opt/emscripten/default/emsdk/emscripten/*) \
  /usr/bin


TRYPATH:=$(call findfirst,emcc,$(SEARCHPATH))
ifneq ($(TRYPATH),)
EMSDKPATH:=$(TRYPATH)
endif
endif #EMSDKPATH

##################### EMLLVM ######################
ifndef EMLLVMPATH
SEARCHPATH := \
  /opt/emscripten/default/clang/fastcomp/build_master_64/bin \
  /opt/emscripten/default/clang/fastcomp/build_master_32/bin \
  $(wildcard /opt/emscripten/default/emsdk/clang/*) \
  /usr/bin


TRYPATH:=$(call findfirst,llvm-ar,$(SEARCHPATH))
ifneq ($(TRYPATH),)
EMLLVMPATH:=$(TRYPATH)
endif
endif #EMLLVMPATH

##################### CLANGPPP ######################
ifndef CLANGPPPATH
SEARCHPATH := \
  /usr/bin


TRYPATH:=$(call findfirst,clang++,$(SEARCHPATH))
ifneq ($(TRYPATH),)
CLANGPPPATH:=$(TRYPATH)
endif
endif #CLANGPPPATH

##################### NODEJS ######################
ifndef NODEJSPATH
SEARCHPATH := \
  /usr/bin


TRYPATH:=$(call findfirst,nodejs,$(SEARCHPATH))
ifneq ($(TRYPATH),)
NODEJSPATH:=$(TRYPATH)
endif
endif #NODEJSPATH

##################### ESPOPENSDK ######################
ifndef ESPOPENSDKPATH
SEARCHPATH := \
  /opt/esp/patched/esp-open-sdk \
  /opt/esp/esp-open-sdk \


TRYPATH:=$(call findfirst,xtensa-lx106-elf/bin/xtensa-lx106-elf-gcc,$(SEARCHPATH))
ifneq ($(TRYPATH),)
ESPOPENSDKPATH:=$(TRYPATH)
endif
endif #ESPOPENSDKPATH

##################### ESPARDUINO ######################
ifndef ESPARDUINOPATH
SEARCHPATH := \
  $(HOME)/.arduino15/packages/esp8266 \


TRYPATH:=$(call findfirst,hardware/esp8266/2.2.0/cores/esp8266/core_esp8266_main.cpp,$(SEARCHPATH))
ifneq ($(TRYPATH),)
ESPARDUINOPATH:=$(TRYPATH)
endif
endif #ESPARDUINOPATH

##################### XTENSAGCC ######################
ifndef XTENSAGCCPATH
SEARCHPATH := \
  $(ESPOPENSDKPATH)/xtensa-lx106-elf \
  $(HOME)/.arduino15/packages/esp8266/tools/xtensa-lx106-elf-gcc/1.20.0-26-gb404fb9-2


TRYPATH:=$(call findfirst,bin/xtensa-lx106-elf-gcc,$(SEARCHPATH))
ifneq ($(TRYPATH),)
XTENSAGCCPATH:=$(TRYPATH)
endif
endif #XTENSAGCCPATH

##################### ESPTOOL ######################
ifndef ESPTOOLPATH
SEARCHPATH := \
  $(ESPOPENSDKPATH)/esptool \
  $(XTENSAGCCPATH)/bin \
  $(HOME)/prg/esp/esptool


TRYPATH:=$(call findfirst,esptool.py,$(SEARCHPATH))
ifneq ($(TRYPATH),)
ESPTOOLPATH:=$(TRYPATH)
endif
endif #ESPTOOLPATH

##################### ESPNONOSSDK ######################
ifndef ESPNONOSSDKPATH
SEARCHPATH := \
  /opt/esp/ESP8266_NONOS_SDK \
  $(HOME)/.arduino15/packages/esp8266/hardware/esp8266/2.2.0/tools/sdk \


TRYPATH:=$(call findfirst,include/ets_sys.h,$(SEARCHPATH))
ifneq ($(TRYPATH),)
ESPNONOSSDKPATH:=$(TRYPATH)
endif
endif #ESPNONOSSDKPATH


##################### ESPRTOSSDK ######################
ifndef ESPRTOSSDKPATH
SEARCHPATH := \
  /opt/esp/ESP8266_RTOS_SDK


TRYPATH:=$(call findfirst,third_party/spiffs/esp_spiffs.c,$(SEARCHPATH))
ifneq ($(TRYPATH),)
ESPRTOSSDKPATH:=$(TRYPATH)
endif
endif #ESPRTOSSDKPATH

##################### SXMLC ######################
ifndef SXMLCPATH
SEARCHPATH := \
  /opt/sxmlc/default


TRYPATH:=$(call findfirst,src/sxmlc.h,$(SEARCHPATH))
ifneq ($(TRYPATH),)
SXMLCPATH:=$(TRYPATH)
endif
endif #SXMLCPATH


endif # ifndef OPENMRN_EXPLICIT_DEPS_ONLY
endif # if  $(OS)  != Windows_NT
endif # ifndef OPENMRN_PATH_MK

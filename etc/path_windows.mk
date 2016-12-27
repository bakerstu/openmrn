# Path definitions that are specific to Windows builds

################### FreeRTOS #####################
ARMGCC ?= $(shell \
sh -c "if [ -d /a/openmrn/FreeRTOS/default/Source ]; then echo /a/openmrn/FreeRTOS/default; \
     elif [ -d /b/openmrn/FreeRTOS/default/Source ]; then echo /b/openmrn/FreeRTOS/default; \
     elif [ -d /c/openmrn/FreeRTOS/default/Source ]; then echo /c/openmrn/FreeRTOS/default; \
     elif [ -d /d/openmrn/FreeRTOS/default/Source ]; then echo /d/openmrn/FreeRTOS/default; \
     elif [ -d /e/openmrn/FreeRTOS/default/Source ]; then echo /e/openmrn/FreeRTOS/default; \
     elif [ -d /f/openmrn/FreeRTOS/default/Source ]; then echo /f/openmrn/FreeRTOS/default; \
     elif [ -d /g/openmrn/FreeRTOS/default/Source ]; then echo /g/openmrn/FreeRTOS/default; \
     else echo ; fi" \
)

################### ARM-GCC #####################
ARMGCC ?= $(shell \
sh -c "if [ -d /a/openmrn/windows/armgcc/default/bin ]; then echo /a/openmrn/windows/armgcc/default; \
     elif [ -d /b/openmrn/windows/armgcc/default/bin ]; then echo /b/openmrn/windows/armgcc/default; \
     elif [ -d /c/openmrn/windows/armgcc/default/bin ]; then echo /c/openmrn/windows/armgcc/default; \
     elif [ -d /d/openmrn/windows/armgcc/default/bin ]; then echo /d/openmrn/windows/armgcc/default; \
     elif [ -d /e/openmrn/windows/armgcc/default/bin ]; then echo /e/openmrn/windows/armgcc/default; \
     elif [ -d /f/openmrn/windows/armgcc/default/bin ]; then echo /f/openmrn/windows/armgcc/default; \
     elif [ -d /g/openmrn/windows/armgcc/default/bin ]; then echo /g/openmrn/windows/armgcc/default; \
     else echo ; fi" \
)

################### MIPS-ELF-GCC #####################
ifndef MIPSGCCPATH
SEARCHPATH := \
  /opt/CodeSourcery/default_mips_elf \
  /opt/CodeSourcery/Sourcery_CodeBench_Lite_for_MIPS_ELF \
  /opt/MentorGraphics/default_mips_elf \
  c:/mgc/embedded/codebench


# To download go here https://sourcery.mentor.com/GNUToolchain/release3215

# or the aggregate page:
# https://www.mentor.com/embedded-software/sourcery-tools/sourcery-codebench/editions/lite-edition/
# and make sure to select the ELF release for MIPS processor.

TRYPATH:=$(call findfirst,bin/mips-sde-elf-g++.exe,$(SEARCHPATH))
ifneq ($(TRYPATH),)
MIPSGCCPATH:=$(TRYPATH)
endif
endif #MIPSGCCPATH

################### PIC32MXLIB #####################
ifndef PIC32MXLIBPATH
SEARCHPATH := \
  /opt/microchip/xc32/default \
  $(HOME)/train/git/pic32/includes \
  c:/mgc/xc32/v1.32

TRYPATH:=$(call findfirst,pic32mx/include/p32xxxx.h,$(SEARCHPATH))
ifneq ($(TRYPATH),)
PIC32MXLIBPATH:=$(TRYPATH)
endif
endif #PIC32MXLIBPATH

################### PIC32MXLEGACYPLIB #####################
ifndef PIC32MXLEGACYPLIBPATH
SEARCHPATH := \
  $(PIC32MXLIBPATH) \

TRYPATH:=$(call findfirst,pic32mx/include/peripheral/CAN.h,$(SEARCHPATH))
ifneq ($(TRYPATH),)
PIC32MXLEGACYPLIBPATH:=$(TRYPATH)
endif
endif #PIC32MXLEGACYPLIBPATH

################### FreeRTOS ####################
ifndef FREERTOSPATH
SEARCHPATH := \
  /opt/FreeRTOS \
  /opt/FreeRTOS/default \
  $(HOME)/FreeRTOS \
  /d/FreeRTOS/default \
  c:/mgc/FreeRTOS/FreeRTOSV8.2.0/FreeRTOS

TRYPATH:=$(call findfirst,Source,$(SEARCHPATH))
ifneq ($(TRYPATH),)
FREERTOSPATH:=$(TRYPATH)
endif
endif #FREERTOSPATH

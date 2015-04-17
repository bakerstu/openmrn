OPENMRNPATH ?= $(shell \
sh -c "if [ \"X`printenv OPENMRNPATH`\" != \"X\" ]; then printenv OPENMRNPATH; \
     elif [ -d /opt/openmrn/src ]; then echo /opt/openmrn; \
     elif [ -d ~/openmrn/src ]; then echo ~/openmrn; \
     elif [ -d ../../../src ]; then echo ../../..; \
     else echo OPENMRNPATH not found; fi" \
)

# Find LPC Chip libraries
include $(OPENMRNPATH)/etc/lpc_chip_17xx_40xx.mk

LDFLAGSEXTRA += 
SYSLIBRARIESEXTRA += 
OBJEXTRA += 

CFLAGS += -DCHIP_LPC175X_6X
CXXFLAGS += -DCHIP_LPC175X_6X

TARGET := freertos.armv7m
#BOARD := BOARD_LAUNCHPAD_EK
include $(OPENMRNPATH)/etc/prog.mk

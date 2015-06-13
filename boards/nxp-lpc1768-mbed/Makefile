-include ../../config.mk
OPENMRNPATH ?= $(shell \
sh -c "if [ \"X`printenv OPENMRNPATH`\" != \"X\" ]; then printenv OPENMRNPATH; \
     elif [ -d /opt/openmrn/src ]; then echo /opt/openmrn; \
     elif [ -d ~/openmrn/src ]; then echo ~/openmrn; \
     elif [ -d ../../../src ]; then echo ../../..; \
     else echo OPENMRNPATH not found; fi" \
)

OBJEXTRA += $(OPENMRNPATH)/targets/freertos.armv7m/freertos_drivers/lpc_chip_175x_6x/clock_17xx_40xx.o 

LDFLAGSEXTRA += 
SYSLIBRARIESEXTRA += -lfreertos_drivers_lpc_chip_175x_6x

# Find LPC Chip libraries
include $(OPENMRNPATH)/boards/nxp-lpc175x_6x-generic/Makefile

ifeq ($(MISSING_DEPS),)
all: $(EXECUTABLE).bin
endif

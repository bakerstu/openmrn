APP_PATH ?= $(realpath ../..)
-include $(APP_PATH)/config.mk

OPENMRNPATH ?= $(shell \
sh -c "if [ \"X`printenv OPENMRNPATH`\" != \"X\" ]; then printenv OPENMRNPATH; \
     elif [ -d /opt/openmrn/src ]; then echo /opt/openmrn; \
     elif [ -d ~/openmrn/src ]; then echo ~/openmrn; \
     elif [ -d ../../../src ]; then echo ../../..; \
     else echo OPENMRNPATH not found; fi" \
)

# Find TivaWare
include $(OPENMRNPATH)/etc/tivaware.mk

export TARGET := freertos.armv7m
export BOARD := BOARD_BRACZ_ACC

LDFLAGSEXTRA = -L$(TIVAWAREPATH)/driverlib/gcc \

SYSLIBRARIESEXTRA = -ldriver
OBJEXTRA = $(OPENMRNPATH)/targets/freertos.armv7m/freertos_drivers/tivaware/TivaCan.o \
           $(OPENMRNPATH)/targets/freertos.armv7m/freertos_drivers/tivaware/TivaUart.o \

CXXFLAGSEXTRA += -DPART_TM4C123AE6PM

include $(OPENMRNPATH)/etc/prog.mk

ifdef ADDRESS
CXXFLAGS+= -DNODEID_LOW_BITS=$(ADDRESS)
else
$(error Must define address with ADDRESS=0x4c)
endif

flash: $(EXECUTABLE).elf
	ti-flash.sh $<

gdb:
	ti-gdb.sh $(EXECUTABLE).elf

# This will force re-building NodeId.cxx every invocation, because the address
# might have changed.
NodeId.o: FORCE

FORCE:

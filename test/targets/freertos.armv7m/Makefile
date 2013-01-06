OPENMRNPATH ?= $(shell \
sh -c "if [ \"X`printenv OPENMRNPATH`\" != \"X\" ]; then printenv OPENMRNPATH; \
     elif [ -d /opt/openmrn/src ]; then echo /opt/openmrn; \
     elif [ -d ~/openmrn/src ]; then echo ~/openmrn; \
     else echo OPENMRNPATH not found; fi" \
)

# Find StellarisWare
include $(OPENMRNPATH)/etc/stellarisware.mk

LDFLAGSEXTRA = -L$(STELLARISWAREPATH)/driverlib/gcc-cm3
SYSLIBRARIESEXTRA = -ldriver-cm3
OBJEXTRA = $(OPENMRNPATH)/targets/freertos.armv7m/freertos_drivers/stellaris_can.o

include $(OPENMRNPATH)/etc/prog.mk

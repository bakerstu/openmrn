APP_PATH ?= $(realpath ../..)
-include $(APP_PATH)/config.mk

OPENMRNPATH ?= $(shell \
sh -c "if [ \"X`printenv OPENMRNPATH`\" != \"X\" ]; then printenv OPENMRNPATH; \
     elif [ -d /opt/openmrn/src ]; then echo /opt/openmrn; \
     elif [ -d ~/openmrn/src ]; then echo ~/openmrn; \
     elif [ -d ../../../src ]; then echo ../../..; \
     else echo OPENMRNPATH not found; fi" \
)

# Find driverlib
include $(OPENMRNPATH)/etc/cc32xx.mk

LDFLAGSEXTRA +=
SYSLIBRARIESEXTRA +=
OBJEXTRA =

CXXFLAGSEXTRA += -DTARGET_IS_CC3200

include $(OPENMRNPATH)/etc/prog.mk

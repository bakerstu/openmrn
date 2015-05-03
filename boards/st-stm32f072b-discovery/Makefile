OPENMRNPATH ?= $(shell \
sh -c "if [ \"X`printenv OPENMRNPATH`\" != \"X\" ]; then printenv OPENMRNPATH; \
     elif [ -d /opt/openmrn/src ]; then echo /opt/openmrn; \
     elif [ -d ~/openmrn/src ]; then echo ~/openmrn; \
     elif [ -d ../../../src ]; then echo ../../..; \
     else echo OPENMRNPATH not found; fi" \
)

# Find STM32CubeF0 libraries
include $(OPENMRNPATH)/etc/stm32cubef0.mk

LDFLAGSEXTRA += 
SYSLIBRARIESEXTRA += -lfreertos_drivers_stm32cubef071xb_2xb
OBJEXTRA += 

CFLAGS += -DSTM32F072xB
CXXFLAGS += -DSTM32F072xB

TARGET := freertos.armv6m
include $(OPENMRNPATH)/etc/prog.mk

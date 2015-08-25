OPENMRNPATH ?= $(shell \
sh -c "if [ \"X`printenv OPENMRNPATH`\" != \"X\" ]; then printenv OPENMRNPATH; \
     elif [ -d /opt/openmrn/src ]; then echo /opt/openmrn; \
     elif [ -d ~/openmrn/src ]; then echo ~/openmrn; \
     elif [ -d ../../../src ]; then echo ../../..; \
     else echo OPENMRNPATH not found; fi" \
)

# Find STM32CubeF0 libraries
include $(OPENMRNPATH)/etc/stm32cubef3.mk

LDFLAGSEXTRA += 
SYSLIBRARIESEXTRA += -lfreertos_drivers_stm32cubef303x_28x_58x_98x
OBJEXTRA += 

CFLAGS += -DSTM32F303xC
CXXFLAGS += -DSTM32F303xC

TARGET := freertos.armv7m
include $(OPENMRNPATH)/etc/prog.mk

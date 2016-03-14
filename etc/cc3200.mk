include $(OPENMRNPATH)/etc/path.mk

ifdef TICC3200SDKPATH
INCLUDES += -I$(TICC3200SDKPATH) \
            -I$(OPENMRNPATH)/src/freertos_drivers/ti
endif

DEPS += TICC3200SDKPATH

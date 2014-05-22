include $(OPENMRNPATH)/etc/path.mk

ifdef TIVAWAREPATH
INCLUDES += -I$(TIVAWAREPATH) \
            -I$(OPENMRNPATH)/src/freertos_drivers/ti
endif

DEPS += TIVAWAREPATH

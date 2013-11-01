include $(OPENMRNPATH)/etc/path.mk

ifdef STELLARISWAREPATH
INCLUDES += -I$(STELLARISWAREPATH) \
            -I$(OPENMRNPATH)/src/freertos_drivers/stellarisware
endif

DEPS += STELLARISWAREPATH

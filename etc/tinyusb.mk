include $(OPENMRNPATH)/etc/path.mk

ifdef TINYUSBPATH
INCLUDES += -I$(TINYUSBPATH)/src \
	-I$(OPENMRNPATH)/src/freertos_drivers/tinyusb

TUSB_VPATH = $(TINYUSBPATH)/src \
        $(TINYUSBPATH)/src/class/cdc \
        $(TINYUSBPATH)/src/common \
        $(TINYUSBPATH)/src/device \

endif

DEPS += TINYUSBPATH

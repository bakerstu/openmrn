include $(OPENMRNPATH)/etc/path.mk

SYSLIB_SUBDIRS += freertos freertos_drivers
SYSLIBRARIES += -lfreertos -lfreertos_drivers $(OPENMRNPATH)/targets/$(TARGET)/freertos/event_groups.o
DEPS += FREERTOSPATH

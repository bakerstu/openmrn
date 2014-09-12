include $(OPENMRNPATH)/etc/path.mk

SYSLIB_SUBDIRS += freertos freertos_drivers
SYSLIBRARIES += -lfreertos -lfreertos_drivers
DEPS += FREERTOSPATH

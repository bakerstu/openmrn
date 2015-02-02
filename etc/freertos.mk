include $(OPENMRNPATH)/etc/path.mk

SYSLIB_SUBDIRS += freertos freertos_drivers
SYSLIBRARIES += -lfreertos_drivers -lfreertos 
LDFLAGSEXTRA += -Wl,--undefined=_open_r

DEPS += FREERTOSPATH

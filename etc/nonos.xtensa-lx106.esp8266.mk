
include $(OPENMRNPATH)/etc/xtensa-lx106.mk

INCLUDES += -I $(OPENMRNPATH)/include/esp8266  \

SYSLIB_SUBDIRS += spiffs core
SYSLIBRARIES += -lspiffs -lcore


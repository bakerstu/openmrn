include $(OPENMRNPATH)/etc/path.mk

ifdef FREERTOSTCPPATH
INCLUDES += -I$(FREERTOSTCPPATH) \
	    -I$(FREERTOSTCPPATH)/include \
	    -I$(FREERTOSTCPPATH)/portable/Compiler/GCC \
            -I$(OPENMRNPATH)/src/freertos_drivers/ti \
            -I$(OPENMRNPATH)/src/freertos_drivers/net_freertos_tcp

SYSLIBRARIESEXTRA += -lfreertos_drivers_freertos_tcp \
                     -lfreertos_drivers_net_freertos_tcp
TARGET := freertos.armv7m
endif

DEPS += FREERTOSTCPPATH

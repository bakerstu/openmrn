include $(OPENMRNPATH)/etc/path.mk

DEPS += TICC3220SDKPATH

ifdef TICC3220SDKPATH
INCLUDES += -DSL_PLATFORM_MULTI_THREADED -DSL_FULL -DTARGET_IS_CC3220 \
            -I$(OPENMRNPATH)/src/freertos_drivers/ti \
            -I$(OPENMRNPATH)/src/freertos_drivers/net_cc32xx \
            -I$(TICC3220SDKPATH)/source/ti/devices/cc32xx/driverlib \
            -I$(TICC3220SDKPATH)/source/ti/devices/cc32xx \
            -I$(TICC3220SDKPATH)/source/ti/posix/gcc

ifneq ($(SPIFFSPATH),)
INCLUDES += -I$(SPIFFSPATH)/src \
            -I$(OPENMRNPATH)/src/freertos_drivers/spiffs \
            -I$(OPENMRNPATH)/src/freertos_drivers/spiffs/cc32x0sf
endif

SYSLIBRARIESEXTRA += $(TICC3220SDKPATH)/source/ti/devices/cc32xx/driverlib/gcc/Release/driverlib.a \
                     -lfreertos_drivers_cc3220 \
                     -lfreertos_drivers_cc3220sdk \
                     -lfreertos_drivers_net_cc3220 \
                     -lutils \
                     $(OPENMRNPATH)/targets/freertos.armv7m/freertos_drivers/cc3220sdk/SimplelinkPthread.o

ifneq ($(SPIFFSPATH),)
SYSLIBRARIESEXTRA += -lfreertos_drivers_spiffs_cc32x0sf
endif

INCLUDES += -I$(OPENMRNPATH)/src/freertos_drivers/net_cc322x \
            -I$(TICC3220SDKPATH)/source

ifndef EXCLUDESDKINCLUDES
INCLUDES += -idirafter $(TICC3220SDKPATH)/source/ti/drivers/net/wifi \
            -I$(TICC3220SDKPATH)/source/
endif

TiDrivers.o : CFLAGS += -Wno-strict-prototypes

endif


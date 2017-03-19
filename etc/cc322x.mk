include $(OPENMRNPATH)/etc/path.mk

DEPS += TICC3220SDKPATH

ifdef TICC3220SDKPATH
INCLUDES += -DSL_PLATFORM_MULTI_THREADED \
            -I$(OPENMRNPATH)/src/freertos_drivers/ti/CC3200_compat \
            -I$(OPENMRNPATH)/src/freertos_drivers/ti \
            -I$(OPENMRNPATH)/src/freertos_drivers/net_cc32xx \
            -I$(TICC3220SDKPATH)/source/ti/devices/cc32xx \
            -I$(TICC3220SDKPATH)/source/ti/devices/cc32xx/driverlib \


SYSLIBRARIESEXTRA += $(TICC3220SDKPATH)/source/ti/devices/cc32xx/driverlib/gcc/Release/driverlib.a \
                     -lfreertos_drivers_cc3220 \
                     -lfreertos_drivers_cc3220sdk \
                     -lfreertos_drivers_net_cc3220 \
                     -lutils


INCLUDES += -I$(OPENMRNPATH)/src/freertos_drivers/net_cc322x \
            -I$(TICC3220SDKPATH)/source \
            -idirafter $(TICC3220SDKPATH)/source/ti/drivers/net/wifi \
            -I$(TICC3220SDKPATH)/source/ti/drivers/net/wifi/source \
            -I$(TICC3220SDKPATH)/source/ti/drivers/net/wifi/porting \


#  ??   -I$(TICC3220SDKPATH)/oslib \

TiDrivers.o : CFLAGS+= -Wno-strict-prototypes

endif


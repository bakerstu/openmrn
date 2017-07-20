include $(OPENMRNPATH)/etc/path.mk

DEPS += TICC3220SDKPATH

ifdef TICC3220SDKPATH
INCLUDES += -DSL_PLATFORM_MULTI_THREADED -DSL_API_V2 -DTARGET_IS_CC3220 \
            -I$(OPENMRNPATH)/src/freertos_drivers/ti/CC3200_compat \
            -I$(OPENMRNPATH)/src/freertos_drivers/ti \
            -I$(OPENMRNPATH)/src/freertos_drivers/net_cc32xx \
            -I$(TICC3220SDKPATH)/source/ti/devices/cc32xx \
            -I$(TICC3220SDKPATH)/source/ti/devices/cc32xx/driverlib \


SYSLIBRARIESEXTRA += $(TICC3220SDKPATH)/source/ti/devices/cc32xx/driverlib/gcc/Release/driverlib.a \
                     -lfreertos_drivers_cc3220 \
                     -lfreertos_drivers_cc3220sdk \
                     -lfreertos_drivers_cc3220driverlib \
                     -lfreertos_drivers_net_cc3220 \
                     -lutils


INCLUDES += -I$(OPENMRNPATH)/src/freertos_drivers/net_cc322x \
            -I$(TICC3220SDKPATH)/source

ifeq ($(wildcard $(TICC3220SDKPATH)/source/ti/drivers/net/wifi/sys/errno.h),)
#new version of SDK
INCLUDES += -DSIMPLELINK_SDK_V1_4
endif

ifndef EXCLUDESDKINCLUDES
INCLUDES += -idirafter $(TICC3220SDKPATH)/source/ti/drivers/net/wifi \
            -I$(TICC3220SDKPATH)/source/ti/drivers/net/wifi/source \
            -I$(TICC3220SDKPATH)/source/ti/drivers/net/wifi/porting
endif

#  ??   -I$(TICC3220SDKPATH)/oslib \

TiDrivers.o : CFLAGS+= -Wno-strict-prototypes

endif


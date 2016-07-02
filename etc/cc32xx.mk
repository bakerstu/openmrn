include $(OPENMRNPATH)/etc/path.mk

ifdef TICC3200SDKPATH
INCLUDES += -I$(OPENMRNPATH)/src/freertos_drivers/ti \
            -I$(OPENMRNPATH)/src/freertos_drivers/net_cc32xx \
            -I$(TICC3200SDKPATH)


LDFLAGSEXTRA += -L$(TICC3200SDKPATH)/driverlib/gcc/exe
SYSLIBRARIESEXTRA += -ldriver \
                     -lfreertos_drivers_cc32xx \
                     -lfreertos_drivers_cc32xxsdk \
                     -lfreertos_drivers_net_cc32xx

export TARGET := freertos.armv7m
endif

DEPS += TICC3200SDKPATH

include $(OPENMRNPATH)/etc/path.mk

ifdef TICC3200SDKPATH
INCLUDES += -I$(TICC3200SDKPATH) \
            -I$(OPENMRNPATH)/src/freertos_drivers/ti \
            -I$(OPENMRNPATH)/src/freertos_drivers/net_cc32xx

LDFLAGSEXTRA += -L$(TICC3200SDKPATH)/driverlib/gcc/exe
SYSLIBRARIESEXTRA += -ldriver -lfreertos_drivers_cc32xx \
                     -lfreertos_drivers_net_cc32xx
TARGET := freertos.armv7m
endif

DEPS += TICC3200SDKPATH

include $(OPENMRNPATH)/etc/path.mk

ifdef TIVAWAREPATH
INCLUDES += -I$(TIVAWAREPATH) \
            -I$(OPENMRNPATH)/src/freertos_drivers/ti

# set library names when using locally built Tivaware
SYSLIBRARIESEXTRA = -lfreertos_drivers_tivausblib -lfreertos_drivers_tivausblib_device \
		-lfreertos_drivers_tivadriverlib

# set library names when using prebuilt Tivaware (remove # to activate)
#SYSLIBRARIESEXTRA = -lusb -ldriver
#LDFLAGSEXTRA = -L$(TIVAWAREPATH)/driverlib/gcc \
#               -L$(TIVAWAREPATH)/usblib/gcc
endif

DEPS += TIVAWAREPATH

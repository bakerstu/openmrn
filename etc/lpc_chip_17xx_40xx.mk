include $(OPENMRNPATH)/etc/path.mk

ifdef LPCCHIPPATH_17XX_40XX
INCLUDES += -I$(OPENMRNPATH)/src/freertos_drivers/nxp \
            -I$(LPCCHIPPATH_17XX_40XX)/inc
endif

# unfortunately, the lpc_chip libraries do not compile with no warnings
# we define __SYS_CONFIG_H_ in order to override the CHIP_LPC* define in a
# Makefile rather than in sys_config.h
CFLAGS += -D__SYS_CONFIG_H_ -DCORE_M3 -Wno-error \
          -Wno-strict-prototypes -Wno-uninitialized \
          -Wno-unused-but-set-variable -Wno-switch
CXXFLAGS += -D__SYS_CONFIG_H_ -DCORE_M3

DEPS += LPCCHIPPATH_17XX_40XX

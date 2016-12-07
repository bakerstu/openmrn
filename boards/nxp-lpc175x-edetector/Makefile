OPENMRNPATH ?= $(shell \
sh -c "if [ \"X`printenv OPENMRNPATH`\" != \"X\" ]; then printenv OPENMRNPATH; \
     elif [ -d /opt/openmrn/src ]; then echo /opt/openmrn; \
     elif [ -d ~/openmrn/src ]; then echo ~/openmrn; \
     elif [ -d ../../../src ]; then echo ../../..; \
     else echo OPENMRNPATH not found; fi" \
)

# Find LPC Chip libraries
include $(OPENMRNPATH)/etc/lpc_chip_17xx_40xx.mk

OBJEXTRA += $(OPENMRNPATH)/targets/freertos.armv7m/freertos_drivers/lpc_chip_175x_6x/clock_17xx_40xx.o 

LDFLAGSEXTRA += 
SYSLIBRARIESEXTRA += -lfreertos_drivers_lpc_chip_175x_6x

CFLAGS += -DCHIP_LPC175X_6X
CXXFLAGS += -DCHIP_LPC175X_6X

TARGET := freertos.armv7m
#BOARD := BOARD_LAUNCHPAD_EK
include $(OPENMRNPATH)/etc/prog.mk

OPENOCDARGS = -f interface/ftdi/olimex-arm-usb-tiny-h.cfg  -f target/lpc17xx.cfg
# -f interface/ftdi/swd-resistor-hack.cfg

flash: $(EXECUTABLE)$(EXTENTION) $(EXECUTABLE).lst
	dfu-util -d 0x471:0xdf55 -c 0 -t 2048 -R -D $(LPCXPRESSOPATH)/bin/LPCXpressoWIN.enc && sleep 2 || exit 0
	PATH=$(LPCXPRESSOPATH)/bin:$$PATH LD_LIBRARY_PATH=$(LPCXPRESSOPATH)/bin $(LPCXPRESSOPATH)/bin/crt_emu_cm3_nxp -pLPC1759 -wire=winusbswd -flash-load-exec=$< -s1000


gdb:
	dfu-util -d 0x471:0xdf55 -c 0 -t 2048 -R -D $(LPCXPRESSOPATH)/bin/LPCXpressoWIN.enc && sleep 2 || exit 0
	PATH=$(LPCXPRESSOPATH)/bin:$$PATH LD_LIBRARY_PATH=$(LPCXPRESSOPATH)/bin $(GDB) $(EXECUTABLE)$(EXTENTION) -ex "target extended-remote | $(LPCXPRESSOPATH)/bin/crt_emu_cm3_nxp -pLPC1759 -wire=winusbswd" -ex "continue"


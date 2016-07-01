-include ../../config.mk
include $(OPENMRNPATH)/etc/path.mk

INCLUDES += -I$(ESPARDUINOPATH)/hardware/esp8266/2.2.0/bootloaders/eboot

include hardware.mk

ifndef ADDRESS
ADDRESS=0xDD
endif

.PHONY: FORCE

address.h: FORCE
	@ADR="#define NODEID_LOW_BITS $(ADDRESS)" ; if [ ! -f $@ ] || [ "$$ADR" != "$$(<$@)" ] ; then echo replacing address file. old: $$(<$@) new $$ADR ; echo "$$ADR" > $@ ; else echo address file up-to-date for $(TARGET):$@ ; fi 

NodeId.o: address.h


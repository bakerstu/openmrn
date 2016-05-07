APP_PATH ?= $(realpath ../..)
-include $(APP_PATH)/config.mk

export TARGET ?= nonos.xtensa-lx106.esp8266

include $(OPENMRNPATH)/etc/prog.mk

$(EXECUTABLE)-0x00000.bin: $(EXECUTABLE)$(EXTENTION)
	ln -sf $(EXECUTABLE)$(EXTENTION) $(EXECUTABLE)
	$(ESPTOOL) elf2image $(EXECUTABLE)

flash: $(EXECUTABLE)-0x00000.bin
	$(ESPTOOL)  write_flash 0 $(EXECUTABLE)-0x00000.bin 0x40000 $(EXECUTABLE)-0x40000.bin

clean:
	rm -f $(EXECUTABLE)-0x00000.bin $(EXECUTABLE)-0x40000.bin

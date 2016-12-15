APP_PATH ?= $(realpath ../..)
-include $(APP_PATH)/config.mk

export TARGET ?= nonos.xtensa-lx106.esp8266

#LDSCRIPT ?= eagle.app.v6.new.2048.ld
#LDSCRIPT ?= eagle.app.v6.ld
#LDSCRIPT = test.ld
#LDSCRIPT = $(ESPNONOSSDKPATH)/ld/eagle.flash.4m.ld
LDSCRIPT = target.ld

ifndef ADDRESS
ADDRESS=0xDD
endif

include $(OPENMRNPATH)/etc/prog.mk


XLATEDLIBS=nmranet spiffs utils os executor

ifdef FOOOXXX
XLATEDSTDLIBS=stdc++ c
XLATEDGCCLIBS=gcc
endif


FULLP_XLATEDLIBS=$(foreach fbase,$(XLATEDLIBS) $(XLATEDGCCLIBS) $(XLATEDSTDLIBS),lib/lib$(fbase).a)

### Helper function to copy a library from the core target directory to the
### application target/lib subdirectory
### 
### arg 1: basename of the library (e.g. 'nmranet')
### arg 2: source where the library will be copied from
###
define LIBCOPY_template

#	mkdir lib/$(1)
#	cd lib/$(1) ; $(AR) xv $$< | cut -d " " -f 3 | while read fname ; do 


lib/lib$(1).a: $(2) lib/timestamp $$(LIBDIR)/timestamp hardware.mk
	$(OBJDUMP) -h $$<  | grep [.]text[.] | cut -d . -f 3- | cut -d " " -f 1 | sed 's/.*/--rename-section .text.\0=.irom0.text --rename-section .literal.\0=.irom0.literal/g' > lib/flagfile$(1).lst
	$(OBJCOPY) @lib/flagfile$(1).lst --rename-section .text=.irom0.text --rename-section .literal=.irom0.literal $$< $$@
	rm -f $$(LIBDIR)/lib$(1).a

$$(LIBDIR)/lib$(1).a: $$(LIBDIR)/timestamp

$(2): $$(LIBDIR)/timestamp

endef

$(eval $(foreach fbase,$(XLATEDLIBS),$(call LIBCOPY_template,$(fbase),$(LIBDIR)/../$(fbase)/lib$(fbase).a)))

$(eval $(foreach fbase,$(XLATEDSTDLIBS),$(call LIBCOPY_template,$(fbase),$(XTENSAGCCPATH)/xtensa-lx106-elf/lib/lib$(fbase).a)))

$(eval $(foreach fbase,$(XLATEDGCCLIBS),$(call LIBCOPY_template,$(fbase),$(wildcard $(XTENSAGCCPATH)/lib/gcc/xtensa-lx106-elf/*/lib$(fbase).a))))

$(EXECUTABLE)$(EXTENTION): $(FULLP_XLATEDLIBS) scrape_main_o

scrape_main_o: main.o
	$(OBJDUMP) -h $<  | grep [.]text[.] | cut -d . -f 3- | cut -d " " -f 1 | sed 's/.*/--rename-section .text.\0=.irom0.text --rename-section .literal.\0=.irom0.literal/g' > lib/flagfilemaino.lst
	$(OBJCOPY) @lib/flagfilemaino.lst $<
	touch $@


$(EXECUTABLE)-0x00000.bin: $(EXECUTABLE)$(EXTENTION)
	ln -sf $(EXECUTABLE)$(EXTENTION) $(EXECUTABLE)
	$(ESPTOOL) elf2image $(EXECUTABLE)
	rm -f $(EXECUTABLE)


$(EXECUTABLE)-bload.bin:  $(EXECUTABLE)$(EXTENTION)
	$(ESPARDUINOPATH)/tools/esptool/*/esptool -eo "$(ESPARDUINOPATH)/hardware/esp8266/2.2.0/bootloaders/eboot/eboot.elf" -bo $@ -bm qio -bf 40 -bz 4M -bs .text -bp 4096 -ec -eo $< -bs .irom0.text -bs .text -bs .iram.text -bs .data -bs .rodata -bc -ec

$(EXECUTABLE)-btgt.bin:  $(EXECUTABLE)$(EXTENTION)
	$(ESPARDUINOPATH)/tools/esptool/*/esptool -eo $< -bo $@ -bm qio -bf 40 -bz 4M -bs .irom0.text -bs .text -bs .iram.text -bs .data -bs .rodata -bc -ec

flash: $(EXECUTABLE)-bload.bin $(EXECUTABLE).lst
	$(ESPTOOL) write_flash 0 $<

clean:
	rm -f $(EXECUTABLE)-{0x00000,0x40000,bload,btgt}.bin lib/*.a lib/*.lst


xflash: $(EXECUTABLE)-bload.bin $(EXECUTABLE).lst
	$(ESPARDUINOPATH)/tools/esptool/*/esptool -vv -cd nodemcu -cb 230400 -cp /dev/ttyUSB0 -ca 0x00000 -cf $<


rflash: $(EXECUTABLE)-btgt.bin $(EXECUTABLE).lst
	$(OPENMRNPATH)/applications/bootloader_client/targets/linux.x86/bootloader_client -r -c esp8266 -w 10 -n 0x0501010114$$(printf %02x $(ADDRESS)) -f $< 


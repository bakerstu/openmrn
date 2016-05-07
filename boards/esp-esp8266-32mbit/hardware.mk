APP_PATH ?= $(realpath ../..)
-include $(APP_PATH)/config.mk

export TARGET ?= nonos.xtensa-lx106.esp8266

#LDSCRIPT ?= eagle.app.v6.new.2048.ld
LDSCRIPT ?= eagle.app.v6.ld

include $(OPENMRNPATH)/etc/prog.mk


XLATEDLIBS=nmranet

FULLP_XLATEDLIBS=$(foreach fbase,$(XLATEDLIBS),lib/lib$(fbase).a)

### Helper function to copy a library from the core target directory to the
### application target/lib subdirectory
### 
### arg 1: basename of the library (e.g. 'nmranet')
###
define LIBCOPY_template

#	mkdir lib/$(1)
#	cd lib/$(1) ; $(AR) xv $$< | cut -d " " -f 3 | while read fname ; do 


lib/lib$(1).a: $$(LIBDIR)/../$(1)/lib$(1).a lib/timestamp $$(LIBDIR)/timestamp hardware.mk
	$(OBJDUMP) -h $$<  | grep [.]text[.] | cut -d . -f 3 | cut -d " " -f 1 | sed 's/.*/--rename-section .text.\0=.irom0.text.\0/g' > lib/flagfile$(1).lst
	$(OBJCOPY) @lib/flagfile$(1).lst $$< $$@
	rm -f $$(LIBDIR)/lib$(1).a

$$(LIBDIR)/lib$(1).a: $$(LIBDIR)/timestamp

endef

$(eval $(foreach fbase,$(XLATEDLIBS),$(call LIBCOPY_template,$(fbase))))

$(EXECUTABLE)$(EXTENTION): $(FULLP_XLATEDLIBS)

$(EXECUTABLE)-0x00000.bin: $(EXECUTABLE)$(EXTENTION)
	ln -sf $(EXECUTABLE)$(EXTENTION) $(EXECUTABLE)
	$(ESPTOOL) elf2image $(EXECUTABLE)
	rm -f $(EXECUTABLE)

flash: $(EXECUTABLE)-0x00000.bin
	$(ESPTOOL)  write_flash 0 $(EXECUTABLE)-0x00000.bin 0x40000 $(EXECUTABLE)-0x40000.bin

clean:
	rm -f $(EXECUTABLE)-0x00000.bin $(EXECUTABLE)-0x40000.bin

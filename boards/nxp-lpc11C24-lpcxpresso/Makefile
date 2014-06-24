APP_PATH ?= $(realpath ../..)
-include $(APP_PATH)/config.mk

export TARGET := freertos.armv6m

DEPS += HAVE_MBED

OBJEXTRA = $(OPENMRNPATH)/targets/freertos.armv6m/freertos_drivers/drivers_lpc11cxx/11cxx_async_can.o \


#           $(OPENMRNPATH)/targets/freertos.armv6m/freertos_drivers/mbed_i2c.o \


SYSLIBRARIESEXTRA = -lmbed

LDFLAGSEXTRA = -Wl,--wrap=__cxa_pure_virtual  -Wl,--wrap=__cxa_atexit  -Wl,--wrap=exit

include $(OPENMRNPATH)/etc/prog.mk

ifeq ($(MISSING_DEPS),)

all: $(EXECUTABLE).bin $(EXECUTABLE).lst

$(EXECUTABLE)$(EXTENTION): target.ld

delete.sym: delete_destructors Makefile | $(LIBDIR)/timestamp
	cat $< | grep -v '^[[:space:]]$$' | while read a ; do echo $${a}D[0125]Ev ; done > delete_pat.tmp
	nm $(FULLPATHLIBS) $(OBJS) | grep -f delete_pat.tmp | cut -c 11- | sort | uniq > $@

arpreproc: delete.sym
	while read a ; do echo -W $${a} ; done < $< > $@

ldpreproc: delete.sym
	(echo --undef=destructor ; while read a ; do echo --defsym=$${a}=destructor ; done < $< ) > $@


delete_destructors:
	if [ ! -f $@ ] ; then touch $@ ; fi

define TGTLIBSTRIP_template

lib/$(notdir $(1)): $(LIBDIR)/$(notdir $(1)) arpreproc $(LIBDIR)/timestamp
	$(OBJCOPY) @arpreproc $$< $$@

strippedlibs: lib/$(notdir $(1))

endef

# Calls the directory template for every .testout file we have.
$(foreach lib,$(wildcard $(LIBDIR)/*.a),$(eval $(call TGTLIBSTRIP_template,$(lib))))

$(EXECUTABLE)$(EXTENTION): $(OBJS) $(FULLPATHLIBS) $(LIBDIR)/timestamp lib/timestamp arpreproc ldpreproc strippedlibs
	for f in $(SUBDIRS) ; do $(OBJCOPY) @arpreproc lib/$$f; done
	for f in $(OBJS) $(OBJEXTRA) ; do $(OBJCOPY) @arpreproc $$f ; done
	$(LD) -o $@ $(OBJS) $(OBJEXTRA) $(LDFLAGS) $(LIBS) $(SYSLIBRARIES)
ifdef SIZE
	$(SIZE) $@
endif

LDFLAGSEXTRA += -Wl,@ldpreproc 


%.lst: %.elf
	$(SIZE) $<
	$(OBJDUMP) -d $< > $@

%.bin: %.elf
	$(OBJCOPY) -O binary $< $@
	$(CHECKSUM) -p LPC11C24 -d $@

endif #MISSING_DEPS

APP_PATH ?= $(realpath ../..)
-include $(APP_PATH)/config.mk

export TARGET := freertos.armv6m

DEPS += HAVE_MBED

OBJEXTRA = $(OPENMRNPATH)/targets/freertos.armv6m/freertos_drivers/11cxx_async_can.o \


#           $(OPENMRNPATH)/targets/freertos.armv6m/freertos_drivers/mbed_i2c.o \


SYSLIBRARIESEXTRA = -lmbed

LDFLAGSEXTRA = -Wl,--wrap=__cxa_pure_virtual  -Wl,--wrap=__cxa_atexit  -Wl,--wrap=exit

include $(OPENMRNPATH)/etc/prog.mk

ifeq ($(MISSING_DEPS),)

all: $(EXECUTABLE).bin $(EXECUTABLE).lst

$(EXECUTABLE)$(EXTENTION): target.ld


arpreproc: delete_destructors
	while read a ; do echo -W $${a}D0Ev  -W $${a}D1Ev -W $${a}D2Ev ; done < $< > $@


ldpreproc: delete_destructors
	(echo --undef=destructor ; while read a ; do echo --defsym=$${a}D0Ev=destructor  --defsym=$${a}D1Ev=destructor --defsym=$${a}D2Ev=destructor ; done < $< ) > $@


delete_destructors:
	if [ ! -f $@ ] ; then touch $@ ; fi


$(EXECUTABLE)$(EXTENTION): $(OBJS) $(FULLPATHLIBS) $(LIBDIR)/timestamp lib/timestamp arpreproc ldpreproc
	for f in $(FULLPATHLIBS) ; do $(OBJCOPY) @arpreproc $$f; done
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

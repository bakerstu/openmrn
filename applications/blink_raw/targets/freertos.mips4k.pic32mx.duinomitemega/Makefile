OPENMRNPATH ?= $(shell \
sh -c "if [ \"X`printenv OPENMRNPATH`\" != \"X\" ]; then printenv OPENMRNPATH; \
     elif [ -d /opt/openmrn/src ]; then echo /opt/openmrn; \
     elif [ -d ~/openmrn/src ]; then echo ~/openmrn; \
     elif [ -d ../../../../src ]; then echo ../../../..; \
     else echo OPENMRNPATH not found; fi" \
)
export OPENMRNPATH:=$(realpath $(OPENMRNPATH))
TARGET := freertos.mips4k.pic32mx


include $(OPENMRNPATH)/etc/prog.mk

ifeq ($(MISSING_DEPS),)

all: $(EXECUTABLE).hex $(EXECUTABLE).lst 

%.lst: %.elf
	$(SIZE) $<
	$(OBJDUMP) -d $< > $@

%.delf: %.elf
	$(OBJCOPY) $$($(OBJDUMP) -h blink_raw.elf | grep ' b[df]' | cut  -b 5- | sed -e 's/^\([^ ]*\) .*$$/--change-section-lma \1-0xA0000000/' | tr '\n' ' ') $< $@
#	$(OBJDUMP) -h $< | grep ' b[df]'
#	cp -f $< $@

%.hex: %.delf
	$(OBJDUMP) -h $<
	$(OBJCOPY) -O ihex $< $@

$(EXECUTABLE)$(EXTENTION): depmake

.PHONY: depmake

depmake:
	+$(MAKE) -C $(OPENMRNPATH)/targets/$(TARGET) all

endif  #MISSING_DEPS

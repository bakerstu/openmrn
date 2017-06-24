include $(OPENMRNPATH)/etc/path.mk

ifneq ($(strip $(ARMGCCPATH)),)
HAVE_ARMGCC = 1
TOOLPATH:=$(ARMGCCPATH)
ARMGCCVERSION:=$(shell $(ARMGCCPATH)/bin/arm-none-eabi-gcc -dumpversion)
ARMGCCMAJORVERSION:=$(word 1,$(subst ., ,$(ARMGCCVERSION)))
ARMGCCMINORVERSION:=$(word 2,$(subst ., ,$(ARMGCCVERSION)))
else
TOOLPATH=
endif


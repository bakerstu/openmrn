include $(OPENMRNPATH)/etc/path.mk

ifneq ($(strip $(ARMGCCPATH)),)
HAVE_ARMGCC = 1
TOOLPATH:=$(ARMGCCPATH)
else
TOOLPATH=
endif

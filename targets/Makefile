SUBDIRS = linux.x86 nuttx.armv7m freertos.armv7m mach.x86 mach.x86_64

all:
	$(foreach dir, $(SUBDIRS), $(MAKE) -C $(dir) all || exit 1;)

clean::
	$(foreach dir, $(SUBDIRS), $(MAKE) -C $(dir) clean || exit 1;)

veryclean:: clean
	$(foreach dir, $(SUBDIRS), $(MAKE) -C $(dir) veryclean || exit 1;)


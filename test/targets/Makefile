SUBDIRS = linux.x86 mach.x86 mach.x86_64 mingw.x86 \
          freertos.armv7m freertos.armv7m.ek-lm4f120xl \
          freertos.armv7m.dk-lm3s9d96

all:
	$(foreach dir, $(SUBDIRS), $(MAKE) -C $(dir) all || exit 1;)

clean::
	$(foreach dir, $(SUBDIRS), $(MAKE) -C $(dir) clean || exit 1;)

veryclean:: clean
	$(foreach dir, $(SUBDIRS), $(MAKE) -C $(dir) veryclean || exit 1;)


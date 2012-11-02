SUBDIRS = targets

all:
	$(foreach dir, $(SUBDIRS), $(MAKE) -C $(dir) all || exit 1;)

.PHONY:
docs:
	$(foreach dir, $(SUBDIRS), $(MAKE) -C $(dir) doc || exit 1;)

clean::
	$(foreach dir, $(SUBDIRS), $(MAKE) -C $(dir) clean || exit 1;)

veryclean:: clean
	$(foreach dir, $(SUBDIRS), $(MAKE) -C $(dir) veryclean || exit 1;)


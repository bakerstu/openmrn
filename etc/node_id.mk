ifndef _NODEID_H_INCLUDED
_NODEID_H_INCLUDED=1

ifndef ADDRESS
ADDRESS=$(DEFAULT_ADDRESS)
endif

.PHONY: FORCE

address.h: FORCE
	@ADR="#define NODEID_LOW_BITS $(ADDRESS)" ; if [ ! -f $@ ] || [ "$$ADR" != "$$(<$@)" ] ; then echo replacing address file. old: $$(<$@) new $$ADR ; echo "$$ADR" > $@ ; else echo address file up-to-date for $(TARGET):$@ ; fi 

NodeId.o: address.h

endif # include guard

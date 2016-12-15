# How to use:
#
# In the Makefile in the application target dir (normally where you include
# prog.mk) do the following:
#
# ---
# 
# DEFAULT_ADDRESS=0x101
# include $(OPENMRNPATH)/etc/node_id.mk
#
# ---
#
# Add adress.h to the .gitignore in the same directory.
#
# Then copy $(OPENMRNPATH)/etc/NodeId.cxx to theapp target directory and change
# the node ID prefix to suit your ID range.

ifndef _NODEID_H_INCLUDED
_NODEID_H_INCLUDED=1

ifndef ADDRESS
ifndef DEFAULT_ADDRESS
$(error the DEFAULT_ADDRESS has not been set for $(notdir $(realpath ../..)) target $(notdir $(realpath .)))
endif
ADDRESS=$(DEFAULT_ADDRESS)
endif

.PHONY: FORCE

address.h: FORCE
	@ADR="#define NODEID_LOW_BITS $(ADDRESS)" ; if [ ! -f $@ ] || [ "$$ADR" != "$$(<$@)" ] ; then echo $(notdir $(realpath .)): replacing address file. old: $$(<$@) new $$ADR ; echo "$$ADR" > $@ ; else echo address file up-to-date for $(notdir $(realpath .)):$@ / with $(ADDRESS); fi 

NodeId.o: address.h

endif # include guard

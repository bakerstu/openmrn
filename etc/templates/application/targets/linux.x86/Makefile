OPENMRNPATH ?= $(shell \
sh -c "if [ \"X`printenv OPENMRNPATH`\" != \"X\" ]; then printenv OPENMRNPATH; \
     elif [ -d ~/openmrn/src ]; then echo ~/openmrn; \
     elif [ -d /opt/openmrn/src ]; then echo /opt/openmrn; \
     else echo OPENMRNPATH not found; fi" \
)

##TARGET##

include $(OPENMRNPATH)/etc/prog.mk


ifndef MAKE_INC_CONFIG_MK
MAKE_INC_CONFIG_MK := 1

ifneq ($(TARGET),bare.pruv3)
CORELIBS := console utils executor os dcc nmranet

LINKCORELIBS = -lconsole -lnmranet -ldcc -lexecutor -lutils -lexecutor -los
endif

endif # MAKE_INC_CONFIG_MK

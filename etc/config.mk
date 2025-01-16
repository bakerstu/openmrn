ifndef MAKE_INC_CONFIG_MK
MAKE_INC_CONFIG_MK := 1

ifneq ($(TARGET),bare.pruv3)
CORELIBS := console utils executor os dcc openlcb withrottle ble

LINKCORELIBS = -lconsole -lopenlcb -lwithrottle -ldcc -lexecutor -lutils -lexecutor -los -lble
endif

endif # MAKE_INC_CONFIG_MK

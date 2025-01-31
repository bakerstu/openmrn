ifndef MAKE_INC_CONFIG_MK
MAKE_INC_CONFIG_MK := 1

ifneq ($(TARGET),bare.pruv3)
CORELIBS := console utils executor os dcc openlcb withrottle ble traction_modem

LINKCORELIBS = -lconsole -lopenlcb -lwithrottle -ldcc -lexecutor -lutils \
               -lexecutor -los -lble -ltraction_modem
endif

endif # MAKE_INC_CONFIG_MK

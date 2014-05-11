ifndef MAKE_INC_CONFIG_MK
MAKE_INC_CONFIG_MK := 1

CORELIBS := utils executor os if core cue dcc nmranet

LINKCORELIBS = -lnmranet -ldcc -lcue -lif -lcore $(STARTGROUP) -lexecutor -lutils -los $(ENDGROUP)

endif # MAKE_INC_CONFIG_MK

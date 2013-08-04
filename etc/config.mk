ifndef MAKE_INC_CONFIG_MK
MAKE_INC_CONFIG_MK := 1

CORELIBS := utils os if core cue nmranet
#CORELIBS := os if core

LINKCORELIBS = -lnmranet -lcue -lif -lcore  $(STARTGROUP) -lutils -los $(ENDGROUP)
#LINKCORELIBS = -lif -lcore  $(STARTGROUP) -los $(ENDGROUP)

endif # MAKE_INC_CONFIG_MK

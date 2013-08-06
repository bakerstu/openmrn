ifndef MAKE_INC_CONFIG_MK
MAKE_INC_CONFIG_MK := 1

CORELIBS := utils executor os if core cue
#CORELIBS := os if core

LINKCORELIBS = -lcue -lif -lcore  -lexecutor $(STARTGROUP) -lutils -los $(ENDGROUP)
#LINKCORELIBS = -lif -lcore  $(STARTGROUP) -los $(ENDGROUP)

endif # MAKE_INC_CONFIG_MK

-include ../../config.mk
include $(OPENMRNPATH)/etc/prog.mk
LDFLAGS += --bind -s MODULARIZE=1 -s 'EXPORT_NAME="OpenMRN"'

clean: clean-wasm


clean-wasm:
	rm -f $(EXECUTABLE).wasm $(EXECUTABLE).wast

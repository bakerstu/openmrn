-include ../../config.mk
include $(OPENMRNPATH)/etc/prog.mk
LDFLAGS += --bind -s WASM=0


# How to prepare for releasing this:
# as administrator do
# npm install -g pkg
# then you can call make release
release:
	pkg -C Brotli .


clean: clean-wasm clean-bin


clean-wasm:
	rm -f $(EXECUTABLE).{wasm,wast}

clean-bin:
	rm -f openmrn-bootloader-client-{linux,macos,win.exe}

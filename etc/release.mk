# Helper makefile for building releases of OpenMRN.


### Call this template for each binary application that should be built for a
### release. The call site should be in the toplevel makefile.
###
### Arguments: app-name target-path
###
### example: $(call RELEASE_BIN_template,hub,applications/hub/targets/linux.x86)
define RELEASE_BIN_template_helper

release-bin-all: $(RELDIR)/$(1)

$(RELDIR)/$(1): $(2)/$(1)
	strip -o $$@ $$<

$(2)/$(1):
	+$(MAKE) -C $(2)

release-clean: release-clean-$(1)

release-clean-$(1):
	+$(MAKE) -C $(2) clean rclean

endef

define RELEASE_BIN_template
$(eval $(call RELEASE_BIN_template_helper,$(1),$(2)))
endef


### Call this template for each JS application that should be built for a
### release. The call site should be in the toplevel makefile.
###
### Arguments: app-name target-path
###
### example: $(call RELEASE_JS_template,openmrn-bootloader-client,applications/bootloader_client/targets/js.emscripten)
define RELEASE_JS_template_helper

release-js-all: $(JSRELDIR)/win/$(1)-win.exe

$(JSRELDIR)/win/$(1)-win.exe: $(2)/$(1)-win.exe
	mkdir -p $(JSRELDIR)/win $(JSRELDIR)/macos
	cp $(2)/$(1)-win.exe $(JSRELDIR)/win/$(1)-win.exe
	cp $(2)/$(1)-macos $(JSRELDIR)/macos/$(1)-macos

$(2)/$(1)-win.exe:
	+$(MAKE) -C $(2)
	+$(MAKE) -C $(2) release

release-clean: release-clean-$(1)

release-clean-$(1):
	+$(MAKE) -C $(2) clean rclean
	rm -rf $(2)/$(1)-win.exe $(2)/$(1)-macos $(2)/$(1)-linux

endef

define RELEASE_JS_template
$(eval $(call RELEASE_JS_template_helper,$(1),$(2)))
endef



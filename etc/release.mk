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
	$(MAKE) -C $(2)

release-clean: release-clean-$(1)

release-clean-$(1):
	$(MAKE) -C $(2) clean rclean

endef

define RELEASE_BIN_template
$(eval $(call RELEASE_BIN_template_helper,$(1),$(2)))
endef

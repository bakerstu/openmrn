ifndef MAKE_INC_MAKE_UTILS_MK
MAKE_INC_MAKE_UTILS_MK := 1

### Helper template
### Arguments: single-target-name relative-directory-from-here
define REDIRECT_helper_template

$(1):
	+$(MAKE) -C $(2) $(1)

endef


### Call this template to alias one or more makefile targets form a different
### directory to the current directory.
###
### Argument: target(s) relative-directory-from-here
###
### Usage: at makefile toplevel do
### include $(OPENMRNPATH)/etc/make_utils.mk
### $(call REDIRECT_template,xflash rflash gdb,../bootloader)
define REDIRECT_template
$(foreach tgt,$(1),$(eval $(call REDIRECT_helper_template,$(tgt),$(2))))
endef


### Helper template to declare a dependency.
### Arguments: target_file dependency_file
### Example on how to call: Put the following on a standalone line in the Makefile
### $(foreach lib,$(LIBDIRS),$(eval $(call SUBDIR_helper_template,lib/lib$(lib).a,build-$(lib))))
define SUBDIR_helper_template

$(1)/lib$(1).a: | build-$(1)

lib/lib$(1).a: $(1)/lib$(1).a

lib/timestamp: lib/lib$(1).a

endef


endif  # MAKE_INC_MAKE_UTILS_MK

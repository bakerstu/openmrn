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


endif  # MAKE_INC_MAKE_UTILS_MK

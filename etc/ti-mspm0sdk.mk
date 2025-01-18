include $(OPENMRNPATH)/etc/path.mk

ifdef TIMSPM0SDKPATH
INCLUDES += -I$(TIMSPM0SDKPATH)/source \
	-I$(TIMSPM0SDKPATH)/source/third_party/CMSIS/Core/Include

endif

DEPS += TIMSPM0SDKPATH

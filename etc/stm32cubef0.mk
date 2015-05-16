include $(OPENMRNPATH)/etc/path.mk

ifdef STM32CUBEF0PATH
INCLUDES += -I$(OPENMRNPATH)/src/freertos_drivers/st \
            -I$(STM32CUBEF0PATH)/Drivers/STM32F0xx_HAL_Driver/Inc \
            -I$(STM32CUBEF0PATH)/Drivers/CMSIS/Device/ST/STM32F0xx/Include \
            -I$(STM32CUBEF0PATH)/Drivers/CMSIS/Include
endif

CFLAGS +=
CXXFLAGS +=

DEPS += STM32CUBEF0PATH

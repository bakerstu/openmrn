include $(OPENMRNPATH)/etc/path.mk

ifdef STM32CUBEF1PATH
INCLUDES += -I$(OPENMRNPATH)/src/freertos_drivers/st \
            -I$(STM32CUBEF1PATH)/Drivers/STM32F1xx_HAL_Driver/Inc \
            -I$(STM32CUBEF1PATH)/Drivers/CMSIS/Device/ST/STM32F1xx/Include \
            -I$(STM32CUBEF1PATH)/Drivers/CMSIS/Include
endif

CFLAGS +=
CXXFLAGS +=

DEPS += STM32CUBEF1PATH

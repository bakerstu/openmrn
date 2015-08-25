include $(OPENMRNPATH)/etc/path.mk

ifdef STM32CUBEF3PATH
INCLUDES += -I$(OPENMRNPATH)/src/freertos_drivers/st \
            -I$(STM32CUBEF3PATH)/Drivers/STM32F3xx_HAL_Driver/Inc \
            -I$(STM32CUBEF3PATH)/Drivers/CMSIS/Device/ST/STM32F3xx/Include \
            -I$(STM32CUBEF3PATH)/Drivers/CMSIS/Include
endif

CFLAGS +=
CXXFLAGS +=

DEPS += STM32CUBEF3PATH

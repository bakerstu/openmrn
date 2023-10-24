include $(OPENMRNPATH)/etc/path.mk

ifdef STM32CUBEG0PATH
INCLUDES += -I$(OPENMRNPATH)/src/freertos_drivers/st \
            -I$(STM32CUBEG0PATH)/Drivers/STM32G0xx_HAL_Driver/Inc \
            -I$(STM32CUBEG0PATH)/Drivers/CMSIS/Device/ST/STM32G0xx/Include \
            -I$(STM32CUBEG0PATH)/Drivers/CMSIS/Include
endif

CFLAGS +=
CXXFLAGS +=

DEPS += STM32CUBEG0PATH

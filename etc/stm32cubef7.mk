include $(OPENMRNPATH)/etc/path.mk

ifdef STM32CUBEF7PATH
INCLUDES += -I$(OPENMRNPATH)/src/freertos_drivers/st \
            -I$(STM32CUBEF7PATH)/Drivers/STM32F7xx_HAL_Driver/Inc \
            -I$(STM32CUBEF7PATH)/Drivers/CMSIS/Device/ST/STM32F7xx/Include \
            -I$(STM32CUBEF7PATH)/Drivers/CMSIS/Include
endif

CFLAGS += 
CXXFLAGS += 

DEPS += STM32CUBEF7PATH

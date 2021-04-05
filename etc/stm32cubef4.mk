include $(OPENMRNPATH)/etc/path.mk

ifdef STM32CUBEF4PATH
INCLUDES += -I$(OPENMRNPATH)/src/freertos_drivers/st \
            -I$(STM32CUBEF4PATH)/Drivers/STM32F4xx_HAL_Driver/Inc \
            -I$(STM32CUBEF4PATH)/Drivers/CMSIS/Device/ST/STM32F4xx/Include \
            -I$(STM32CUBEF4PATH)/Drivers/CMSIS/Include
endif

CFLAGS += 
CXXFLAGS += 

DEPS += STM32CUBEF4PATH

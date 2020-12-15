include $(OPENMRNPATH)/etc/path.mk

ifdef STM32CUBEL4PATH
INCLUDES += -I$(OPENMRNPATH)/src/freertos_drivers/st \
            -I$(STM32CUBEL4PATH)/Drivers/STM32L4xx_HAL_Driver/Inc \
            -I$(STM32CUBEL4PATH)/Drivers/CMSIS/Device/ST/STM32L4xx/Include \
            -I$(STM32CUBEL4PATH)/Drivers/CMSIS/Include
endif

CFLAGS += 
CXXFLAGS += 

DEPS += STM32CUBEL4PATH

OPENMRNPATH ?= $(abspath ../..)
#MAKE_INC_CONFIG_MK := 1
#CORELIBS =  os executor utils
#LINKCORELIBS = -los -lexecutor -lutils

include $(OPENMRNPATH)/etc/core_target.mk

SRCDIR = $(OPENMRNPATH)/src
HOST_TARGET=1

#TESTSRCS = utils/Stm32GpioTest.cxxtest

# These use MDNS.cxx which does not link for 32-bit linux host (maybe need
# avahi-client for 32-bit?).
TESTBLACKLIST += utils/OpenSSLAesCcm.cxxtest utils/SocketClient.cxxtest

# Unknown problem
TESTBLACKLIST += openlcb/MemoryConfigStream.cxxtest

# Problem with parsing and rendering dates.
TESTBLACKLIST += openlcb/BroadcastTimeDefs.cxxtest

# Problem with assumption around alignment
TESTBLACKLIST += utils/ForwardAllocator.cxxtest

include $(OPENMRNPATH)/etc/core_test.mk

clean veryclean: clean-gtest

include $(OPENMRNPATH)/etc/path.mk

ifdef STM32CUBEG0PATH

utils/Stm32GpioTest.test.o : CXXFLAGS += -DSTM32G0B1xx

utils/Stm32GpioTest.test.o : INCLUDES += \
            -I$(OPENMRNPATH)/src/freertos_drivers/common \
            -I$(OPENMRNPATH)/src/freertos_drivers/st \
            -I$(STM32CUBEG0PATH)/Drivers/STM32G0xx_HAL_Driver/Inc \
            -I$(STM32CUBEG0PATH)/Drivers/CMSIS/Device/ST/STM32G0xx/Include \
            -I$(STM32CUBEG0PATH)/Drivers/CMSIS/Include

endif


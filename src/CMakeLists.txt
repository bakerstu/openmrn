project(openmrn)

if(NOT DEFINED OPENMRNPATH)
set(OPENMRNPATH $ENV{OPENMRNPATH})
endif()

# Default option values
option(FREERTOS "freertos" OFF)
option(USE_DEVICE_FILE_SYSTEM "use device file system" OFF)
option(TARGET_BARE_ARMV7M     "bare armv7m"            OFF)
option(TARGET_ARMV7M          "bare armv7m"            OFF)
option(TARGET_BARE_ARMV6M     "bare armv6m"            OFF)
option(TARGET_ARMV6M          "bare armv6m"            OFF)
option(TARGET_SPIFFS          "spiffs"                 OFF)
option(TARGET_TM4C129         "tm4c129"                OFF)

# List of sources that get compiled.
set(SOURCES
    ${OPENMRNPATH}/src/ble/Advertisement.cxx
    ${OPENMRNPATH}/src/ble/Defs.cxx

    ${OPENMRNPATH}/src/console/Console.cxx

    ${OPENMRNPATH}/src/dcc/dcc_constants.cxx
    ${OPENMRNPATH}/src/dcc/DccDebug.cxx
    ${OPENMRNPATH}/src/dcc/Defs.cxx
    ${OPENMRNPATH}/src/dcc/LocalTrackIf.cxx
    ${OPENMRNPATH}/src/dcc/Loco.cxx
    ${OPENMRNPATH}/src/dcc/Packet.cxx
    ${OPENMRNPATH}/src/dcc/RailcomBroadcastDecoder.cxx
    ${OPENMRNPATH}/src/dcc/RailCom.cxx
    ${OPENMRNPATH}/src/dcc/RailcomDebug.cxx
    ${OPENMRNPATH}/src/dcc/SimpleUpdateLoop.cxx
    ${OPENMRNPATH}/src/dcc/UpdateLoop.cxx
    
    ${OPENMRNPATH}/src/executor/AsyncNotifiableBlock.cxx
    ${OPENMRNPATH}/src/executor/Executor.cxx
    ${OPENMRNPATH}/src/executor/Notifiable.cxx
    ${OPENMRNPATH}/src/executor/Service.cxx
    ${OPENMRNPATH}/src/executor/StateFlow.cxx
    ${OPENMRNPATH}/src/executor/Timer.cxx

    ${OPENMRNPATH}/src/openlcb/AliasAllocator.cxx
    ${OPENMRNPATH}/src/openlcb/AliasCache.cxx
    ${OPENMRNPATH}/src/openlcb/BLEAdvertisement.cxx
    ${OPENMRNPATH}/src/openlcb/BLEService.cxx
    ${OPENMRNPATH}/src/openlcb/BroadcastTime.cxx
    ${OPENMRNPATH}/src/openlcb/BroadcastTimeClient.cxx
    ${OPENMRNPATH}/src/openlcb/BroadcastTimeDefs.cxx
    ${OPENMRNPATH}/src/openlcb/BroadcastTimeServer.cxx
    ${OPENMRNPATH}/src/openlcb/BulkAliasAllocator.cxx
    ${OPENMRNPATH}/src/openlcb/CanDefs.cxx
    ${OPENMRNPATH}/src/openlcb/ConfigEntry.cxx
    ${OPENMRNPATH}/src/openlcb/ConfigUpdateFlow.cxx
    ${OPENMRNPATH}/src/openlcb/Datagram.cxx
    ${OPENMRNPATH}/src/openlcb/DatagramCan.cxx
    ${OPENMRNPATH}/src/openlcb/DatagramTcp.cxx
    ${OPENMRNPATH}/src/openlcb/DccAccyProducer.cxx
    ${OPENMRNPATH}/src/openlcb/DefaultNode.cxx
    ${OPENMRNPATH}/src/openlcb/DefaultCdi.cxx
    ${OPENMRNPATH}/src/openlcb/EventHandler.cxx
    ${OPENMRNPATH}/src/openlcb/EventHandlerContainer.cxx
    ${OPENMRNPATH}/src/openlcb/EventHandlerTemplates.cxx
    ${OPENMRNPATH}/src/openlcb/EventService.cxx
    ${OPENMRNPATH}/src/openlcb/If.cxx
    ${OPENMRNPATH}/src/openlcb/IfCan.cxx
    ${OPENMRNPATH}/src/openlcb/IfImpl.cxx
    ${OPENMRNPATH}/src/openlcb/IfTcp.cxx
    ${OPENMRNPATH}/src/openlcb/MemoryConfig.cxx
    ${OPENMRNPATH}/src/openlcb/nmranet_constants.cxx
    ${OPENMRNPATH}/src/openlcb/Node.cxx
    ${OPENMRNPATH}/src/openlcb/NodeBrowser.cxx
    ${OPENMRNPATH}/src/openlcb/NodeInitializeFlow.cxx
    ${OPENMRNPATH}/src/openlcb/NonAuthoritativeEventProducer.cxx
    ${OPENMRNPATH}/src/openlcb/PIPClient.cxx
    ${OPENMRNPATH}/src/openlcb/RoutingLogic.cxx
    ${OPENMRNPATH}/src/openlcb/SimpleNodeInfo.cxx
    ${OPENMRNPATH}/src/openlcb/SimpleNodeInfoMockUserFile.cxx
    ${OPENMRNPATH}/src/openlcb/SimpleNodeInfoResponse.cxx
    ${OPENMRNPATH}/src/openlcb/SimpleStack.cxx
    ${OPENMRNPATH}/src/openlcb/StreamReceiver.cxx
    ${OPENMRNPATH}/src/openlcb/StreamTransport.cxx
    ${OPENMRNPATH}/src/openlcb/TcpDefs.cxx
    ${OPENMRNPATH}/src/openlcb/TractionCvSpace.cxx
    ${OPENMRNPATH}/src/openlcb/TractionDefs.cxx
    ${OPENMRNPATH}/src/openlcb/TractionProxy.cxx
    ${OPENMRNPATH}/src/openlcb/TractionTestTrain.cxx
    ${OPENMRNPATH}/src/openlcb/TractionThrottle.cxx
    ${OPENMRNPATH}/src/openlcb/TractionTrain.cxx
    ${OPENMRNPATH}/src/openlcb/Velocity.cxx
    ${OPENMRNPATH}/src/openlcb/WriteHelper.cxx

    
    ${OPENMRNPATH}/src/os/FakeClock.cxx
    ${OPENMRNPATH}/src/os/logging_malloc.cxx
    ${OPENMRNPATH}/src/os/MDNS.cxx
    ${OPENMRNPATH}/src/os/os.c
    ${OPENMRNPATH}/src/os/OSImpl.cxx
    ${OPENMRNPATH}/src/os/OSSelectWakeup.cxx
    ${OPENMRNPATH}/src/os/stack_malloc.c
    ${OPENMRNPATH}/src/os/TempFile.cxx
    ${OPENMRNPATH}/src/os/watchdog.c

    ${OPENMRNPATH}/src/utils/Base64.cxx
    ${OPENMRNPATH}/src/utils/Blinker.cxx
    ${OPENMRNPATH}/src/utils/Buffer.cxx
    ${OPENMRNPATH}/src/utils/CanIf.cxx
    ${OPENMRNPATH}/src/utils/ClientConnection.cxx
    ${OPENMRNPATH}/src/utils/ConfigUpdateListener.cxx
    ${OPENMRNPATH}/src/utils/constants.cxx
    ${OPENMRNPATH}/src/utils/Crc.cxx
    ${OPENMRNPATH}/src/utils/DirectHub.cxx
    ${OPENMRNPATH}/src/utils/DirectHubGc.cxx
    ${OPENMRNPATH}/src/utils/DirectHubLegacy.cxx
    ${OPENMRNPATH}/src/utils/errno_exit.c
    ${OPENMRNPATH}/src/utils/FdUtils.cxx
    ${OPENMRNPATH}/src/utils/FileUtils.cxx
    ${OPENMRNPATH}/src/utils/format_utils.cxx
    ${OPENMRNPATH}/src/utils/ForwardAllocator.cxx
    ${OPENMRNPATH}/src/utils/GcStreamParser.cxx
    ${OPENMRNPATH}/src/utils/GcTcpHub.cxx
    ${OPENMRNPATH}/src/utils/gc_format.cxx
    ${OPENMRNPATH}/src/utils/GridConnect.cxx
    ${OPENMRNPATH}/src/utils/GridConnectHub.cxx
    ${OPENMRNPATH}/src/utils/HubDevice.cxx
    ${OPENMRNPATH}/src/utils/HubDeviceSelect.cxx
    ${OPENMRNPATH}/src/utils/ieeehalfprecision.c
    ${OPENMRNPATH}/src/utils/JSHubPort.cxx
    ${OPENMRNPATH}/src/utils/logging.cxx
    ${OPENMRNPATH}/src/utils/Queue.cxx
    ${OPENMRNPATH}/src/utils/ReflashBootloader.cxx
    ${OPENMRNPATH}/src/utils/ServiceLocator.cxx
    ${OPENMRNPATH}/src/utils/SocketCan.cxx
    ${OPENMRNPATH}/src/utils/SocketClient.cxx
    ${OPENMRNPATH}/src/utils/socket_listener.cxx
    ${OPENMRNPATH}/src/utils/Stats.cxx
    ${OPENMRNPATH}/src/utils/StringPrintf.cxx
)

if(${FREERTOS})         
set(SOURCES ${SOURCES}
    ${FREERTOSPATH}/Source/croutine.c
    ${FREERTOSPATH}/Source/event_groups.c
    ${FREERTOSPATH}/Source/list.c
    ${FREERTOSPATH}/Source/queue.c
    ${FREERTOSPATH}/Source/tasks.c
    ${FREERTOSPATH}/Source/timers.c
    ${FREERTOSPATH}/Source/portable/GCC/ARM_CM3/port.c
    ${FREERTOSPATH}/Source/portable/MemMang/heap_3.c
)
endif() # FREERTOS

if(${USE_DEVICE_FILE_SYSTEM})
set(SOURCES ${SOURCES}
    ${OPENMRNPATH}/src/freertos_drivers/common/Can.cxx
    ${OPENMRNPATH}/src/freertos_drivers/common/CpuLoad.cxx
    ${OPENMRNPATH}/src/freertos_drivers/common/Device.cxx
    ${OPENMRNPATH}/src/freertos_drivers/common/DeviceBuffer.cxx
    ${OPENMRNPATH}/src/freertos_drivers/common/DeviceFile.cxx
    ${OPENMRNPATH}/src/freertos_drivers/common/EEPROM.cxx
    ${OPENMRNPATH}/src/freertos_drivers/common/EEPROMEmulation.cxx
    ${OPENMRNPATH}/src/freertos_drivers/common/EEPROMEmulation_weak.cxx
    ${OPENMRNPATH}/src/freertos_drivers/common/Fileio.cxx
    ${OPENMRNPATH}/src/freertos_drivers/common/FileioWeak.cxx
    ${OPENMRNPATH}/src/freertos_drivers/common/FileSystem.cxx
    ${OPENMRNPATH}/src/freertos_drivers/common/I2C.cxx
    ${OPENMRNPATH}/src/freertos_drivers/common/MCP2515.cxx
    ${OPENMRNPATH}/src/freertos_drivers/common/Node.cxx
    ${OPENMRNPATH}/src/freertos_drivers/common/Null.cxx
    ${OPENMRNPATH}/src/freertos_drivers/common/PCA9685PWM.cxx
    ${OPENMRNPATH}/src/freertos_drivers/common/Pipe.cxx
    ${OPENMRNPATH}/src/freertos_drivers/common/Select.cxx
    ${OPENMRNPATH}/src/freertos_drivers/common/Serial.cxx
    ${OPENMRNPATH}/src/freertos_drivers/common/SPI.cxx
    ${OPENMRNPATH}/src/freertos_drivers/common/SPIFlash.cxx
    ${OPENMRNPATH}/src/freertos_drivers/common/SN74HC595GPO.cxx
    ${OPENMRNPATH}/src/freertos_drivers/common/Socket.cxx
    ${OPENMRNPATH}/src/freertos_drivers/common/WifiDefs.cxx
    ${OPENMRNPATH}/src/freertos_drivers/common/TCAN4550Can.cxx
)
endif() # USE_DEVICE_FILE_SYSTEM


if (${GTEST})
# List of tests
set(TESTS ${TESTS}
    ${OPENMRNPATH}/src/ble/Advertisement.cxxtest
    ${OPENMRNPATH}/src/ble/Defs.cxxtest

    ${OPENMRNPATH}/src/console/Console.cxxtest

    ${OPENMRNPATH}/src/dcc/DccDebug.cxxtest
    ${OPENMRNPATH}/src/dcc/LogonFeedback.cxxtest
    ${OPENMRNPATH}/src/dcc/Packet.cxxtest

    ${OPENMRNPATH}/src/executor/AsyncNotifiableBlock.cxxtest
    ${OPENMRNPATH}/src/executor/Dispatcher.cxxtest
    ${OPENMRNPATH}/src/executor/Notifiable.cxxtest
    ${OPENMRNPATH}/src/executor/StateFlow.cxxtest
    ${OPENMRNPATH}/src/executor/Timer.cxxtest

    ${OPENMRNPATH}/src/openlcb/AliasAllocator.cxxtest
    ${OPENMRNPATH}/src/openlcb/AliasCache.cxxtest
    ${OPENMRNPATH}/src/openlcb/BLEAdvertisement.cxxtest
    ${OPENMRNPATH}/src/openlcb/Bootloader.cxxtest
    ${OPENMRNPATH}/src/openlcb/BootloaderDg.cxxtest
    ${OPENMRNPATH}/src/openlcb/BroadcastTimeAlarm.cxxtest
    ${OPENMRNPATH}/src/openlcb/BroadcastTimeClient.cxxtest
    ${OPENMRNPATH}/src/openlcb/BroadcastTimeDefs.cxxtest
    ${OPENMRNPATH}/src/openlcb/BroadcastTimeServer.cxxtest
    ${OPENMRNPATH}/src/openlcb/CallbackEventHandler.cxxtest
    ${OPENMRNPATH}/src/openlcb/CanRoutingHub.cxxtest
    ${OPENMRNPATH}/src/openlcb/ConfigRenderer.cxxtest
    ${OPENMRNPATH}/src/openlcb/ConfigUpdateFlow.cxxtest
    ${OPENMRNPATH}/src/openlcb/DatagramCan.cxxtest
    ${OPENMRNPATH}/src/openlcb/DatagramTcp.cxxtest
    ${OPENMRNPATH}/src/openlcb/DccAccyConsumer.cxxtest
    ${OPENMRNPATH}/src/openlcb/DccAccyProducer.cxxtest
    ${OPENMRNPATH}/src/openlcb/EventHandlerContainer.cxxtest
    ${OPENMRNPATH}/src/openlcb/EventHandlerTemplates.cxxtest
    ${OPENMRNPATH}/src/openlcb/EventHandlerTemplatesConsumer.cxxtest
    ${OPENMRNPATH}/src/openlcb/EventHandlerTemplatesPC.cxxtest
    ${OPENMRNPATH}/src/openlcb/EventHandlerTemplatesProducer.cxxtest
    ${OPENMRNPATH}/src/openlcb/EventHandlerTemplatesRange.cxxtest
    ${OPENMRNPATH}/src/openlcb/EventIdentifyGlobal.cxxtest
    ${OPENMRNPATH}/src/openlcb/EventService.cxxtest
    ${OPENMRNPATH}/src/openlcb/HubLatency.cxxtest
    ${OPENMRNPATH}/src/openlcb/IfCan.cxxtest
    ${OPENMRNPATH}/src/openlcb/IfCanStress.cxxtest
    ${OPENMRNPATH}/src/openlcb/IfImpl.cxxtest
    ${OPENMRNPATH}/src/openlcb/IfTcp.cxxtest
    ${OPENMRNPATH}/src/openlcb/MemoryConfig.cxxtest
    ${OPENMRNPATH}/src/openlcb/MemoryConfigClient.cxxtest
    ${OPENMRNPATH}/src/openlcb/MemoryConfigStream.cxxtest
    ${OPENMRNPATH}/src/openlcb/NodeBrowser.cxxtest
    ${OPENMRNPATH}/src/openlcb/NodeInitializeFlow.cxxtest
    ${OPENMRNPATH}/src/openlcb/NonAuthoritativeEventProducer.cxxtest
    ${OPENMRNPATH}/src/openlcb/PIPClient.cxxtest
    ${OPENMRNPATH}/src/openlcb/PolledProducer.cxxtest
    ${OPENMRNPATH}/src/openlcb/ProtocolIdentification.cxxtest
    ${OPENMRNPATH}/src/openlcb/RefreshLoop.cxxtest
    ${OPENMRNPATH}/src/openlcb/RoutingLogic.cxxtest
    ${OPENMRNPATH}/src/openlcb/SimpleInfoProtocol.cxxtest
    ${OPENMRNPATH}/src/openlcb/SimpleNodeInfo.cxxtest
    ${OPENMRNPATH}/src/openlcb/SimpleStack.cxxtest
    ${OPENMRNPATH}/src/openlcb/SNIPClient.cxxtest
    ${OPENMRNPATH}/src/openlcb/StreamReceiver.cxxtest
    ${OPENMRNPATH}/src/openlcb/StreamSender.cxxtest
    ${OPENMRNPATH}/src/openlcb/StreamTransport.cxxtest
    ${OPENMRNPATH}/src/openlcb/TractionConsist.cxxtest
    ${OPENMRNPATH}/src/openlcb/TractionCvSpace.cxxtest
    ${OPENMRNPATH}/src/openlcb/TractionDefs.cxxtest
    ${OPENMRNPATH}/src/openlcb/TractionTestTrain.cxxtest
    ${OPENMRNPATH}/src/openlcb/TractionThrottle.cxxtest
    ${OPENMRNPATH}/src/openlcb/TractionTrain.cxxtest
    ${OPENMRNPATH}/src/openlcb/Velocity.cxxtest
    ${OPENMRNPATH}/src/openlcb/VirtualMemorySpace.cxxtest

    ${OPENMRNPATH}/src/os/FakeClock.cxxtest
    ${OPENMRNPATH}/src/os/OS.cxxtest

    ${OPENMRNPATH}/src/utils/async_if_test_helper.cxxtest
    ${OPENMRNPATH}/src/utils/BandwidthMerger.cxxtest
    ${OPENMRNPATH}/src/utils/Base64.cxxtest
    ${OPENMRNPATH}/src/utils/Blinker.cxxtest
    ${OPENMRNPATH}/src/utils/BufferQueue.cxxtest
    ${OPENMRNPATH}/src/utils/BusMaster.cxxtest
    ${OPENMRNPATH}/src/utils/ByteBuffer.cxxtest
    ${OPENMRNPATH}/src/utils/Crc.cxxtest
    ${OPENMRNPATH}/src/utils/DataBuffer.cxxtest
    ${OPENMRNPATH}/src/utils/Debouncer.cxxtest
    ${OPENMRNPATH}/src/utils/DirectHub.cxxtest
    ${OPENMRNPATH}/src/utils/DirectHubGc.cxxtest
    ${OPENMRNPATH}/src/utils/dummy.cxxtest
    ${OPENMRNPATH}/src/utils/EEPROMEmu.cxxtest
    ${OPENMRNPATH}/src/utils/EEPROMEmuWithShadow.cxxtest
    ${OPENMRNPATH}/src/utils/EntryModel.cxxtest
    ${OPENMRNPATH}/src/utils/Fixed16.cxxtest
    ${OPENMRNPATH}/src/utils/format_utils.cxxtest
    ${OPENMRNPATH}/src/utils/ForwardAllocator.cxxtest
    ${OPENMRNPATH}/src/utils/gc_format.cxxtest
    ${OPENMRNPATH}/src/utils/GcTcpHub.cxxtest
    ${OPENMRNPATH}/src/utils/GridConnect.cxxtest
    ${OPENMRNPATH}/src/utils/GridConnectHub.cxxtest
    ${OPENMRNPATH}/src/utils/HubDevice.cxxtest
    ${OPENMRNPATH}/src/utils/HubDeviceSelect.cxxtest
    ${OPENMRNPATH}/src/utils/HubStress.cxxtest
    ${OPENMRNPATH}/src/utils/LimitedPool.cxxtest
    ${OPENMRNPATH}/src/utils/LimitTimer.cxxtest
    ${OPENMRNPATH}/src/utils/LinearMap.cxxtest
    ${OPENMRNPATH}/src/utils/LruCounter.cxxtest
    ${OPENMRNPATH}/src/utils/macros_ndebug.cxxtest
    ${OPENMRNPATH}/src/utils/macros.cxxtest
    ${OPENMRNPATH}/src/utils/Map.cxxtest
    ${OPENMRNPATH}/src/utils/median.cxxtest
    ${OPENMRNPATH}/src/utils/NodeHandlerMap.cxxtest
    ${OPENMRNPATH}/src/utils/OpenSSLAesCcm.cxxtest
    ${OPENMRNPATH}/src/utils/OptionalArgs.cxxtest
    ${OPENMRNPATH}/src/utils/ScheduledQueue.cxxtest
    ${OPENMRNPATH}/src/utils/ServiceLocator.cxxtest
    ${OPENMRNPATH}/src/utils/SimpleQueue.cxxtest
    ${OPENMRNPATH}/src/utils/Singleton.cxxtest
    ${OPENMRNPATH}/src/utils/SocketClient.cxxtest
    ${OPENMRNPATH}/src/utils/SortedListMap.cxxtest
    ${OPENMRNPATH}/src/utils/StlMap.cxxtest
    ${OPENMRNPATH}/src/utils/StlMultiMap.cxxtest
    ${OPENMRNPATH}/src/utils/StoredBitSet.cxxtest
    ${OPENMRNPATH}/src/utils/SyncStream.cxxtest
    ${OPENMRNPATH}/src/utils/SysMap.cxxtest

    PARENT_SCOPE
)
endif() # ${GTEST}

# Common openmrn library
add_library(${PROJECT_NAME} ${SOURCES})

# Common openmrn includes
target_include_directories(${PROJECT_NAME}
    PUBLIC
        ${OPENMRNPATH}/src
        ${OPENMRNPATH}/include
)

include(${OPENMRNPATH}/cmake/spiffs.cmake)

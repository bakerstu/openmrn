set(SRCS
    $ENV{OPENMRNPATH}/src/dcc/dcc_constants.cxx
    $ENV{OPENMRNPATH}/src/dcc/DccDebug.cxx
    $ENV{OPENMRNPATH}/src/dcc/Defs.cxx
    $ENV{OPENMRNPATH}/src/dcc/LocalTrackIf.cxx
    $ENV{OPENMRNPATH}/src/dcc/Loco.cxx
    $ENV{OPENMRNPATH}/src/dcc/Packet.cxx
    $ENV{OPENMRNPATH}/src/dcc/RailcomBroadcastDecoder.cxx
    $ENV{OPENMRNPATH}/src/dcc/RailCom.cxx
    $ENV{OPENMRNPATH}/src/dcc/RailcomDebug.cxx
    $ENV{OPENMRNPATH}/src/dcc/SimpleUpdateLoop.cxx
    $ENV{OPENMRNPATH}/src/dcc/UpdateLoop.cxx
    
    $ENV{OPENMRNPATH}/src/executor/AsyncNotifiableBlock.cxx
    $ENV{OPENMRNPATH}/src/executor/Executor.cxx
    $ENV{OPENMRNPATH}/src/executor/Notifiable.cxx
    $ENV{OPENMRNPATH}/src/executor/Service.cxx
    $ENV{OPENMRNPATH}/src/executor/StateFlow.cxx
    $ENV{OPENMRNPATH}/src/executor/Timer.cxx

    $ENV{OPENMRNPATH}/src/freertos_drivers/esp32/Esp32HardwareI2C.cxx
    $ENV{OPENMRNPATH}/src/freertos_drivers/esp32/Esp32HardwareTwai.cxx
    $ENV{OPENMRNPATH}/src/freertos_drivers/esp32/Esp32Ledc.cxx
    $ENV{OPENMRNPATH}/src/freertos_drivers/esp32/Esp32SocInfo.cxx
    $ENV{OPENMRNPATH}/src/freertos_drivers/esp32/Esp32WiFiManager.cxx

    $ENV{OPENMRNPATH}/src/openlcb/AliasAllocator.cxx
    $ENV{OPENMRNPATH}/src/openlcb/AliasCache.cxx
    $ENV{OPENMRNPATH}/src/openlcb/BLEAdvertisement.cxx
    $ENV{OPENMRNPATH}/src/openlcb/BLEService.cxx
    $ENV{OPENMRNPATH}/src/openlcb/BroadcastTime.cxx
    $ENV{OPENMRNPATH}/src/openlcb/BroadcastTimeClient.cxx
    $ENV{OPENMRNPATH}/src/openlcb/BroadcastTimeDefs.cxx
    $ENV{OPENMRNPATH}/src/openlcb/BroadcastTimeServer.cxx
    $ENV{OPENMRNPATH}/src/openlcb/BulkAliasAllocator.cxx
    $ENV{OPENMRNPATH}/src/openlcb/CanDefs.cxx
    $ENV{OPENMRNPATH}/src/openlcb/ConfigEntry.cxx
    $ENV{OPENMRNPATH}/src/openlcb/ConfigUpdateFlow.cxx
    $ENV{OPENMRNPATH}/src/openlcb/Datagram.cxx
    $ENV{OPENMRNPATH}/src/openlcb/DatagramCan.cxx
    $ENV{OPENMRNPATH}/src/openlcb/DatagramTcp.cxx
    $ENV{OPENMRNPATH}/src/openlcb/DccAccyProducer.cxx
    $ENV{OPENMRNPATH}/src/openlcb/DefaultNode.cxx
    $ENV{OPENMRNPATH}/src/openlcb/DefaultCdi.cxx
    $ENV{OPENMRNPATH}/src/openlcb/EventHandler.cxx
    $ENV{OPENMRNPATH}/src/openlcb/EventHandlerContainer.cxx
    $ENV{OPENMRNPATH}/src/openlcb/EventHandlerTemplates.cxx
    $ENV{OPENMRNPATH}/src/openlcb/EventService.cxx
    $ENV{OPENMRNPATH}/src/openlcb/If.cxx
    $ENV{OPENMRNPATH}/src/openlcb/IfCan.cxx
    $ENV{OPENMRNPATH}/src/openlcb/IfImpl.cxx
    $ENV{OPENMRNPATH}/src/openlcb/IfTcp.cxx
    $ENV{OPENMRNPATH}/src/openlcb/MemoryConfig.cxx
    $ENV{OPENMRNPATH}/src/openlcb/nmranet_constants.cxx
    $ENV{OPENMRNPATH}/src/openlcb/Node.cxx
    $ENV{OPENMRNPATH}/src/openlcb/NodeBrowser.cxx
    $ENV{OPENMRNPATH}/src/openlcb/NodeInitializeFlow.cxx
    $ENV{OPENMRNPATH}/src/openlcb/NonAuthoritativeEventProducer.cxx
    $ENV{OPENMRNPATH}/src/openlcb/PIPClient.cxx
    $ENV{OPENMRNPATH}/src/openlcb/RoutingLogic.cxx
    $ENV{OPENMRNPATH}/src/openlcb/SimpleNodeInfo.cxx
    $ENV{OPENMRNPATH}/src/openlcb/SimpleNodeInfoMockUserFile.cxx
    $ENV{OPENMRNPATH}/src/openlcb/SimpleNodeInfoResponse.cxx
    $ENV{OPENMRNPATH}/src/openlcb/SimpleStack.cxx
    $ENV{OPENMRNPATH}/src/openlcb/StreamReceiver.cxx
    $ENV{OPENMRNPATH}/src/openlcb/StreamTransport.cxx
    $ENV{OPENMRNPATH}/src/openlcb/TcpDefs.cxx
    $ENV{OPENMRNPATH}/src/openlcb/TractionCvSpace.cxx
    $ENV{OPENMRNPATH}/src/openlcb/TractionDefs.cxx
    $ENV{OPENMRNPATH}/src/openlcb/TractionProxy.cxx
    $ENV{OPENMRNPATH}/src/openlcb/TractionTestTrain.cxx
    $ENV{OPENMRNPATH}/src/openlcb/TractionThrottle.cxx
    $ENV{OPENMRNPATH}/src/openlcb/TractionTrain.cxx
    $ENV{OPENMRNPATH}/src/openlcb/Velocity.cxx
    $ENV{OPENMRNPATH}/src/openlcb/WriteHelper.cxx

    
    $ENV{OPENMRNPATH}/src/os/FakeClock.cxx
    $ENV{OPENMRNPATH}/src/os/logging_malloc.cxx
    $ENV{OPENMRNPATH}/src/os/MDNS.cxx
    $ENV{OPENMRNPATH}/src/os/os.c
    $ENV{OPENMRNPATH}/src/os/OSImpl.cxx
    $ENV{OPENMRNPATH}/src/os/OSSelectWakeup.cxx
    $ENV{OPENMRNPATH}/src/os/stack_malloc.c
    $ENV{OPENMRNPATH}/src/os/TempFile.cxx
    $ENV{OPENMRNPATH}/src/os/watchdog.c

    $ENV{OPENMRNPATH}/src/utils/Base64.cxx
    $ENV{OPENMRNPATH}/src/utils/Blinker.cxx
    $ENV{OPENMRNPATH}/src/utils/Buffer.cxx
    $ENV{OPENMRNPATH}/src/utils/CanIf.cxx
    $ENV{OPENMRNPATH}/src/utils/ClientConnection.cxx
    $ENV{OPENMRNPATH}/src/utils/ConfigUpdateListener.cxx
    $ENV{OPENMRNPATH}/src/utils/constants.cxx
    $ENV{OPENMRNPATH}/src/utils/Crc.cxx
    $ENV{OPENMRNPATH}/src/utils/DirectHub.cxx
    $ENV{OPENMRNPATH}/src/utils/DirectHubGc.cxx
    $ENV{OPENMRNPATH}/src/utils/DirectHubLegacy.cxx
    $ENV{OPENMRNPATH}/src/utils/errno_exit.c
    $ENV{OPENMRNPATH}/src/utils/FdUtils.cxx
    $ENV{OPENMRNPATH}/src/utils/FileUtils.cxx
    $ENV{OPENMRNPATH}/src/utils/format_utils.cxx
    $ENV{OPENMRNPATH}/src/utils/ForwardAllocator.cxx
    $ENV{OPENMRNPATH}/src/utils/GcStreamParser.cxx
    $ENV{OPENMRNPATH}/src/utils/GcTcpHub.cxx
    $ENV{OPENMRNPATH}/src/utils/gc_format.cxx
    $ENV{OPENMRNPATH}/src/utils/GridConnect.cxx
    $ENV{OPENMRNPATH}/src/utils/GridConnectHub.cxx
    $ENV{OPENMRNPATH}/src/utils/HubDevice.cxx
    $ENV{OPENMRNPATH}/src/utils/HubDeviceSelect.cxx
    $ENV{OPENMRNPATH}/src/utils/ieeehalfprecision.c
    $ENV{OPENMRNPATH}/src/utils/JSHubPort.cxx
    $ENV{OPENMRNPATH}/src/utils/logging.cxx
    $ENV{OPENMRNPATH}/src/utils/Queue.cxx
    $ENV{OPENMRNPATH}/src/utils/ReflashBootloader.cxx
    $ENV{OPENMRNPATH}/src/utils/ServiceLocator.cxx
    $ENV{OPENMRNPATH}/src/utils/SocketCan.cxx
    $ENV{OPENMRNPATH}/src/utils/SocketClient.cxx
    $ENV{OPENMRNPATH}/src/utils/socket_listener.cxx
    $ENV{OPENMRNPATH}/src/utils/Stats.cxx
    $ENV{OPENMRNPATH}/src/utils/StringPrintf.cxx
)

set(INCLUDE_DIRS
    $ENV{OPENMRNPATH}/src
    $ENV{OPENMRNPATH}/include/esp-idf
    $ENV{OPENMRNPATH}/include
)

if(CONFIG_ENABLE_ARDUINO_DEPENDS)
    list(APPEND SOURCE_DIRS src)
endif()

set(IDF_DEPS
    app_update
    bootloader_support
    driver
    esp_adc
    esp_app_format
    esp_system
    esp_wifi
    espcoredump
    hal
    heap
    vfs
    mdns
    bt)

idf_component_register(SRCS "${SRCS}"
                       INCLUDE_DIRS "${INCLUDE_DIRS}"
                       REQUIRES "${IDF_DEPS}")

###############################################################################
# Suppress compilation warnings in OpenMRN
###############################################################################

target_compile_options(${COMPONENT_LIB} PUBLIC $<$<COMPILE_LANGUAGE:CXX>:-Wno-volatile>)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wall)
target_compile_options(${COMPONENT_LIB} PRIVATE -Werror)

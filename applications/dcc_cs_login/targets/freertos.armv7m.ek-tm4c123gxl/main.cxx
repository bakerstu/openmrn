/** \copyright
 * Copyright (c) 2013, Balazs Racz
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are  permitted provided that the following conditions are met:
 *
 *  - Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *  - Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \file main.cxx
 *
 * Main file for the DCC CS with Logon application on the Tiva Launchpad board.
 *
 * @author Balazs Racz
 * @date 11 Aug 2021
 */

#define LOGLEVEL INFO

#include "os/os.h"
#include "nmranet_config.h"

#include "openlcb/SimpleStack.hxx"
#include "openlcb/TractionTrain.hxx"
#include "openlcb/EventHandlerTemplates.hxx"
#include "dcc/Loco.hxx"
#include "dcc/Logon.hxx"
#include "dcc/SimpleUpdateLoop.hxx"
#include "dcc/LocalTrackIf.hxx"
#include "dcc/RailcomHub.hxx"
#include "dcc/RailcomPortDebug.hxx"
#include "executor/PoolToQueueFlow.hxx"
#include "openlcb/TractionCvSpace.hxx"

#include "freertos_drivers/ti/TivaGPIO.hxx"
#include "freertos_drivers/common/BlinkerGPIO.hxx"
#include "freertos_drivers/common/PersistentGPIO.hxx"
#include "config.hxx"
#include "hardware.hxx"
#include "TrainStorage.hxx"

#include "utils/stdio_logging.h"

// These preprocessor symbols are used to select which physical connections
// will be enabled in the main(). See @ref appl_main below.
//#define SNIFF_ON_SERIAL
#define SNIFF_ON_USB
//#define HAVE_PHYSICAL_CAN_PORT

// Changes the default behavior by adding a newline after each gridconnect
// packet. Makes it easier for debugging the raw device.
OVERRIDE_CONST(gc_generate_newlines, 1);
// Specifies how much RAM (in bytes) we allocate to the stack of the main
// thread. Useful tuning parameter in case the application runs out of memory.
OVERRIDE_CONST(main_thread_stack_size, 2500);

OVERRIDE_CONST(local_nodes_count, 30);
OVERRIDE_CONST(local_alias_cache_size, 32);

// Specifies the 48-bit OpenLCB node identifier. This must be unique for every
// hardware manufactured, so in production this should be replaced by some
// easily incrementable method.
extern const openlcb::NodeID NODE_ID = 0x0501010118DAULL;

// Sets up a comprehensive OpenLCB stack for a single virtual node. This stack
// contains everything needed for a usual peripheral node -- all
// CAN-bus-specific components, a virtual node, PIP, SNIP, Memory configuration
// protocol, ACDI, CDI, a bunch of memory spaces, etc.
openlcb::SimpleCanStack stack(NODE_ID);

// ConfigDef comes from config.hxx and is specific to the particular device and
// target. It defines the layout of the configuration memory space and is also
// used to generate the cdi.xml file. Here we instantiate the configuration
// layout. The argument of offset zero is ignored and will be removed later.
openlcb::ConfigDef cfg(0);
// Defines weak constants used by the stack to tell it which device contains
// the volatile configuration information. This device name appears in
// HwInit.cxx that creates the device drivers.
extern const char *const openlcb::CONFIG_FILENAME = "/dev/eeprom";
// The size of the memory space to export over the above device.
extern const size_t openlcb::CONFIG_FILE_SIZE =
    cfg.seg().size() + cfg.seg().offset();
static_assert(openlcb::CONFIG_FILE_SIZE <= 300, "Need to adjust eeprom size");
// The SNIP user-changeable information in also stored in the above eeprom
// device. In general this could come from different eeprom segments, but it is
// simpler to keep them together.
extern const char *const openlcb::SNIP_DYNAMIC_FILENAME =
    openlcb::CONFIG_FILENAME;

/// This timer checks the eeprom once a second and if the user has written
/// something, executes a reload of the configuration via the OpenLCB config
/// service.
class AutoUpdateTimer : public ::Timer
{
public:
    AutoUpdateTimer()
        : ::Timer(stack.executor()->active_timers())
    {
        start(SEC_TO_NSEC(1));
    }

    long long timeout() override
    {
        extern uint8_t eeprom_updated;
        if (eeprom_updated)
        {
            needUpdate_ = true;
            eeprom_updated = 0;
        }
        else
        {
            if (needUpdate_)
            {
                stack.config_service()->trigger_update();
                needUpdate_ = false;
            }
        }
        return RESTART;
    }

    bool needUpdate_ {false};
} update_timer;

// ====== Command Station components =======
OVERRIDE_CONST(num_memory_spaces, 10);

dcc::LocalTrackIf track(stack.service(), 2);
dcc::SimpleUpdateLoop updateLoop(stack.service(), &track);
PoolToQueueFlow<Buffer<dcc::Packet>> pool_translator(
    stack.service(), track.pool(), &updateLoop);

openlcb::TrainService trainService(stack.iface());

dcc::Dcc28Train train3Impl(dcc::DccShortAddress(3));
openlcb::TrainNodeForProxy train3Node(&trainService, &train3Impl);
openlcb::FixedEventProducer<openlcb::TractionDefs::IS_TRAIN_EVENT>
    trainEventProducer(&train3Node);

// ===== RailCom components ======
dcc::RailcomHubFlow railcom_hub(stack.service());
openlcb::RailcomToOpenLCBDebugProxy gRailcomProxy(&railcom_hub, stack.node(),
    nullptr, true /*broadcast enabled*/, false /* ack enabled */);

openlcb::TractionCvSpace traction_cv(stack.memory_config_handler(), &track,
    &railcom_hub, openlcb::MemoryConfigDefs::SPACE_DCC_CV);

// ===== Logon components =====
TrainLogonModule module{&trainService};
dcc::LogonHandler<TrainLogonModule> logonHandler{
        &trainService, &track, &railcom_hub, &module};

/** Entry point to application.
 * @param argc number of command line arguments
 * @param argv array of command line arguments
 * @return 0, should never return
 */
int appl_main(int argc, char *argv[])
{
    stack.check_version_and_factory_reset(
        cfg.seg().internal_config(), openlcb::CANONICAL_VERSION, false);

    int fd = ::open("/dev/mainline", O_WRONLY);
    HASSERT(fd >= 0);
    track.set_fd(fd);
    
    // The necessary physical ports must be added to the stack.
    //
    // It is okay to enable multiple physical ports, in which case the stack
    // will behave as a bridge between them. For example enabling both the
    // physical CAN port and the USB port will make this firmware act as an
    // USB-CAN adapter in addition to the producers/consumers created above.
    //
    // If a port is enabled, it must be functional or else the stack will
    // freeze waiting for that port to send the packets out.
#if defined(HAVE_PHYSICAL_CAN_PORT)
    stack.add_can_port_select("/dev/can0");
#endif
#if defined(SNIFF_ON_USB)
    stack.add_gridconnect_port("/dev/serUSB0");
#endif
#if defined(SNIFF_ON_SERIAL)
    stack.add_gridconnect_port("/dev/ser0");
#endif

    HubDeviceNonBlock<dcc::RailcomHubFlow> railcom_port(&railcom_hub,
                                                        "/dev/railcom");

    // Enables weak pull-up on the UART input pin.
    MAP_GPIOPadConfigSet(RAILCOM_CH1_Pin::GPIO_BASE, RAILCOM_CH1_Pin::GPIO_PIN,
        GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    /// @todo store the session ID and generate a new one every startup.
    logonHandler.startup_logon(0x3344, 25);

    LOG(ALWAYS, "hello world");

    // This command donates the main thread to the operation of the
    // stack. Alternatively the stack could be started in a separate stack and
    // then application-specific business logic could be executed ion a busy
    // loop in the main thread.
    stack.loop_executor();
    return 0;
}

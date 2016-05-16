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
 * An application that blinks an LED.
 *
 * @author Balazs Racz
 * @date 3 Aug 2013
 */

#include <stdio.h>
#include <unistd.h>

#include "dcc/Defs.hxx"
#include "executor/StateFlow.hxx"
#include "nmranet/EventHandlerTemplates.hxx"
#include "nmranet/SimpleStack.hxx"
#include "nmranet/TractionTrain.hxx"
#include "nmranet/TrainInterface.hxx"
#include "os/os.h"
#include "utils/ESPWifiClient.hxx"
#include "utils/blinker.h"

extern "C" {
#include <gpio.h>
#include <osapi.h>
#include <user_interface.h>
extern void ets_delay_us(uint32_t us);
#define os_delay_us ets_delay_us
}

#include "nmranet/TrainInterface.hxx"

class ESPHuzzahTrain : public nmranet::TrainImpl
{
public:
    static constexpr int f1pin = 2;
    ESPHuzzahTrain()
    {
        PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO2_U, FUNC_GPIO2);
        gpio_output_set(0, 0, (1 << f1pin), 0);
        resetblink(0);
        gpio_output_set(1 << f1pin, 0, 0, 0);
    }

    void set_speed(nmranet::SpeedType speed) override
    {
        lastSpeed_ = speed;
    }
    /** Returns the last set speed of the locomotive. */
    nmranet::SpeedType get_speed() override
    {
        return lastSpeed_;
    }

    /** Sets the train to emergency stop. */
    void set_emergencystop() override
    {
        set_speed(0);
    }

    /** Sets the value of a function.
     * @param address is a 24-bit address of the function to set. For legacy DCC
     * locomotives, see @ref TractionDefs for the address definitions (0=light,
     * 1-28= traditional function buttons).
     * @param value is the function value. For binary functions, any non-zero
     * value sets the function to on, zero sets it to off.*/
    void set_fn(uint32_t address, uint16_t value) override
    {
        switch (address)
        {
            case 0:
                f0 = value;
                resetblink(f0 ? 1 : 0);
                break;
            case 1:
                f1 = value;
                if (f1)
                {
                    gpio_output_set(0, (1 << f1pin), 0, 0);
                }
                else
                {
                    gpio_output_set((1 << f1pin), 0, 0, 0);
                }
                break;
        }
    }

    /** @returns the value of a function. */
    uint16_t get_fn(uint32_t address) override
    {
        switch (address)
        {
            case 0:
                return f0 ? 1 : 0;
                break;
            case 1:
                return f1 ? 1 : 0;
                break;
        }
        return 0;
    }

    uint32_t legacy_address() override
    {
        return 883;
    }

    /** @returns the type of legacy protocol in use. */
    dcc::TrainAddressType legacy_address_type() override
    {
        return dcc::TrainAddressType::DCC_LONG_ADDRESS;
    }

private:
    nmranet::SpeedType lastSpeed_ = 0.0;
    bool f0 = false;
    bool f1 = false;
};

namespace nmranet
{

class TrainSnipHandler : public IncomingMessageStateFlow
{
public:
    TrainSnipHandler(If *parent, SimpleInfoFlow *info_flow)
        : IncomingMessageStateFlow(parent)
        , responseFlow_(info_flow)
    {
        iface()->dispatcher()->register_handler(this,
            nmranet::Defs::MTI_IDENT_INFO_REQUEST, nmranet::Defs::MTI_EXACT);
    }
    ~TrainSnipHandler()
    {
        iface()->dispatcher()->unregister_handler(this,
            nmranet::Defs::MTI_IDENT_INFO_REQUEST, nmranet::Defs::MTI_EXACT);
    }

    Action entry() OVERRIDE
    {
        return allocate_and_call(responseFlow_, STATE(send_response_request));
    }

    Action send_response_request()
    {
        auto *b = get_allocation_result(responseFlow_);
        b->data()->reset(
            nmsg(), snipResponse_, nmranet::Defs::MTI_IDENT_INFO_REPLY);
        b->set_done(n_.reset(this));
        responseFlow_->send(b);
        release();
        return wait_and_call(STATE(send_done));
    }

    Action send_done()
    {
        return exit();
    }

private:
    SimpleInfoFlow *responseFlow_;
    BarrierNotifiable n_;
    static SimpleInfoDescriptor snipResponse_[];
};

nmranet::SimpleInfoDescriptor TrainSnipHandler::snipResponse_[] = {
    {nmranet::SimpleInfoDescriptor::LITERAL_BYTE, 4, 0, nullptr},
    {nmranet::SimpleInfoDescriptor::C_STRING, 41, 0, "Balazs Racz"},
    {nmranet::SimpleInfoDescriptor::C_STRING, 41, 0, "Dead-rail train"},
    {nmranet::SimpleInfoDescriptor::C_STRING, 21, 0, "ESP12"},
    {nmranet::SimpleInfoDescriptor::C_STRING, 21, 0, "0.1"},
    {nmranet::SimpleInfoDescriptor::LITERAL_BYTE, 2, 0, nullptr},
    {nmranet::SimpleInfoDescriptor::C_STRING, 63, 1, "E12 883"},
    {nmranet::SimpleInfoDescriptor::C_STRING, 64, 0, "No description"},
    {nmranet::SimpleInfoDescriptor::END_OF_DATA, 0, 0, 0}};

const char kFdiXml[] =
    R"(<?xml version='1.0' encoding='UTF-8'?>
<?xml-stylesheet type='text/xsl' href='xslt/fdi.xsl'?>
<fdi xmlns:xsi='http://www.w3.org/2001/XMLSchema-instance' xsi:noNamespaceSchemaLocation='http://openlcb.org/trunk/prototypes/xml/schema/fdi.xsd'>
<segment space='249'><group><name/>
<function size='1' kind='binary'>
<name>Light</name>
<number>0</number>
</function>
<function size='1' kind='binary'>
<name>BlueL</name>
<number>1</number>
</function>
</group></segment></fdi>)";

struct DeadrailStack
{
    DeadrailStack()
    {
        AddAliasAllocator(trainNode_.node_id(), &ifCan_);
        memoryConfigHandler_.registry()->insert(
            &trainNode_, MemoryConfigDefs::SPACE_FDI, &fdiBlock_);
    }

    void start_stack()
    {
        ifCan_.alias_allocator()->send(ifCan_.alias_allocator()->alloc());
    }

    Executor<5> executor_{NO_THREAD()};
    Service service_{&executor_};
    CanHubFlow canHub0_{&service_};
    IfCan ifCan_{&executor_, &canHub0_, config_local_alias_cache_size(),
        config_remote_alias_cache_size(), config_local_nodes_count()};
    InitializeFlow initFlow_{&service_};
    EventService eventService_{&ifCan_};

    TrainService tractionService_{&ifCan_};
    ESPHuzzahTrain trainImpl_;
    TrainNode trainNode_{&tractionService_, &trainImpl_};
    FixedEventProducer<nmranet::TractionDefs::IS_TRAIN_EVENT>
        isTrainEventHandler{&trainNode_};

    SimpleInfoFlow infoFlow_{&ifCan_};
    TrainSnipHandler snipHandler_{&ifCan_, &infoFlow_};

    ProtocolIdentificationHandler pip{
        &trainNode_, nmranet::Defs::SIMPLE_PROTOCOL_SUBSET |
            nmranet::Defs::DATAGRAM | nmranet::Defs::MEMORY_CONFIGURATION |
            nmranet::Defs::EVENT_EXCHANGE |
            nmranet::Defs::SIMPLE_NODE_INFORMATION |
            nmranet::Defs::TRACTION_CONTROL | nmranet::Defs::TRACTION_FDI};

    CanDatagramService datagramService_{&ifCan_,
        config_num_datagram_registry_entries(), config_num_datagram_clients()};
    MemoryConfigHandler memoryConfigHandler_{
        &datagramService_, nullptr, config_num_memory_spaces()};

    ReadOnlyMemoryBlock fdiBlock_{
        reinterpret_cast<const uint8_t *>(kFdiXml), strlen(kFdiXml)};
};

extern Pool *const g_incoming_datagram_allocator = init_main_buffer_pool();

} // namespace nmranet

nmranet::DeadrailStack stack;

/** Entry point to application.
 * @param argc number of command line arguments
 * @param argv array of command line arguments
 * @return 0, should never return
 */
int appl_main(int argc, char *argv[])
{
    new ESPWifiClient(
        "GoogleGuest", "", &stack.canHub0_, "28k.ch", 50002, []() {
            stack.executor_.thread_body();
            stack.start_stack();
        });

    return 0;
}

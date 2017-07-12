/** @copyright
 * Copyright (c) 2016, Stuart W Baker
 * All rights reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without written consent.
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
 * @file withrottle/ServerCommandLoco.cxx
 *
 * This file provides the WiThrottle Server Loco Command handler base object.
 *
 * @author Stuart Baker
 * @date 27 December 2016
 */

#include "withrottle/ServerCommandLoco.hxx"

#include <cstdio>

#include "openlcb/TractionDefs.hxx"
#include "withrottle/Server.hxx"

namespace withrottle
{

/*
 * ServerCommandLoco::ServerCommandLoco()
 */
ServerCommandLoco::ServerCommandLoco(ThrottleFlow *throttle)
    : ServerCommandBase(throttle, PRIMARY)
{
    throttle->dispatcher.register_handler(this, SECONDARY, TYPE_MASK);
    throttle->dispatcher.register_handler(this, MULTI, TYPE_MASK);
}

/*
 * ServerCommandLoco::ServerCommandLoco()
 */
ServerCommandLoco::~ServerCommandLoco()
{
    throttle->dispatcher.unregister_handler(this, SECONDARY, TYPE_MASK);
    throttle->dispatcher.unregister_handler(this, MULTI, TYPE_MASK);
}

/*
 * ServerCommandLoco::entry()
 */
StateFlowBase::Action ServerCommandLoco::entry()
{
    switch (message()->data()->commandSubType)
    {
        default:
            return release_and_exit();
        case ADDR_LONG:
            return call_immediately(STATE(address_long));
    }
}

/*
 * ServerCommandLoco::address_long()
 */
StateFlowBase::Action ServerCommandLoco::address_long()
{
    unsigned long value = strtoul(message()->data()->payload.c_str(), NULL, 0);

    if ((value == 0 && errno == EINVAL) || value > 9999)
    {
        return release_and_exit();
    }

    printf("loco: %lu\n", value);

    /** @todo need to search for train */
    openlcb::NodeID node_id = openlcb::TractionDefs::train_node_id_from_legacy(
        dcc::TrainAddressType::DCC_LONG_ADDRESS, value);

    return invoke_subflow_and_wait(&throttle->olcbThrottle, STATE(assign_train),
        openlcb::TractionThrottleCommands::ASSIGN_TRAIN, node_id, 0);
}

/*
 * ServerCommandLoco::assign_train()
 */
StateFlowBase::Action ServerCommandLoco::assign_train()
{
    auto *m = full_allocation_result(&throttle->olcbThrottle);
    switch (m->data()->resultCode)
    {
        //case openlcb::Defs::ERROR_CODE_OK:
        //    break;
        //case openlcb::Defs::ERROR_TIMEOUT:
        //case openlcb::Defs::ERROR_REJECTED:
        default:
            break;
    }
    m->unref();

    return invoke_subflow_and_wait(&throttle->olcbThrottle, STATE(load_state), 
                   openlcb::TractionThrottleCommands::LOAD_STATE);
}

/*
 * ServerCommandLoco::load_state()
 */
StateFlowBase::Action ServerCommandLoco::load_state()
{
    auto *m = full_allocation_result(&throttle->olcbThrottle);
    m->unref();

    string status = Defs::get_loco_status_string(&throttle->olcbThrottle,
                                                 message()->data()->train.c_str());

    ::write(throttle->fd, status.c_str(), status.length());
    printf("%s", status.c_str());

    return release_and_exit();
}

} /* namespace withrottle */

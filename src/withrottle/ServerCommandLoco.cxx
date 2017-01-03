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
    return release_and_exit();
}

} /* namespace withrottle */


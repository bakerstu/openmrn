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
 * @file withrottle/ServerCommand.cxx
 *
 * This file provides the WiThrottle Server Command handler base object.
 *
 * @author Stuart Baker
 * @date 27 December 2016
 */

#include "withrottle/ServerCommand.hxx"

#include "withrottle/Server.hxx"

namespace withrottle
{

/*
 * ServerCommandBase::ServerCommandBase()
 */
ServerCommandBase::ServerCommandBase(ThrottleFlow *throttle, CommandType type)
    : StateFlow<Buffer<ThrottleCommand>, QList<1>>(throttle->service())
    , throttle(throttle)
{
    throttle->dispatcher.register_handler(this, type, TYPE_MASK);
}

/*
 * ServerCommandBase::ServerCommandBase()
 */
ServerCommandBase::~ServerCommandBase()
{
    throttle->dispatcher.unregister_handler_all(this);
}

} /* namespace withrottle */

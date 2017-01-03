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
 * @file withrottle/ServerCommand.hxx
 *
 * This file provides the WiThrottle Server Command handler base object.
 *
 * @author Stuart Baker
 * @date 27 December 2016
 */

#ifndef _WITHROTTLE_SERVERCOMMAND_HXX_
#define _WITHROTTLE_SERVERCOMMAND_HXX_

#include "executor/Dispatcher.hxx"
#include "executor/StateFlow.hxx"
#include "withrottle/Defs.hxx"

namespace withrottle
{

/* forward declaration */
class ThrottleFlow;

/** WiThrottle server command handler base object.
 */
class ServerCommandBase : public StateFlow<Buffer<ThrottleCommand>, QList<1>>
{
protected:
    /** Constructor.
     * @param throttle parent throttle that this flow is acting on
     * @param type the command type belonging to this handler.
     */
    ServerCommandBase(ThrottleFlow *throttle, CommandType type);

    /** Destructor.
     */
    ~ServerCommandBase();

    /** pointer to parent throttle */
    ThrottleFlow *throttle;

private:
    DISALLOW_COPY_AND_ASSIGN(ServerCommandBase);
};

} /* namespace withrottle */

#endif /* _WITHROTTLE_SERVERCOMMAND_HXX_ */

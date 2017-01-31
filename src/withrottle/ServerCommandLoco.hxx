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
 * @file withrottle/ServerCommandLoco.hxx
 *
 * This file provides the WiThrottle Server Loco Command handler base object.
 *
 * @author Stuart Baker
 * @date 27 December 2016
 */

#ifndef _WITHROTTLE_SERVERCOMMANDLOCO_HXX_
#define _WITHROTTLE_SERVERCOMMANDLOCO_HXX_

#include "withrottle/ServerCommand.hxx"

namespace withrottle
{

/** WiThrottle server command handler base object for multi, primary, and
 * secondary locomotive.
 */
class ServerCommandLoco : public ServerCommandBase
{
public:
    /** Constructor.
     * @param throttle parent throttle that this flow is acting on
     */
    ServerCommandLoco(ThrottleFlow *throttle);

    /** Destructor.
     */
    ~ServerCommandLoco();

private:
    /** Entry point to the state machine.
     * @return next state based on the CommandSubType
     */
    StateFlowBase::Action entry() override;

    /** Handle a DCC long address sub-command.
     * @return next state assign_train
     */
    StateFlowBase::Action address_long();

    /** Handle succes or failure of assigning the train, including getting the
     * latest train state.
     * @return next state load_state()
     */
    StateFlowBase::Action assign_train();

    /** Update the LCD display with the trains current state.
     * @return next state release_and_exit()
     */
    StateFlowBase::Action load_state();

    DISALLOW_COPY_AND_ASSIGN(ServerCommandLoco);
};

} /* namespace withrottle */

#endif /* _WITHROTTLE_SERVERCOMMANDLOCO_HXX_ */

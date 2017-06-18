/** @copyright
 * Copyright (c) 2017, Stuart W Baker
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
 * @file DccAccyProducer.hxx
 *
 * Producer class that represents 2044 consecutive bits out of DCC accessory
 * control Well-Known Event ID space.
 *
 * @author Stuart Baker
 * @date 17 June 2017
 */

#ifndef _OPENLCB_DCCACCYPRODUCER_HXX_
#define _OPENLCB_DCCACCYPRODUCER_HXX_

#include "executor/CallableFlow.hxx"
#include "openlcb/NonAuthoritativeEventProducer.hxx"
#include "openlcb/TractionDefs.hxx"

namespace openlcb
{

/// Request structure used to send requests to the TractionThrottle
/// class. Contains parametrized reset calls for properly supporting
/// @ref StateFlowBase::invoke_subflow_and_wait() syntax.
struct DccAccyProducerInput : public CallableFlowRequestBase
{
    /// Possible subflow commands
    enum Command
    {
        CMD_QUERY, ///< state query
        CMD_SET, ///< state set/change
    };

    Command cmd; ///< subflow command
    uint16_t address; ///< DCC accessory address
    bool value; ///< DCC accessory address value

};

/// DCC accessory address event producer for the Well-Known DCC Accessory range.
class DccAccyProducer : public CallableFlow<DccAccyProducerInput>,
                        protected BitRangeNonAuthoritativeEventP
{
public:
    /// highest possible DCC address supported
    static constexpr uint16_t MAX_ADDRESS = 2040;

    /// Constructor.  Creates a new DCC Accessory range producer.
    ///
    /// @param node the node that the producer will be bound to
    DccAccyProducer(Node *node,
               std::function<void(unsigned, bool)> dcc_state_callback = nullptr)
        : CallableFlow<DccAccyProducerInput>(node->iface())
        , BitRangeNonAuthoritativeEventP(node,
                        TractionDefs::ACTIVATE_BASIC_DCC_ACCESSORY_EVENT_BASE,
                        TractionDefs::INACTIVATE_BASIC_DCC_ACCESSORY_EVENT_BASE,
                        MAX_ADDRESS,
                        std::bind(&DccAccyProducer::state_callback, this,
                                  std::placeholders::_1, std::placeholders::_2))
        , dccStateCallback_(dcc_state_callback)
        , writer_()
    {
    }

private:
    using Command = DccAccyProducerInput::Command;

    /// Entry point to sub-flow that dispatches the next state based on the
    /// incoming command.
    ///
    /// @return next state appropriate to the command, else
    ///         Defs::ERROR_INVALID_ARGS on error
    Action entry() override
    {
        HASSERT(input()->address > 0 && input()->address <= MAX_ADDRESS);
        switch (input()->cmd)
        {
            case Command::CMD_QUERY:
                call_immediately(STATE(send_query));
                break;
            case Command::CMD_SET:
                call_immediately(STATE(set));
                break;
            default:
                return return_with_error(Defs::ERROR_INVALID_ARGS);
        }
    }

    /// Query the last known state of the accessory address.
    ///
    /// @return waits for the query message to go out and advances to the
    ///         parent flow's next state once the memory holding the message is
    ///         freed.
    Action send_query()
    {
        send_query_consumer(input()->address - 1, &writer_, &input()->done);
        return wait_and_return_ok();
    }

    /// Set the state of the accessory address.
    ///
    /// @return waits for the query message to go out and advances to the
    ///         parent flow's next state once the memory holding the message is
    ///         freed.
    Action set()
    {
        BitRangeNonAuthoritativeEventP::set(input()->address - 1,
                                            input()->value, &writer_,
                                            &input()->done);
        return wait_and_return_ok();
    }

    /// Helper method for accessing the subflow input data
    ///
    /// @return pointer to the subflow data
    DccAccyProducerInput *input()
    {
        return message()->data();
    }

    /// Callback called when there is a notification of event (DCC accessory
    /// address) state.  Pass to the next level up after accounting for DCC
    /// address range starting at 1.
    ///
    /// @param bit bit index within event pair range
    /// @param value value of the event pair
    void state_callback(unsigned bit, bool value)
    {
        if (dccStateCallback_)
        {
            // add one to bit because the DCC address range starts at 1, not 0
            dccStateCallback_(bit + 1, value);
        }
    }

    /// Callback method that will be invoked when a consumer identified
    /// message is received with a known state.
    std::function<void(unsigned, bool)> dccStateCallback_;

    WriteHelper writer_; ///< statically allocated buffer

    DISALLOW_COPY_AND_ASSIGN(DccAccyProducer);
};

} // namespace openlcb

#endif // _OPENLCB_DCCACCYPRODUCER_HXX_

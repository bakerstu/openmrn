/** @copyright
 * Copyright (c) 2025, Stuart Baker
 * All rights reserved
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
 * @file TxFlow.hxx
 *
 * Implements the message flow for transmission.
 *
 * @author Stuart Baker
 * @date 6 Feb 2025
 */

#ifndef _TRACTION_MODEM_TXFLOW_HXX_
#define _TRACTION_MODEM_TXFLOW_HXX_

#include "traction_modem/Message.hxx"
#include "utils/logging.h"

namespace traction_modem
{

/// Public interface to aid in testing.
class TxInterface
{
public:
    /// Bind an interface to the flow to start transmitting to.
    /// @param fd interface to transmit the messages on
    virtual void start(int fd) = 0;

    /// Send a packet.
    /// @param p payload to send
    virtual void send_packet(Defs::Payload p) = 0;
};

/// Using simplification.
using TxFlowBase = StateFlow<Buffer<Message>, QList<2>>;

/// Object responsible for writing messages to the modem interface.
class TxFlow : public TxInterface, public TxFlowBase
{
public:
    /// Constructor.
    /// @param service service that the flow is bound to
    TxFlow(Service *service)
        : TxFlowBase(service)
    {
        LOG(VERBOSE, "[ModemTx] constructor");
    }

    /// Bind an interface to the flow to start transmitting to.
    /// @param fd interface to transmit the messages on, should be open with
    ///           all serial settings already applied
    void start(int fd) override
    {
        LOG(VERBOSE, "[ModemTx] fd");
        fd_ = fd;
    }

    /// Equates to a packet send.
    /// @param p payload to send
    void send_packet(Defs::Payload p) override
    {
        auto *b = alloc();
        b->data()->payload = std::move(p);
        send(b);
    }

private:
    /// Entry point to the state flow for incoming Messages to transmit.
    /// @return next state write_complete
    Action entry() override
    {
        if (fd_ < 0)
        {
            LOG(WARNING, "[ModemTx] no uart");
            return release_and_exit();
        }
        LOG(VERBOSE, "[ModemTx] msg len %d",
            (int)message()->data()->payload.size());
        return write_repeated(&helper_, fd_, message()->data()->payload.data(),
            message()->data()->payload.size(), STATE(write_complete));
    }

    /// Finish up the write and exit
    /// @return release_and_exit
    Action write_complete()
    {
        unsigned len = message()->data()->payload.size();
        unsigned num_sent = len - helper_.remaining_;
        const uint8_t *d = (const uint8_t *)message()->data()->payload.data();
        LOG(VERBOSE, "[ModemTx] sent E%d len %u done %u %08x %04x...",
            helper_.hasError_, len, num_sent, *(unsigned *)(d + 0),
            (unsigned)be32toh(*(uint16_t *)(d + 4)));
        /// @TODO check for error
        return release_and_exit();
    }

    /// Helper for performing the writes.
    StateFlowSelectHelper helper_ {this};
    /// Interface fd.
    int fd_ = -1;
};

} // namespace traction_modem

#endif // _TRACTION_MODEM_TXFLOW_HXX_
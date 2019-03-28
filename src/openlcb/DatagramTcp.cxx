/** \copyright
 * Copyright (c) 2019, Balazs Racz
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
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
 * \file DatagramTcp.cxx
 *
 * TCP-If datagram parser and renderer flows.
 *
 * @author Balazs Racz
 * @date 24 March 2019
 */

#include "openlcb/DatagramTcp.hxx"


namespace openlcb
{

class TcpDatagramClient : public DatagramClient,
                          public StateFlowBase,
                          public LinkedObject<CanDatagramClient>
{
public:
    TcpDatagramClient(IfTcp *iface)
        : StateFlowBase(iface)
    {
    }

    /** Requests cancelling the datagram send operation. Will notify the done
     * callback when the canceling is completed. */
    void cancel() override
    {
        DIE("Canceling datagram send operation is not yet implemented.");
    }

    void write_datagram(Buffer<GenMessage> *b, unsigned priority) OVERRIDE
    {
        if (!b->data()->mti)
        {
            b->data()->mti = Defs::MTI_DATAGRAM;
        }
        HASSERT(b->data()->mti == Defs::MTI_DATAGRAM);
        result_ = OPERATION_PENDING;
        reset_message(b, priority);
        start_flow(STATE(acquire_srcdst_lock));
    }

    Action acquire_srcdst_lock()
    {
        // First check if there is another datagram client sending a datagram
        // to the same target node.
        {
            AtomicHolder h(LinkedObject<TcpDatagramClient>::head_mu());
            for (TcpDatagramClient* c = LinkedObject<TcpDatagramClient>::head_;
                 c;
                 c = c->LinkedObject<TcpDatagramClient>::link_next()) {
                // this will catch c == this.
                if (!c->sendPending_) continue;
                if (c->nmsg()->src.id != nmsg()->src.id) continue; 
                if (!async_if()->matching_node(c->nmsg()->dst, nmsg()->dst))
                    continue;
                // Now: there is another datagram client sending a datagram to
                // this destination. We need to wait for that transaction to
                // complete.
                c->waitingClients_.push_front(this);
                return wait();
            }
        }
        register_handlers();
        /// @TODO(balazs.racz) this will not work for loopback messages because
        /// it calls transfer_message().
        return call_immediately(STATE(addressed_entry));
    }

private:
    IfTcp *async_if()
    {
        return static_cast<IfTcp *>(service());
    }

};

} // namespace openlcb

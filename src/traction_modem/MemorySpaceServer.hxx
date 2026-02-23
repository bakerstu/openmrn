/** @copyright
 * Copyright (c) 2026, Stuart Baker
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
 * @file MemorySpaceServer.hxx
 *
 * Logic for handling memory space read/write requests to the modem.
 *
 * @author Stuart Baker
 * @date 22 Feb 2026
 */

#ifndef _TRACTION_MODEM_MEMORYSPACESERVER_HXX_
#define _TRACTION_MODEM_MEMORYSPACESERVER_HXX_

#include "traction_modem/RxFlow.hxx"
#include "traction_modem/TxFlow.hxx"
#include "traction_modem/Message.hxx"
#include "traction_modem/ModemTrainHwInterface.hxx"

namespace traction_modem
{

/// Handler for memory space reads/writes to the modem.
class MemorySpaceServer : public PacketFlowInterface
{
public:
    /// Constructor.
    /// @param tx_flow reference to the transmit flow
    /// @param rx_flow reference to the receive flow
    /// @param hw_interface hardware specific interface to the modem train.
    MemorySpaceServer(TxInterface *tx_flow, RxInterface *rx_flow,
        ModemTrainHwInterface *hw_interface)
        : txFlow_(tx_flow)
        , rxFlow_(rx_flow)
        , hwIf_(hw_interface)
    {
        rxFlow_->register_handler(this, Defs::CMD_MEM_W);
        rxFlow_->register_handler(this, Defs::CMD_MEM_R);
    }

    /// Destructor.
    ~MemorySpaceServer()
    {
        rxFlow_->unregister_handler_all(this);
    }

private:
    /// Receive for memory write commands.
    /// @buf incoming message
    /// @prio message priority
    void send(Buffer<Message> *buf, unsigned prio) override
    {
        auto b = get_buffer_deleter(buf);
        switch (b->data()->command())
        {
            case Defs::CMD_MEM_W:
            {
                Defs::Write *wr = (Defs::Write*)b->data()->payload.data();
                size_t wr_size = be16toh(wr->header_.length_) - Defs::LEN_MEM_W;
                Defs::Payload wr_data = b->data()->payload.substr(
                    offsetof(Defs::Write, data_), wr_size);
                ModemTrainHwInterface::MemoryWriteError error =
                    hwIf_->memory_write(wr->space_, be32toh(wr->address_),
                        std::move(wr_data), &wr_size);
                txFlow_->send_packet(Defs::get_memw_resp_payload(
                    static_cast<uint16_t>(error), wr_size));
                break;
            }
            case Defs::CMD_MEM_R:
            {
                Defs::Read *rd = (Defs::Read*)b->data()->payload.data();
                size_t rd_size = rd->size_ == 0 ? 256 : rd->size_;
                Defs::Payload rd_data;
                rd_data.reserve(rd_size);
                ModemTrainHwInterface::MemoryReadError error =
                    hwIf_->memory_read(
                        rd->space_, be32toh(rd->address_), &rd_data, rd_size);
                txFlow_->send_packet(Defs::get_memr_resp_payload(
                    static_cast<uint16_t>(error), std::move(rd_data)));
                break;
            }
        }
    }

    TxInterface *txFlow_; ///< reference to the transmit flow
    RxInterface *rxFlow_; ///< reference to the receive flow
    ModemTrainHwInterface *hwIf_; ///< hardware specific interface
};

} // namespace traction_modem

#endif // _TRACTIONMODEM_MEMORYSPACESERVER_HXX_
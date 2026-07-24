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
 * @file ProgramTrack.hxx
 *
 * Logic for handling program track mode requests from the decoder.
 *
 * @author Stuart Baker
 * @date 17 Apr 2026
 */

#ifndef _TRACTION_MODEM_PROGRAMTRACK_HXX_
#define _TRACTION_MODEM_PROGRAMTRACK_HXX_

#include "traction_modem/RxFlow.hxx"
#include "traction_modem/TxFlow.hxx"
#include "traction_modem/Message.hxx"
#include "traction_modem/ModemTrainHwInterface.hxx"

namespace traction_modem
{

/// Handler for program track mode requests from the decoder. When a
/// CMD_PROGRAM_TRACK command is received, the hardware interface
/// program_track() method is called and a RESP_PROGRAM_TRACK response with
/// no data payload is sent back to the decoder.
class ProgramTrack : public PacketFlowInterface
{
public:
    /// Constructor.
    /// @param tx_flow reference to the transmit flow
    /// @param rx_flow reference to the receive flow
    /// @param hw_interface hardware specific interface to the modem train
    ProgramTrack(TxInterface *tx_flow, RxInterface *rx_flow,
        ModemTrainHwInterface *hw_interface)
        : txFlow_(tx_flow)
        , rxFlow_(rx_flow)
        , hwIf_(hw_interface)
    {
        rxFlow_->register_handler(this, Defs::CMD_PROGRAM_TRACK);
    }

    /// Destructor.
    ~ProgramTrack()
    {
        rxFlow_->unregister_handler_all(this);
    }

private:
    /// Shortcut for accessing the ModemTrainHwInterface::ProgramTrackMode.
    using ProgramTrackMode = ModemTrainHwInterface::ProgramTrackMode;

    /// Receive for program track commands.
    /// @param buf incoming message
    /// @param prio message priority
    void send(Buffer<Message> *buf, unsigned prio) override
    {
        auto b = get_buffer_deleter(buf);
        switch (b->data()->command())
        {
            case Defs::CMD_PROGRAM_TRACK:
            {
                Defs::ProgramTrackCmd *pt =
                    (Defs::ProgramTrackCmd*)b->data()->payload.data();
                hwIf_->program_track(
                    static_cast<ProgramTrackMode>(pt->mode_));
                txFlow_->send_packet(Defs::get_program_track_resp_payload());
                break;
            }
        }
    }

    TxInterface *txFlow_; ///< reference to the transmit flow
    RxInterface *rxFlow_; ///< reference to the receive flow
    ModemTrainHwInterface *hwIf_; ///< hardware specific interface
};

} // namespace traction_modem

#endif // _TRACTION_MODEM_PROGRAMTRACK_HXX_

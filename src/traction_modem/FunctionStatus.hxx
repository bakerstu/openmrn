/** @copyright
 * Copyright (c) 2025, Stuart Baker
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
 * @file FunctionStatus.hxx
 *
 * Logic for querying function status.
 *
 * @author Stuart Baker
 * @date 27 June 2025
 */

#ifndef _TRACTION_MODEM_FUNCTIONSTATUS_HXX_
#define _TRACTION_MODEM_FUNCTIONSTATUS_HXX_

#include "traction_modem/Message.hxx"

namespace traction_modem
{

// Forward declarations.
class ModemTrainHwInterface;
class TxInterface;
class RxInterface;

/// Object for querying and handling function status messages.
class FunctionStatus : public PacketFlowInterface
{
public:
    /// Constructor.
    /// @param tx_flow reference to the transmit flow
    /// @param rx_flow reference to the receive flow
    /// @param hw_interface hardware specific interface to the modem train
    FunctionStatus(TxInterface *tx_flow, RxInterface *rx_flow,
        ModemTrainHwInterface *hw_interface);

    /// Destructor.
    ~FunctionStatus();

    /// Send a function status query to the decoder.
    /// @param fn function number to query
    void query(unsigned fn);

private:
    /// Receive handler for function status response messages.
    /// @param buf incoming message
    /// @param prio message priority
    void send(Buffer<Message> *buf, unsigned prio) override;

    TxInterface *txFlow_; ///< reference to the transmit flow
    RxInterface *rxFlow_; ///< reference to the receive flow
    ModemTrainHwInterface *hwIf_; ///< hardware specific interface
};

} // namespace traction_modem

#endif // _TRACTION_MODEM_FUNCTIONSTATUS_HXX_

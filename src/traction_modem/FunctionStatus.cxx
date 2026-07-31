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
 * @file FunctionStatus.cxx
 *
 * Logic for querying function status.
 *
 * @author Stuart Baker
 * @date 27 June 2025
 */

#include "traction_modem/FunctionStatus.hxx"

#include "traction_modem/ModemTrainHwInterface.hxx"
#include "traction_modem/RxFlow.hxx"
#include "traction_modem/TxFlow.hxx"

namespace traction_modem
{

//
// FunctionStatus::FunctionStatus()
//
FunctionStatus::FunctionStatus(TxInterface *tx_flow, RxInterface *rx_flow,
    ModemTrainHwInterface *hw_interface)
    : txFlow_(tx_flow)
    , rxFlow_(rx_flow)
    , hwIf_(hw_interface)
{
    rxFlow_->register_handler(this, Defs::RESP_FN_QUERY);
}

//
// FunctionStatus::~FunctionStatus()
//
FunctionStatus::~FunctionStatus()
{
    rxFlow_->unregister_handler_all(this);
}

//
// FunctionStatus::query()
//
void FunctionStatus::query(unsigned fn)
{
    txFlow_->send_packet(Defs::get_fn_query_payload(fn));
}

//
// FunctionStatus::send()
//
void FunctionStatus::send(Buffer<Message> *buf, unsigned prio)
{
    auto b = get_buffer_deleter(buf);
    switch (b->data()->command())
    {
        case Defs::RESP_FN_QUERY:
        {
            Defs::FunctionStatusQueryResponse *fsqr =
                (Defs::FunctionStatusQueryResponse *)b->data()->payload.data();
            hwIf_->function_status(be32toh(fsqr->fn_), be16toh(fsqr->value_));
            break;
        }
    }
}

} // namespace traction_modem

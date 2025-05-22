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
 * @file Output.cxx
 *
 * Logic for interacting with outputs.
 *
 * @author Stuart Baker
 * @date 17 May 2025
 */

#include "traction_modem/Output.hxx"

#include "traction_modem/ModemTrain.hxx"

namespace traction_modem
{

//
// Output::send()
//
void Output::send(Buffer<Message> *buf, unsigned prio)
{
    auto b = get_buffer_deleter(buf);
    switch (b->data()->command())
    {
        case Defs::CMD_OUTPUT_STATE:
        {
            Defs::OutputState *os =
                (Defs::OutputState*)b->data()->payload.data();
            train_->output_state(be16toh(os->output_), be16toh(os->effect_));
            break;
        }
        case Defs::RESP_OUTPUT_STATE_QUERY:
        {
            Defs::OutputStateQueryResponse *osqr =
                (Defs::OutputStateQueryResponse*)b->data()->payload.data();
            if (osqr->error_ == 0)
            {
                train_->output_state(
                    be16toh(osqr->output_), be16toh(osqr->effect_));
            }
            break;
        }
        case Defs::CMD_OUTPUT_RESTART:
        {
            Defs::OutputRestart *o_restart =
                (Defs::OutputRestart*)b->data()->payload.data();
            train_->output_restart(be16toh(o_restart->output_));
            break;
        }
    }
}

} // namespace traction_modem
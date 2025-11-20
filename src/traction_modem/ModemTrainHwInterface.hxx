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
 * @file ModemTrainHwInterface.hxx
 *
 * Provides an abstract modem train interface for hardware specific logic
 *
 * @author Stuart Baker
 * @date 22 May 2025
 */

#ifndef _TRACTION_MODEMTRAINHWINTERFACE_HXX_
#define _TRACTION_MODEMTRAINHWINTERFACE_HXX_

namespace traction_modem
{

/// Virtual interface for hardware access needed for a ModemTrain.
class ModemTrainHwInterface
{
public:
    /// Set an output state.
    /// @param output output number
    /// @param effect 0 = off, 0xFFFF = on, else effect
    virtual void output_state(uint16_t output, uint16_t effect)
    {
    }

    /// Restart an output (synchronize lighting effect).
    /// @param output output number
    virtual void output_restart(uint16_t output)
    {
    }
};

} // namespace traction_modem

#endif // _TRACTION_MODEMTRAINHWINTERFACE_HXX_
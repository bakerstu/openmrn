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
 * @file Message.hxx
 *
 * A traction modem message type.
 *
 * @author Stuart Baker
 * @date 6 Feb 2025
 */

#ifndef _TRACTION_MODEM_MESSAGE_HXX_
#define _TRACTION_MODEM_MESSAGE_HXX_

#include "executor/StateFlow.hxx"
#include "traction_modem/Defs.hxx"

namespace traction_modem
{

/// A TractionModem message.
struct Message
{
    /// Type of the dispatcher criteria (command).
    typedef uint16_t id_type;

    /// Mask for an exact match of the ID.
    static constexpr id_type EXACT_MASK = UINT16_MAX;

    /// Test for the message being "valid".
    /// @return true if valid, else false
    bool valid() const
    {
        return Defs::is_valid(payload);
    }

    /// Extract the command from the message.
    /// @return command
    id_type command() const
    {
        return Defs::get_uint16(payload, Defs::OFS_CMD);
    }

    /// Extract the id from the message (for the dispatcher). Same as command.
    /// @return id
    id_type id()
    {
        return command();
    };

    /// Extract the length from the message.
    /// @return length in bytes, excluding header and CRC.
    uint16_t length() const
    {
        return Defs::get_uint16(payload, Defs::OFS_LEN);
    }

    /// Extract the first two bytes of a response payload as an uint16_t. This
    /// is often a status / error code.
    /// @return status and/or error code
    uint16_t response_status()
    {
        return Defs::get_uint16(payload, Defs::OFS_DATA);
    }

    /// The raw data of the message.
    std::string payload;
};

using PacketFlowInterface = FlowInterface<Buffer<Message>>;
using RxFlowBase = StateFlow<Buffer<Message>, QList<2>>;

} // namespace traction_modem

#endif // _TRACTION_MODEM_MESSAGE_HXX_
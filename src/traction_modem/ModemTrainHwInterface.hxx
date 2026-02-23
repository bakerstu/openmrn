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

#include "traction_modem/Defs.hxx"

namespace traction_modem
{

/// Virtual interface for hardware access needed for a ModemTrain.
class ModemTrainHwInterface
{
public:
    /// Output state.
    enum class OutputState : uint16_t
    {
        ON  = 0x0000, ///< output is on
        OFF = 0xFFFF, ///< output is off
    };

    /// Memory write errors.
    enum class MemoryWriteError : uint16_t
    {
        SUCCESS           = 0x0000, ///< no error occurred
        UNSUPPORTED_SPACE = 0x1001, ///< unsupported address space
        OUT_OF_BOUNDS     = 0x1002, ///< address out of bounds
        READ_ONLY         = 0x1003 ///< write to a read only address space
    };

    /// Memory read errors.
    enum class MemoryReadError : uint16_t
    {
        SUCCESS           = 0x0000, ///< no error occurred
        UNSUPPORTED_SPACE = 0x1001, ///< unsupported address space
        OUT_OF_BOUNDS     = 0x1002, ///< address out of bounds
    };

    /// Set an output state.
    /// @param output output number
    /// @param state 0 = off, 0xFFFF = on
    virtual void output_state(uint16_t output, uint16_t state)
    {
    }

    /// Restart an output (synchronize lighting effect).
    /// @param output output number
    virtual void output_restart(uint16_t output)
    {
    }

    /// Handle a memory write request.
    /// @param space address space
    /// @param address address offset within the address space
    /// @param data data to write
    /// @param size size of the data to write, caller sets to size of the data
    ///        actually written
    virtual MemoryWriteError memory_write(
        uint8_t space, uint32_t address, Defs::Payload data, size_t *size)
    {
        *size = 0;
        return MemoryWriteError::UNSUPPORTED_SPACE;
    }

    /// Handle a memory read request.
    /// @param space address space
    /// @param address address offset within the address space
    /// @param data location to copy the data to, requested size reserved
    /// @param size size of the data to read
    virtual MemoryReadError memory_read(
        uint8_t space, uint32_t address, Defs::Payload *data, size_t size)
    {
        return MemoryReadError::UNSUPPORTED_SPACE;
    }
};

/// == operator for the comparing the OutputState enum value to a uint16_t.
/// @param lhs uint16_t value
/// @param rhs OutputState value
/// @return true if equal, else false
inline bool operator==(
    const uint16_t &lhs, const ModemTrainHwInterface::OutputState &rhs)
{
    return lhs == static_cast<uint16_t>(rhs);
}

} // namespace traction_modem

#endif // _TRACTION_MODEMTRAINHWINTERFACE_HXX_
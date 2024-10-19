/** @copyright
 * Copyright (c) 2024, Stuart Baker
 * All rights reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without written consent.
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
 * @file BLEDefs.hxx
 *
 * OpenLCB BLE specific definitions.
 *
 * @author Stuart Baker
 * @date 10 March 2024
 */

#ifndef _OPENLCB_BLEDEFS_HXX_
#define _OPENLCB_BLEDEFS_HXX_

#include <memory>

#include "utils/Destructable.hxx"

namespace openlcb
{

#include <string>

/// Shared base class for protocol implementation on a per-BLE-connection
/// basis.
class BLEProtocolEngine : public Destructable {
public:
    /// Notifies the protocol engine that the connection has been
    /// terminated. The implementation must (eventually) call `delete this`.
    virtual void disconnect_and_delete() = 0;

    struct Deleter {
        void operator()(BLEProtocolEngine* e) {
            e->disconnect_and_delete();
        }
    };
};

using BLEProtocolEnginePtr = std::unique_ptr<BLEProtocolEngine, BLEProtocolEngine::Deleter>;


/// Miscellaneous BLE definitions.
class BLEDefs
{
public:
    /// Create a valid BLE random address from an OpenLCB Node ID.
    /// @param id Node ID to use
    /// @return random address
    static std::basic_string<uint8_t> random_address_from_node_id(NodeID id)
    {
        // Random addresses have 2 most significant bits set.
        id |= 0x0000C00000000000ULL;

        return std::basic_string<uint8_t>(
        {
            static_cast<uint8_t>((id >> 40) & 0xFF),
            static_cast<uint8_t>((id >> 32) & 0xFF),
            static_cast<uint8_t>((id >> 24) & 0xFF),
            static_cast<uint8_t>((id >> 16) & 0xFF),
            static_cast<uint8_t>((id >>  8) & 0xFF),
            static_cast<uint8_t>((id >>  0) & 0xFF)
        });
    }
};

} // namespace openlcb

#endif // _OPENLCB_BLEDEFS_HXX_

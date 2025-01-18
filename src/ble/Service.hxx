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
 * @file Service.hxx
 *
 * BLE Service definition interface.
 *
 * @author Stuart Baker
 * @date 2 March 2024
 */

#ifndef _BLE_SERVICE_HXX_
#define _BLE_SERVICE_HXX_

#include <cstdlib>

#include "ble/Defs.hxx"

namespace ble
{

/// BLE service definition interface
class Service
{
public:
    struct GATTAttribute
    {
        const uint8_t uuidLen_; ///< UUID_LEN_16, UUID_LEN_32, or UUID_LEN_16
        const uint8_t *uuid_; ///< 16-bit, 32-bit, or 128-bit UUID pointer
        const Defs::GATTPerm perm_; ///< permissions
        const uint16_t size_; ///< size in bytes
        const uint8_t *value_; ///< value pointer
    };
};

}

#endif // _BLE_SERVICE_HXX_

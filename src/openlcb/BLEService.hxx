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
 * @file BLEService.hxx
 *
 * OpenLCB BLE Service definition.
 *
 * @author Stuart Baker
 * @date 2 March 2024
 */

#ifndef _OPENLCB_BLESERVICE_HXX_
#define _OPENLCB_BLESERVICE_HXX_

#include <cstring>

#include "ble/Service.hxx"
#include "utils/macros.h"

namespace openlcb
{

/// OpenLCB BLE service definition.
class BLEService : public ble::Service
{
public:
    /// OpenLCB GATT service UUID.
    static const uint8_t GATT_SERVICE_UUID_OPENLCB[16];

    /// OpenLCB GATT data in characterisitic UUID.
    static const uint8_t GATT_CHAR_UUID_DATA_IN[16];

    /// OpenLCB GATT data out characterisitic UUID.
    static const uint8_t GATT_CHAR_UUID_DATA_OUT[16];

private:
    static uint8_t dataIn_[200]; ///< data incoming to the server
    static uint8_t dataOut_[200]; ///< data outgoing from the server

    static uint8_t dataInCCCD_[2]; ///< configuration descriptor for input
    static uint8_t dataOutCCCD_[2]; ///< configuration descriptor for output

public:
    /// The entry indexes for the GATT_ATTRIBUTES table.
    enum GATTAttributeIndexs
    {
        GATT_SERVICE_INDEX = 0,    ///< GATT Primary Service index
        GATT_DATA_IN_DECL_INDEX,   ///< GATT data in declaration index
        GATT_DATA_IN_VALUE_INDEX,  ///< GATT data in value index
        GATT_DATA_IN_CCCD_INDEX,   ///< GATT data in CCCD index
        GATT_DATA_OUT_DECL_INDEX,  ///< GATT data out declaration index
        GATT_DATA_OUT_VALUE_INDEX, ///< GATT data out value index
        GATT_DATA_OUT_CCCD_INDEX,  ///< GATT data out CCCD index
    };

    /// GATT Attribute table properties for this service.
    static constexpr GATTAttribute GATT_ATTRIBUTES[] =
    {
        // Primary service.
        {
            ble::Defs::UUID_LEN_16,
            ble::Defs::PRIMARY_SERVICE_UUID,
            ble::Defs::GATTPerm::READ,
            sizeof(GATT_SERVICE_UUID_OPENLCB),
            GATT_SERVICE_UUID_OPENLCB
        },
        // Characteristic declaration for data in.
        {
            ble::Defs::UUID_LEN_16,
            ble::Defs::CHAR_DECLARATOIN_UUID,
            ble::Defs::GATTPerm::READ,
            sizeof(ble::Defs::CHAR_PROP_WRITE),
            ble::Defs::CHAR_PROP_WRITE
        },
        // Characteristic value for data in.
        {
            ble::Defs::UUID_LEN_128,
            GATT_CHAR_UUID_DATA_IN,
            ble::Defs::GATTPerm::WRITE,
            sizeof(dataIn_),
            dataIn_
        },
        // Client characteristic configuration descriptor (CCCD) for data in.
        {
            ble::Defs::UUID_LEN_16,
            ble::Defs::CHAR_CLIENT_CONFIG_UUID,
            ble::Defs::GATTPerm::READ | ble::Defs::GATTPerm::WRITE,
            sizeof(dataInCCCD_),
            dataInCCCD_
        },
        // Characteristic declaration for data out.
        {
            ble::Defs::UUID_LEN_16,
            ble::Defs::CHAR_DECLARATOIN_UUID,
            ble::Defs::GATTPerm::READ,
            sizeof(ble::Defs::CHAR_PROP_READ_NOTIFY_ACK),
            ble::Defs::CHAR_PROP_READ_NOTIFY_ACK
        },
        // Characteristic value for data out.
        {
            ble::Defs::UUID_LEN_128,
            GATT_CHAR_UUID_DATA_OUT,
            ble::Defs::GATTPerm::READ,
            sizeof(dataOut_),
            dataOut_
        },
        // Client characteristic configuration descriptor (CCCD) for data out.
        {
            ble::Defs::UUID_LEN_16,
            ble::Defs::CHAR_CLIENT_CONFIG_UUID,
            ble::Defs::GATTPerm::READ | ble::Defs::GATTPerm::WRITE,
            sizeof(dataOutCCCD_),
            dataOutCCCD_
        },
    };

    /// Get the size in number of elements (Attributes).
    /// @return number of elements
    static constexpr size_t size()
    {
        return ARRAYSIZE(GATT_ATTRIBUTES);
    }
};

} // namespace ble

#endif // _OPENLCB_BLESERVICE_HXX_

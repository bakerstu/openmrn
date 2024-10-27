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
 * @file BLEAdvertisement.cxx
 *
 * OpenLCB BLE Advertisement definition.
 *
 * @author Stuart Baker
 * @date 9 March 2024
 */

#include "openlcb/BLEAdvertisement.hxx"

#include <endian.h>

#include "openlcb/BLEService.hxx"

namespace openlcb
{

//
// BLEAdvertisement::BLEAdvertisement()
//
BLEAdvertisement::BLEAdvertisement(
    NodeID node_id, const char *node_name, uint32_t pip)
    : Advertisement(ble::Advertisement::MAX_DATA_PAYLOAD_SIZE,
                    ble::Advertisement::MAX_SCAN_DATA_PAYLOAD_SIZE)
{
    // Add flags to data.
    uint8_t flags = static_cast<uint8_t>(Flags::LE_ONLY_GENERAL_DISC_MODE);
    append(Field::DATA, ble::Defs::AdvType::FLAGS, &flags, 1);
    // Add name to data.
    if (node_name && strlen(node_name) != 0)
    {
        std::string name(node_name);
        append_name(Field::DATA, name);
    }

    // Add OpenLCB service data 128 to scan data.
    pip = htole32(pip);
    node_id = htole64(node_id);
    std::basic_string<uint8_t> payload =
        std::basic_string<uint8_t>((uint8_t*)&node_id, 6) +
        std::basic_string<uint8_t>((uint8_t*)&pip, sizeof(pip));
    std::basic_string<uint8_t> srvc_data = concat_service_data_128(
        BLEService::GATT_SERVICE_UUID_OPENLCB, payload);
    append(Field::SCAN_DATA, ble::Defs::AdvType::SERVICE_DATA_128, srvc_data);
}

} // namespace openlcb

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
 * @file Defs.cxx
 *
 * BLE specific definitions.
 *
 * @author Stuart Baker
 * @date 2 March 2024
 */

#include "ble/Defs.hxx"

namespace ble
{

const uint8_t Defs::PRIMARY_SERVICE_UUID[2] = {0x00, 0x28};
const uint8_t Defs::SECONDARY_SERVICE_UUID[2] = {0x01, 0x28};
const uint8_t Defs::CHAR_DECLARATOIN_UUID[2] = {0x03, 0x28};
const uint8_t Defs::CHAR_CLIENT_CONFIG_UUID[2] = {0x02, 0x29};

const uint8_t Defs::CHAR_PROP_READ_WRITE_NOTIFY[1] =
{
    (1 << 1) | // read
    (1 << 3) | // write with response
    (1 << 4)   // notify
};

const uint8_t Defs::CHAR_PROP_READ_NOTIFY_ACK[1] =
{
    (1 << 1) | // read
    (1 << 5)   // notify with ack
};

const uint8_t Defs::CHAR_PROP_WRITE[1] =
{
    (1 << 3) // write
};

//
// Defs::adv_find_data()
//
ssize_t Defs::adv_find_data(std::basic_string<uint8_t> &adv,
                            AdvType type, uint8_t *size, unsigned instance)
{
    char t = static_cast<char>(type);

    for (size_t idx = 1; instance && idx < adv.size(); ++idx)
    {
        uint8_t len = adv[idx - 1];
        if (adv[idx] == t)
        {
            if (size)
            {
                *size = len - 1;
            }
            if (--instance == 0)
            {
                return idx - 1;
            }
        }
        // One additional added by the four loop to skip over next length.
        idx += len;
    }

    return -1;
}

//
// Defs::adv_find_name_short()
//
std::string Defs::adv_find_name_short(
    std::basic_string<uint8_t> &adv, unsigned instance)
{
    ssize_t pos;
    uint8_t size;

    pos = adv_find_data(adv, AdvType::NAME_SHORT, &size, instance);

    return (pos < 0) ?
        std::string() :
        std::string((const char*)(adv.data() + pos + 2), size);
}

//
// Defs::adv_find_name_complete()
//
std::string Defs::adv_find_name_complete(
    std::basic_string<uint8_t> &adv, unsigned instance)
{
    ssize_t pos;
    uint8_t size;

    pos = adv_find_data(adv, AdvType::NAME_COMPLETE, &size, instance);

    return (pos < 0) ?
        std::string() :
        std::string((const char*)(adv.data() + pos + 2), size);
}

//
// Defs::adv_find_service_data_128()
//
std::basic_string<uint8_t> Defs::adv_find_service_data_128(
    std::basic_string<uint8_t> &adv, const uint8_t service_uuid[16],
    unsigned instance)
{
    ssize_t pos;
    unsigned inst = 1;
    uint8_t size;

    for ( ; /* forever */ ; )
    {
        pos = adv_find_data(adv, AdvType::SERVICE_DATA_128, &size, inst);
        if (pos < 0)
        {
            // Not found.
            break;
        }
        if (adv.compare(pos + 2, 16, service_uuid, 16) == 0)
        {
            // Match.
            if (inst == instance)
            {
                // Found and instance count matches.
                return std::basic_string<uint8_t>(adv, pos + 2 + 16, size - 16);
            }
        }
        ++inst;
    }

    // Not found.
    return std::basic_string<uint8_t>();
}

} // namespace ble

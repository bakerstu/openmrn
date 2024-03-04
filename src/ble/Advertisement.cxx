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
 * @file Advertisement.cxx
 *
 * Advertisement definition.
 *
 * @author Stuart Baker
 * @date 2 March 2024
 */

#include "ble/Advertisement.hxx"

namespace ble
{

//
// Advertisement::concat_service_data_128()
//
std::string Advertisement::concat_service_data_128(
    const uint8_t uuid[16], const void *data, size_t size)
{
    std::string result((const char*)(uuid), 16);
    result.append(static_cast<const char*>(data), size);
    return result;
}

//
// Advertisement::prepend()
//
int Advertisement::prepend(
    Field field, Defs::AdvType type, const void *data, size_t size, bool clip)
{
    std::string *d;
    size_t max;
    switch (field)
    {
        default:
        case Field::DATA:
            HASSERT(!extended_);
            d = &data_;
            max = MAX_DATA_PAYLOAD_SIZE;
            break;
        case Field::SCAN_DATA:
            HASSERT(!extended_);
            d = &scanData_;
            max = MAX_SCAN_DATA_PAYLOAD_SIZE;
            break;
        case Field::EXT_DATA:
            HASSERT(extended_);
            d = &data_;
            max = MAX_EXT_DATA_PAYLOAD_SIZE;
            break;
    }

    size_t space = std::min((size + 2), (max - d->size()));
    if (space < size && clip == false)
    {
        return -1;
    }
    d->insert(0, 1, static_cast<char>(space - 1));
    d->insert(1, 1, static_cast<char>(type));
    d->insert(2, static_cast<const char*>(data), space - 2);
    return space;
}

//
// Advertisement::append()
//
int Advertisement::append(
    Field field, Defs::AdvType type, const void *data, size_t size, bool clip)
{
    std::string *d;
    size_t max;
    switch (field)
    {
        default:
        case Field::DATA:
            HASSERT(!extended_);
            d = &data_;
            max = MAX_DATA_PAYLOAD_SIZE;
            break;
        case Field::SCAN_DATA:
            HASSERT(!extended_);
            d = &scanData_;
            max = MAX_SCAN_DATA_PAYLOAD_SIZE;
            break;
        case Field::EXT_DATA:
            HASSERT(extended_);
            d = &data_;
            max = MAX_EXT_DATA_PAYLOAD_SIZE;
            break;
    }

    size_t space = std::min((size + 2), (max - d->size()));
    if (space < size && clip == false)
    {
        return -1;
    }
    d->push_back(static_cast<char>(space - 1));
    d->push_back(static_cast<char>(type));
    d->append(static_cast<const char*>(data), space - 2);
    return space;
}

//
// Advertisement::update()
//
int Advertisement::update(Field field, Defs::AdvType type, const void *data,
                          size_t size, unsigned instance, bool exact_size,
                          bool clip)
{
    std::string *d;
    size_t max;
    switch (field)
    {
        default:
        case Field::DATA:
            HASSERT(!extended_);
            d = &data_;
            max = MAX_DATA_PAYLOAD_SIZE;
            break;
        case Field::SCAN_DATA:
            HASSERT(!extended_);
            d = &scanData_;
            max = MAX_SCAN_DATA_PAYLOAD_SIZE;
            break;
        case Field::EXT_DATA:
            HASSERT(extended_);
            d = &data_;
            max = MAX_EXT_DATA_PAYLOAD_SIZE;
            break;
    }

    uint8_t len;
    size_t pos = Defs::adv_find_data(*d, type, &len, instance);
    if (pos == std::string::npos)
    {
        // No matching advertising data found.
        return -1;
    }
    return max;//pos;
}


} // namespace ble

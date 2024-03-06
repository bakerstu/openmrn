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
std::vector<uint8_t> Advertisement::concat_service_data_128(
    const uint8_t uuid[16], const void *buf, size_t size)
{
    const uint8_t *data = static_cast<const uint8_t*>(buf);
    std::vector<uint8_t> result(uuid, uuid + 16);
    result.insert(result.end(), data, data + size);
    return result;
}

//
// Advertisement::prepend()
//
int Advertisement::prepend(
    Field field, Defs::AdvType type, const void *buf, size_t size, bool clip)
{
    const uint8_t *data = static_cast<const uint8_t*>(buf);
    std::vector<uint8_t> *d;
    size_t max;

    switch (field)
    {
        default:
        case Field::DATA:
            d = &data_;
            max = extended_ ?
                MAX_EXT_DATA_PAYLOAD_SIZE : MAX_SCAN_DATA_PAYLOAD_SIZE;
            break;
        case Field::SCAN_DATA:
            HASSERT(!extended_);
            d = &scanData_;
            max = MAX_DATA_PAYLOAD_SIZE;
            break;
    }

    size_t space = std::min((size + 2), (max - d->size()));
    if (space < size && clip == false)
    {
        // Data doesn't fit and clipping is not allowed.
        return -1;
    }
    d->insert(d->begin(), data, data + (space - 2));
    d->insert(d->begin(), static_cast<uint8_t>(type));
    d->insert(d->begin(), space - 1);
    return space;
}

//
// Advertisement::append()
//
int Advertisement::append(
    Field field, Defs::AdvType type, const void *buf, size_t size, bool clip)
{
    const uint8_t *data = static_cast<const uint8_t*>(buf);
    std::vector<uint8_t> *d;
    size_t max;

    switch (field)
    {
        default:
        case Field::DATA:
            d = &data_;
            max = extended_ ?
                MAX_EXT_DATA_PAYLOAD_SIZE : MAX_SCAN_DATA_PAYLOAD_SIZE;
            break;
        case Field::SCAN_DATA:
            HASSERT(!extended_);
            d = &scanData_;
            max = MAX_DATA_PAYLOAD_SIZE;
            break;
    }

    size_t space = std::min((size + 2), (max - d->size()));
    if (space < size && clip == false)
    {
        // Data doesn't fit and clipping is not allowed.
        return -1;
    }
    d->push_back(space - 1);
    d->push_back(static_cast<uint8_t>(type));
    d->insert(d->end(), data, data + (space - 2));
    return space;
}

//
// Advertisement::update()
//
int Advertisement::update(Field field, Defs::AdvType type, const void *buf,
                          size_t size, unsigned instance, bool exact_size,
                          bool clip)
{
    const uint8_t *data = static_cast<const uint8_t*>(buf);
    std::vector<uint8_t> *d;
    size_t max;

    switch (field)
    {
        default:
        case Field::DATA:
            d = &data_;
            max = extended_ ?
                MAX_EXT_DATA_PAYLOAD_SIZE : MAX_SCAN_DATA_PAYLOAD_SIZE;
            break;
        case Field::SCAN_DATA:
            HASSERT(!extended_);
            d = &scanData_;
            max = MAX_DATA_PAYLOAD_SIZE;
            break;
    }

    uint8_t len;
    ssize_t pos = Defs::adv_find_data(*d, type, &len, instance);
    if (pos < 0)
    {
        // No matching advertising data found.
        return -1;
    }
    if (exact_size && len != size)
    {
        // Found the data, but it is the wrong size.
        return -1;
    }
    size_t space = std::min((size + 2), (max - d->size()) + (len + 1));
    if (space < size && clip == false)
    {
        // Data doesn't fit and clipping is not allowed.
        return -1;
    }
    d->at(pos) = space - 1;
    d->at(pos + 1) = static_cast<uint8_t>(type);
    d->erase(d->begin() + (pos + 2), d->begin() + (pos + 2) + (len - 1));
    d->insert(d->begin() + pos, data, data + space);
    return space;//pos;
}


} // namespace ble

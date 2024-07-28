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
 * @file Advertisement.hxx
 *
 * Advertisement definition.
 *
 * @author Stuart Baker
 * @date 2 March 2024
 */

#ifndef _BLE_ADVERTISEMENT_HXX_
#define _BLE_ADVERTISEMENT_HXX_

#include "ble/Defs.hxx"
#include "utils/macros.h"

namespace ble
{

/// Object helper to define an advertisement
class Advertisement
{
public:
    /// Maximum payload size of data.
    static constexpr size_t MAX_DATA_PAYLOAD_SIZE = 31;

    /// Maximum payload size of scan data.
    static constexpr size_t MAX_SCAN_DATA_PAYLOAD_SIZE = 31;

    /// Maximum payload size of extended data
    static constexpr size_t MAX_EXT_DATA_PAYLOAD_SIZE = 254;

    /// Data fields that make up an advertisement.
    enum Field
    {
        DATA, ///< main data
        SCAN_DATA, ///< scan data
    };

    /// Flag values.
    enum class Flags : uint8_t
    {
        LE_LIMITED_DISC_MODE      = 0x01, ///< BLE limited discovery mode
        LE_GENERAL_DISC_MODE      = 0x02, ///< BLE general discovery mode
        BR_EDR_NOT_SUPPORTED      = 0x04, ///< BR/EDR (classic) not supported
        LE_BR_EDR_CONTROLLER      = 0x08, ///< BLE + BR/EDR controller
        LE_BR_EDR_HOST            = 0x10, ///< BLE + BR/EDR Host
        LE_ONLY_LIMITED_DISC_MODE = 0x05, ///< BLE only limited discovery mode
        LE_ONLY_GENERAL_DISC_MODE = 0x06, ///< BEE only general discovery mode
    };

    /// Constructor.
    /// @param extended true if an extended advertisement, else false
    Advertisement(bool extended = false)
        : extended_(extended)
    {
    }

    /// Constructor which reserves data and scan data space. This cannot
    /// be and extended advertisement because it is assumed to have scan data.
    /// @param data_reserve size in bytes to reserve for the data
    /// @param scan_data_reserve size in bytes to reserve for the scan data
    Advertisement(size_t data_reserve, size_t scan_data_reserve)
        : extended_(false)
    {
        if (data_reserve > MAX_DATA_PAYLOAD_SIZE)
        {
            data_reserve = MAX_DATA_PAYLOAD_SIZE;
        }
        if (scan_data_reserve > MAX_SCAN_DATA_PAYLOAD_SIZE)
        {
            scan_data_reserve = MAX_SCAN_DATA_PAYLOAD_SIZE;
        }
        data_.reserve(data_reserve);
        scanData_.reserve(scan_data_reserve);
    }

    /// Constructor which reserves data space.
    /// @param data_reserve size in bytes to reserve for the data
    /// @param dummy not used, present to remove ambiguity
    /// @param extended true if an extended advertisement, else false
    Advertisement(size_t data_reserve, size_t dummy, bool extended)
        : extended_(extended)
    {
        size_t max =
            extended_ ? MAX_EXT_DATA_PAYLOAD_SIZE : MAX_DATA_PAYLOAD_SIZE;
        if (data_reserve > max)
        {
            data_reserve = max;
        }
        data_.reserve(data_reserve);
    }

    /// Concatenate a 128-bit (16-byte) UUID with provided data.
    /// @param uuid 128-bit UUID
    /// @param data data to concatenate
    /// @param size size of data in bytes to concatenate
    /// @return resulting string
    std::basic_string<uint8_t> concat_service_data_128(
        const uint8_t uuid[16], const void *buf, size_t size);

    /// Concatenate a 128-bit (16-byte) UUID with provided data.
    /// @param uuid 128-bit UUID
    /// @param data data to concatenate
    /// @return resulting string
    std::basic_string<uint8_t> concat_service_data_128(
        const uint8_t uuid[16], std::basic_string<uint8_t> &buf)
    {
        return concat_service_data_128(uuid, buf.data(), buf.size());
    }

    /// Add to the beginning of the advertisement.
    /// @param f field to place the advertisement into
    /// @param type type of data to add
    /// @param buf data to add
    /// @param size size of data in bytes
    /// @param clip if the data does not all fit, clip the end of it off
    /// @return number of bytes added, else -1 upon error
    int prepend(Field field, Defs::AdvType type, const void *buf, size_t size,
                bool clip = false);

    /// Add to the beginning of the advertisement.
    /// @param field field to place the data into
    /// @param type type of data to add
    /// @param buf data to add
    /// @param clip if the data does not all fit, clip the end of it off
    /// @return number of bytes added, else -1 upon error
    int prepend(Field field, Defs::AdvType type,
                std::basic_string<uint8_t> &buf, bool clip = false)
    {
        return prepend(field, type, buf.data(), buf.size(), clip);
    }

    /// Add name to the beginning of the advertisement. Will use type
    /// NAME_COMPLETE if it fits. Will use NAME_SHORT if it does not fit.
    /// @param field field to place the data into
    /// @param name name to add
    /// @return number of bytes added, else -1 upon error
    int prepend_name(Field field, std::string &name)
    {
        int result = prepend(
            field, Defs::AdvType::NAME_COMPLETE, name.data(), name.size());
        if (result > 0)
        {
            return result;
        }
        return prepend(
            field, Defs::AdvType::NAME_SHORT, name.data(), name.size(), true);
    }

    /// Add to the end of the advertisement.
    /// @param field field to place the data into
    /// @param type type of data to add
    /// @param buf data to add
    /// @param size size of data in bytes
    /// @param clip if the data does not all fit, clip the end of it off
    /// @return number of bytes added, else -1 upon error
    int append(Field field, Defs::AdvType type, const void *buf, size_t size,
               bool clip = false);

    /// Add to the end of the advertisement.
    /// @param field field to place the data into
    /// @param type type of data to add
    /// @param buf data to add
    /// @param clip if the data does not all fit, clip the end of it off
    /// @return number of bytes added, else -1 upon error
    int append(Field field, Defs::AdvType type, std::basic_string<uint8_t> &buf,
               bool clip = false)
    {
        return append(field, type, buf.data(), buf.size(), clip);
    }

    /// Add name to the end of the advertisement. Will use type
    /// NAME_COMPLETE if it fits. Will use NAME_SHORT if it does not fit.
    /// @param field field to place the data into
    /// @param name name to add
    /// @return number of bytes added, else -1 upon error
    int append_name(Field field, std::string &name)
    {
        int result = append(
            field, Defs::AdvType::NAME_COMPLETE, name.data(), name.size());
        if (result > 0)
        {
            return result;
        }
        return append(
            field, Defs::AdvType::NAME_SHORT, name.data(), name.size(), true);
    }

    /// Update existing advertisement.
    /// @param field field to place the data into
    /// @param type type of data to update
    /// @param buf data to update
    /// @param size size of data in bytes
    /// @param instance the instance occurance to update
    /// @param exact_size the new data size must match the old data size
    /// @param clip if the data does not all fit, clip the end of it off
    /// @return number of bytes updated, else -1 upon error
    int update(Field field, Defs::AdvType type, const void *buf, size_t size,
               unsigned instance = 1, bool exact_size = true,
               bool clip = false);

    /// Update existing advertisement.
    /// @param field field to place the data into
    /// @param type type of data to update
    /// @param buf data to update
    /// @param size size of data in bytes
    /// @param instance the instance occurance to update
    /// @param exact_size the new data size must match the old data size
    /// @param clip if the data does not all fit, clip the end of it off
    /// @return number of bytes updated, else -1 upon error
    int update(Field field, Defs::AdvType type, std::basic_string<uint8_t> &buf,
               unsigned instance = 1, bool exact_size = true,
               bool clip = false)
    {
        return update(
            field, type, buf.data(), buf.size(), instance, exact_size, clip);
    }

    /// Test if extended advertisement or not.
    /// @return true if extended advertisement, else false
    bool is_extended()
    {
        return extended_;
    }

    /// Get the advertisement data.
    /// @return pointer to the advertisement data
    uint8_t *get_data()
    {
        return (uint8_t*)data_.data();
    }

    /// Get the advertisement data size in bytes.
    /// @return size in bytes
    size_t get_data_size()
    {
        return data_.size();
    }

    /// Get the advertisement data.
    /// @return pointer to the advertisement data
    uint8_t *get_scan_data()
    {
        HASSERT(!extended_);
        return (uint8_t*)scanData_.data();
    }

    /// Get the advertisement data size in bytes.
    /// @return size in bytes
    size_t get_scan_data_size()
    {
        return extended_ ? 0 : scanData_.size();
    }

#if defined(GTEST)
    std::basic_string<uint8_t> &test_get_data()
    {
        return data_;
    }

    std::basic_string<uint8_t> &test_get_scan_data()
    {
        return scanData_;
    }
#endif

private:
    /// advertising data, also used for extended advertising
    std::basic_string<uint8_t> data_;

    ///< advertising scan data
    std::basic_string<uint8_t> scanData_;

    bool extended_; ///< true extended advertisement, else false
};

} // namespace ble

#endif // _BLE_ADVERTISEMENT_HXX_

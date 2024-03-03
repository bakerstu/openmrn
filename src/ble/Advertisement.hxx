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

#include <string>

#include "ble/Defs.hxx"
#include "utils/macros.h"

namespace ble
{

/// Object helper to define an advertisement
class Advertisement
{
public:
    /// Data fields that make up an advertisement.
    enum Field
    {
        DATA, ///< main data
        SCAN_DATA, ///< scan data
        EXT_DATA, ///< extended data
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
        LE_ONLY_GENERAL_DISC_MODE = 0x06, ///< BEE only limited discovery mode
    };

    /// Constructor.
    /// @param extended true if an extended advertisement, else false
    Advertisement(bool extended = false)
        : extended_(extended)
    {
    }

    /// Add to the beginning of the advertisement.
    /// @param f field to place the advertisement into
    /// @param type type of data to add
    /// @param data data to add
    /// @param size size of data in bytes
    /// @param clip if the data does not all fit, clip the end of it off
    /// @return number of bytes added, else -1 upon error
    int prepend(Field field, Defs::AdvType type, const void *data, size_t size,
                bool clip = false);

    /// Add to the beginning of the advertisement.
    /// @param field field to place the data into
    /// @param type type of data to add
    /// @param data data to add
    /// @param clip if the data does not all fit, clip the end of it off
    /// @return number of bytes added, else -1 upon error
    int prepend(Field field, Defs::AdvType type, std::string &data,
                bool clip = false)
    {
        return prepend(field, type, data.c_str(), data.size(), clip);
    }

    /// Add name to the beginning of the advertisement. Will use type
    /// NAME_COMPLETE if it fits. Will use NAME_SHORT if it does not fit.
    /// @param field field to place the data into
    /// @param name name to add
    /// @return number of bytes added, else -1 upon error
    int prepend_name(Field field, std::string &name)
    {
        if (prepend(field, Defs::AdvType::NAME_COMPLETE, name) > 0)
        {
            return name.size();
        }
        return prepend(field, Defs::AdvType::NAME_SHORT, name, true);
    }

    /// Add to the end of the advertisement.
    /// @param field field to place the data into
    /// @param type type of data to add
    /// @param data data to add
    /// @param size size of data in bytes
    /// @param clip if the data does not all fit, clip the end of it off
    /// @return number of bytes added, else -1 upon error
    int append(Field field, Defs::AdvType type, const void *data, size_t size,
               bool clip = false);

    /// Add to the end of the advertisement.
    /// @param field field to place the data into
    /// @param type type of data to add
    /// @param data data to add
    /// @param clip if the data does not all fit, clip the end of it off
    /// @return number of bytes added, else -1 upon error
    int append(Field field, Defs::AdvType type, std::string &data,
               bool clip = false)
    {
        return append(field, type, data.data(), data.size(), clip);
    }

    /// Add name to the end of the advertisement. Will use type
    /// NAME_COMPLETE if it fits. Will use NAME_SHORT if it does not fit.
    /// @param field field to place the data into
    /// @param name name to add
    /// @return number of bytes added, else -1 upon error
    int append_name(Field field, std::string &name)
    {
        if (append(field, Defs::AdvType::NAME_COMPLETE, name) > 0)
        {
            return name.size();
        }
        return append(field, Defs::AdvType::NAME_SHORT, name, true);
    }

    /// Update existing advertisement.
    /// @param field field to place the data into
    /// @param type type of data to update
    /// @param data data to update
    /// @param size size of data in bytes
    /// @param instance the instance occurance to update
    /// @param exact_size the new data size must match the old data size
    /// @param clip if the data does not all fit, clip the end of it off
    /// @return number of bytes updated, else -1 upon error
    int update(Field field, Defs::AdvType type, const void *data, size_t size,
               unsigned instance = 1, bool exact_size = true,
               bool clip = false);

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
        return (uint8_t*)(data_.data());
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

private:
    /// Maximum payload size of data.
    static constexpr size_t MAX_DATA_PAYLOAD_SIZE = 31;

    /// Maximum payload size of scan data.
    static constexpr size_t MAX_SCAN_DATA_PAYLOAD_SIZE = 31;

    /// Maximum payload size of extended data
    static constexpr size_t MAX_EXT_DATA_PAYLOAD_SIZE = 254;

    std::string data_; ///< advertising data, also used for extended advertising
    std::string scanData_; ///< advertising scan data

    bool extended_; ///< true extended advertisement, else false
};

} // namespace ble

#endif // _BLE_ADVERTISEMENT_HXX_

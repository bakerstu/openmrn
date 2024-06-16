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
 * @file Defs.hxx
 *
 * BLE specific definitions.
 *
 * @author Stuart Baker
 * @date 2 March 2024
 */

#ifndef _BLE_DEFS_HXX_
#define _BLE_DEFS_HXX_

#include <sys/types.h> // ssize_t on some platforms
#include <cstdint>
#include <string>

namespace ble
{

/// Miscellaneous BLE definitions.
class Defs
{
public:
    /// The value of an invalid or unitialized attribute handle.
    static constexpr uint16_t ATTR_HANDLE_INVALID = 0;

    /// The value of an invalid or unitialized connection handle.
    static constexpr uint16_t CONN_HANDLE_INVALID = 0xFFFF;

    /// The length of an address.
    static constexpr uint8_t ADDR_LEN = 6;

    /// Primary service UUID.
    static const uint8_t PRIMARY_SERVICE_UUID[2];

    /// Secondary service UUID.
    static const uint8_t SECONDARY_SERVICE_UUID[2];

    /// Characteristic UUID.
    static const uint8_t CHAR_DECLARATOIN_UUID[2];

    /// Characterisitic Client Config Descriptor (CCCD) UUID.
    static const uint8_t CHAR_CLIENT_CONFIG_UUID[2];

    /// Characteristic read/write/notify property.
    static const uint8_t CHAR_PROP_READ_WRITE_NOTIFY[1];

    /// Characteristic read/write/notify property.
    static const uint8_t CHAR_PROP_READ_NOTIFY_ACK[1];

    /// Characteristic read/write/notify property.
    static const uint8_t CHAR_PROP_WRITE[1];

    /// BLE address.
    typedef uint8_t Addr[ADDR_LEN];

    /// Address Type.
    typedef uint8_t AddrType;

    /// Connection handle.
    typedef uint16_t ConnHandle;

    typedef uint16_t AttHandle;

    /// GATT Permisions.
    enum class GATTPerm : uint8_t
    {
        READ          = 0x01, ///< attribute is readable
        WRITE         = 0x02, ///< attribute is writable
        AUTHEN_READ   = 0x04, ///< read requires authentication
        AUTHEN_WRITE  = 0x08, ///< write requires authentication
        AUTHOR_READ   = 0x10, ///< read requires authorization
        AUTHOR_WRITE  = 0x20, ///< write requires authorization
        ENCRYPT_READ  = 0x40, ///< read requires encryption
        ENCRYPT_WRITE = 0x80, ///< write requires encryption
    };

    /// UUID Sizes.
    enum
    {
        UUID_LEN_16 = 2,   ///< length in bytes of a 16-bit UUID
        UUID_LEN_32 = 4,   ///< length in bytes of a 32-bit UUID
        UUID_LEN_128 = 16, ///< length in bytes of a 128-bit UUID
    };

    /// Advertising types.
    enum class AdvType : uint8_t
    {
        FLAGS            = 0x01, ///< GAP discovery modes
        NAME_SHORT       = 0x08, ///< shortened local name
        NAME_COMPLETE    = 0x09, ///< complete local name
        SERVICE_DATA_128 = 0x21, ///< 128-bit service UUID folloed by data  
    };
    
    /// Find an advertisment data within an advertisement set.
    /// @param adv contents of the advertisement set
    /// @param type the data type to find
    /// @param size location to place the data size, size excludes type byte
    /// @param instance which instance, starting from the front, to find
    /// @return starting position of the length byte, else -1 if not found
    static ssize_t adv_find_data(std::basic_string<uint8_t> &adv, AdvType type,
        uint8_t *size, unsigned instance = 1);

    /// Find an advertisment name short type within an advertisement set.
    /// @param adv contents of the advertisement set
    /// @param instance which instance, starting from the front, to find
    /// @return string containing the name, empty string if not found.
    static std::string adv_find_name_short(
        std::basic_string<uint8_t> &adv, unsigned instance = 1);

    /// Find an advertisment name complete type within an advertisement set.
    /// @param adv contents of the advertisement set
    /// @param instance which instance, starting from the front, to find
    /// @return string containing the name, empty string if not found.
    static std::string adv_find_name_complete(
        std::basic_string<uint8_t> &adv, unsigned instance = 1);

    /// Find an advertisment service data 128 type within an advertisement set.
    /// @param adv contents of the advertisement set
    /// @param service_uuid 128-bit UUID of the service to extract data from
    /// @param instance which instance, starting from the front, to find
    /// @return basic_string containing the data, empty basic_string if not
    ///         found.
    static std::basic_string<uint8_t> adv_find_service_data_128(
        std::basic_string<uint8_t> &adv, const uint8_t service_uuid[16],
        unsigned instance = 1);
};

/// '|' operator for GATTPerm.
/// @param a left hand operand
/// @param b right hand operand
inline constexpr Defs::GATTPerm operator|(
    const Defs::GATTPerm &a, const Defs::GATTPerm &b)
{
    return static_cast<Defs::GATTPerm>(
        static_cast<uint8_t>(a) | static_cast<uint8_t>(b));
}

/// '&' operator for GATTPerm.
/// @param a left hand operand
/// @param b right hand operand
inline constexpr bool operator&(
    const Defs::GATTPerm &a, const Defs::GATTPerm &b)
{
    return static_cast<bool>(
        static_cast<uint8_t>(a) & static_cast<uint8_t>(b));
}

} // namespace ble

#endif // _BLE_DEFS_HXX_

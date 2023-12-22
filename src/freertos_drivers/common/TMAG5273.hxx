/** @copyright
 * Copyright (c) 2023, Balazs Racz
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are  permitted provided that the following conditions are met:
 *
 *  - Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *  - Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
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
 * @file TMAG5273.hxx
 * Driver for the Texas Instruments TMAG5273 I2C magnetic sensor.
 *
 * @author Balazs Racz
 * @date 28 Mar 2023
 */

#ifndef _FREERTOS_DRIVERS_COMMON_TMAG5273_HXX_
#define _FREERTOS_DRIVERS_COMMON_TMAG5273_HXX_

#include <fcntl.h>
#include <unistd.h>
#include <stropts.h>
#include <endian.h>

#include "i2c-dev.h"
#include "i2c.h"
#include "utils/Atomic.hxx"
#include "utils/logging.h"
#include "utils/macros.h"

/// Implementation of TMAG5273 sensor driver.
class TMAG5273 : private Atomic
{
public:
    /// Supported I2C addresses.
    enum I2CAddress : uint8_t
    {
        ADDR_A = 0x35, ///< A1 and A2 device address
        ADDR_B = 0x22, ///< B1 and B2 device address
        ADDR_C = 0x78, ///< C1 and C2 device address
        ADDR_D = 0x44, ///< D1 and D2 device address
    };

    /// Device Identifier.
    enum class DeviceID : uint8_t
    {
        UNKNOWN        = 0x00, ///< ID unknown or device not detected
        MT_40_AND_80   = 0x01, ///< 40-mT and 80-mT range supported
        MT_133_AND_266 = 0x02, ///< 133-mT and 266-mT range supported
        ERROR          = 0xFF, ///< some kind of error occured
    };

    /// Channels to enabled.
    enum class ChannelEnable : uint8_t
    {
        ALL_OFF    = 0x00, ///< all channels off, default
        X_ENABLE   = 0x10, ///< X channel enabled
        Y_ENABLE   = 0x20, ///< Y channel enabled
        XY_ENABLE  = 0x30, ///< X and Y channels enabled
        Z_ENABLE   = 0x40, ///< Z channel enabled
        XZ_ENABLE  = 0x50, ///< X and Z channels enabled
        YZ_ENABLE  = 0x60, ///< Y and Z channels enabled
        XYZ_ENABLE = 0x70, ///< X, Y, and Z channels enabled
        XYX_ENABLE = 0x80, ///< pseudo-simultaneous X plus Y channels enabled
        YXY_ENABLE = 0x90, ///< pseudo-simultaneous Y plus X channels enabled
        YZY_ENABLE = 0xA0, ///< pseudo-simultaneous Y plus Z channels enabled
        XZX_ENABLE = 0xB0, ///< pseudo-simultaneous X plus Z channels enabled
    };

    /// Sleep time between conversions when operating mode is wake-up and sleep
    enum class SleepTime : uint8_t
    {
        SLEEP_1_MSEC     = 0x00, ///< 1 millisecond, default
        SLEEP_5_MSEC     = 0x01, ///< 5 milliseconds
        SLEEP_10_MSEC    = 0x02, ///< 10 milliseconds
        SLEEP_15_MSEC    = 0x03, ///< 15 milliseconds
        SLEEP_20_MSEC    = 0x04, ///< 20 milliseconds
        SLEEP_30_MSEC    = 0x05, ///< 30 milliseconds
        SLEEP_50_MSEC    = 0x06, ///< 50 milliseconds
        SLEEP_100_MSEC   = 0x07, ///< 100 milliseconds
        SLEEP_500_MSEC   = 0x08, ///< 500 milliseconds
        SLEEP_1000_MSEC  = 0x09, ///< 1000 milliseconds
        SLEEP_2000_MSEC  = 0x0A, ///< 2000 milliseconds
        SLEEP_5000_MSEC  = 0x0B, ///< 5000 milliseconds
        SLEEP_20000_MSEC = 0x0C, ///< 20000 milliseconds
    };

    /// Device status bit masks.
    enum class DeviceStatus : uint8_t
    {
        /// value of the INT_N pin read at reset, 0 = low, 1 = high
        INT_N_PIN_RESET_LEVEL_MASK = 0x10,
        /// Oscillator error, 0 = no error, 1 = error, write 1 to clear
        OSC_ERROR_MASK             = 0x08,
        /// INT_N pin error, 0 = no error, 1 = error, write 1 to clear
        INT_N_PIN_ERROR_MASK       = 0x04,
        /// OTP CRC error, 0 = no error, 1 = error, write 1 to clear
        OTP_CRC_ERROR_MASK         = 0x02,
        /// VCC < 2.3V, 0 = no VCC UV, 1 = VCC UV, write 1 to clear
        VCC_UV_ERROR_MASK          = 0x01,
    };

    /// Interrupt configuration.
    enum class InterruptConfig : uint8_t
    {
        /// interrupt on conversion complete, 0 = disable, 1 = enable
        CONVERSION_COMPLETE_ENABLE = 0x80,
        /// interrupt on threshold, 0 = disabled, 1 = enabled
        THRESHOLD_ENABLE           = 0x40,
        /// interrupt state, 0 = lached until clear, 1 = pulse for 10 usec
        STATE_MASK                 = 0x20,
        /// no interrupt
        MODE_NONE                  = (0 << 2),
        /// interrupt mode through INT_N
        MODE_INT_N                 = (1 << 2),
        /// interrupt mode through INT_N, except when I2C is busy
        MODE_INT_N_EXCEPT_I2C_BUSY = (2 << 2),
        /// interrupt mode through SCL
        MODE_SCL                   = (3 << 2),
        /// interrupt mode through SCL, except when I2C is busy
        MODE_SCL_EXCEPT_I2C_BUSY   = (4 << 2),
        /// mask interrupt pin when INT_N is connected to ground
        MASK_INT_N_MASK            = 0x01,
    };

    /// conversion status.
    enum class ConversionStatus : uint8_t
    {
        /// rolling count of conversion data sets mask
        SET_COUNT_MASK     = 0xE0,
        /// rolling count of conversion data sets shift
        SET_COUNT_SHIFT    = 5,
        /// Device powered up, 0 = no power on reset, 1 = power on reset
        POR_MASK           = 0x10,
        /// Diagnositic status, 0 = no diag fail, 1 diag fail detected
        DIAG_STATUS_MASK   = 0x02,
        /// Conversion, 0 = conversion not complete, 1 = conversion complete
        RESULT_STATUS_MASK = 0x01,
    };

    /// Conversion oversampling average configuration
    enum class ConversionAverage : uint8_t
    {
        AVG_1  = (0 << 2), ///< average 1 saple
        AVG_2  = (1 << 2), ///< average 2 saples
        AVG_4  = (2 << 2), ///< average 4 saples
        AVG_8  = (3 << 2), ///< average 8 saples
        AVG_16 = (4 << 2), ///< average 16 saples
        AVG_32 = (5 << 2), ///< average 32 saples
    };

    /// Angle calculation to enable.
    enum class AngleEnable : uint8_t
    {
        OFF        = (0 << 2), ///< angle not enabled
        XY_ENABLE  = (1 << 2), ///< X + Y derived angle
        YZ_ENABLE  = (2 << 2), ///< Y + Z derived angle
        XZ_ENABLE  = (3 << 2), ///< X + Z derived angle
    };

    /// Operating modes of the device.
    enum class OperatingMode : uint8_t
    {
        STANDBY           = 0x00, ///< starts new conversion at trigger event
        SLEEP             = 0x01, ///< sleep
        CONTINUOUS        = 0x02, ///< continuous measurement mode
        WAKE_UP_AND_SLEEP = 0x03, ///< wake-up and sleep mode
    };

    /// Constructor.
    /// @param address is the 7-bit address (right aligned), typically 0x35,
    ///        0x22, 0x78 or 0x44. Default is the A1/A2 device.
    TMAG5273(uint8_t address = ADDR_A)
        : i2cAddress_(address)
    { }

    /// Constructor. Can only be called from thread context.
    /// @param i2c_path path to the i2c bus to use.
    /// @param address is the 7-bit address (right aligned), typically 0x35,
    ///        0x22, 0x78 or 0x44. Default is the A1/A2 device.
    TMAG5273(const char *i2c_path, uint8_t address = ADDR_A)
        : i2cAddress_(address)
    {
        init(i2c_path);
    }

    /// Destructor.
    ~TMAG5273()
    {
        if (fd_ >= 0)
        {
            ::close(fd_);
        }
    }

    void init(const char *i2c_path)
    {
        int fd = ::open(i2c_path, O_RDWR);
        HASSERT(fd >= 0);
        init(fd);
    }

    /// Initializes the device.
    /// @param i2c_fd is the file descriptor of the i2c port to use.
    void init(int i2c_fd)
    {
        fd_ = i2c_fd;
    }

    friend class MagSensorTest;
    friend class Input;

    /// Checks whether the magnetic sensor is present.
    /// @return device identifier, else ID_UNKNOWN if not detected or unknown
    ///         device
    DeviceID is_present()
    {
        uint8_t rc[3];
        // We start with a dummy read to ensure I2C is in a known state.
        register_read(TMAG5273::DEVICE_ID, rc, 3);
        if (register_read(TMAG5273::DEVICE_ID, rc, 3) < 0)
        {
            // Error reading from i2c.
            LOG(VERBOSE, "error read");
            return DeviceID::ERROR;
        }
        // mask off the reserved bits of the device ID.
        rc[0] &= DEVICE_ID_VERSION_MASK;
        if ((rc[0] != static_cast<uint8_t>(DeviceID::MT_40_AND_80) &&
             rc[0] != static_cast<uint8_t>(DeviceID::MT_133_AND_266)) ||
            rc[1] != 0x49 || rc[2] != 0x54)
        {
            LOG(VERBOSE, "read %x %x %x", rc[0], rc[1], rc[2]);
            return DeviceID::UNKNOWN;
        }
        return static_cast<DeviceID>(rc[0]);
    }

    /// Get the device status.
    /// @return bit mask of DeviceStatus
    DeviceStatus get_device_status()
    {
        uint8_t result;
        register_read(DEVICE_STATUS, &result, 1);
        return static_cast<DeviceStatus>(result);
    }

    /// Get the device status.
    /// @param bit mask of DeviceStatus bits to clear
    void clr_device_status(DeviceStatus status)
    {
        register_write(DEVICE_STATUS, static_cast<uint8_t>(status));
    }

    /// Enable/disable channels.
    /// @param enable The channel set to enable. Default is all channels off.
    void enable_channels(ChannelEnable enable)
    {
        register_modify(
            SENSOR_CONFIG_1, SCONF1_MAG_CH_MASK, static_cast<uint8_t>(enable));
    }

    /// Set the sleep time between channels in wake and sleep mode.
    /// @param t The sleep time between conversions.
    void set_sleep_time(SleepTime t)
    {
        register_modify(SENSOR_CONFIG_1, SCONF1_SLEEP_TIME_MASK,
            static_cast<uint8_t>(t));
    }

    /// Set the interrupt config.
    /// @param config InterruptConfig options or'd together.
    void set_interrupt_config(InterruptConfig config)
    {
        register_write(INT_CONFIG_1, static_cast<uint8_t>(config));
    }

    /// Set the operating mode.
    /// @param mode operating mode to set
    void set_operating_mode(OperatingMode mode)
    {
        register_modify(DEVICE_CONFIG_2, DCONF2_OPERATING_MODE_MASK,
            static_cast<uint8_t>(mode));
    }

    /// Get the conversion status.
    /// @return mask of ConversionStatus bits
    uint8_t get_conversion_status()
    {
        uint8_t result;
        register_read(CONV_STATUS, &result, 1);
        return result;
    }

    /// Clear the power on reset bit of the conversion status.
    void clr_por_conversion_status()
    {
        register_write(
            CONV_STATUS, static_cast<uint8_t>(ConversionStatus::POR_MASK));
    }

    /// Sets the oversampling+averaging mode.
    /// @param avg average oversample setting
    void set_oversampling(ConversionAverage avg)
    {
        register_modify(
            DEVICE_CONFIG_1, DCONF1_AVG_MASK, static_cast<uint8_t>(avg));
    }

    /// Sets whether angle measurement should be enabled.
    /// @param angle angle measurement to enabld (or disable)
    void set_angle_en(AngleEnable angle)
    {
        register_modify(
            SENSOR_CONFIG_2, SCONF2_ANGLE_EN_MASK, static_cast<uint8_t>(angle));
    }

    /// Sets angle gain parameters.
    ///
    /// @param gain 8-bit gain value (interpreted as 0..1)
    /// @param second true if the second channel is larger than the first
    /// channel
    void set_angle_gain(uint8_t gain, bool second)
    {
        register_write(MAG_GAIN_CONFIG, gain);
        register_modify(SENSOR_CONFIG_2, SCONF2_MAG_GAIN_CH_MASK,
            second ? SCONF2_MAG_GAIN_CH_ADJ2 : SCONF2_MAG_GAIN_CH_ADJ1);
    }

    /// Sets offset correction for the first axis. The maximum correction is
    /// -2048 .. +2032.
    ///
    /// @param offset 8-bit signed value to write to offset correction. The
    /// counts offset are offset * 16.
    ///
    void set_offset_1(int8_t offset)
    {
        register_write(MAG_OFFSET_CONFIG_1, offset);
    }

    /// Sets offset correction for the second axis. The maximum correction is
    /// -2048 .. +2032.
    ///
    /// @param offset 8-bit signed value to write to offset correction.
    ///
    void set_offset_2(int8_t offset)
    {
        register_write(MAG_OFFSET_CONFIG_2, offset);
    }

    /// @return Device ID (die version / sensitivity range). Mask off the
    ///         reserved bits of the Device ID register. Deprecated, recommend
    ///         is_present() API instead.
    DeviceID get_device_id()
    {
        return static_cast<DeviceID>(
            register_read(DEVICE_ID) & DEVICE_ID_VERSION_MASK);
    }

    /// Read the conversion results.
    /// @param xyz results as a three item array in order of X, Y, and Z
    void read_conversion_results(int16_t xyz[3])
    {
        register_read(X_MSB_RESULT, (uint8_t*)xyz, 6);
        xyz[0] = be16toh(xyz[0]);
        xyz[1] = be16toh(xyz[1]);
        xyz[2] = be16toh(xyz[2]);
    }

    /// Read the angle result.
    /// @return angle in degrees (0 - 360) * 16
    int16_t read_angle_result()
    {
        int16_t angle;
        register_read(ANGLE_RESULT_MSB, (uint8_t*)&angle, sizeof(int16_t));
        return be16toh(angle);
    }

private:
    /// Useful bit masks.
    enum BitMasks
    {
        /// Mask for averaging field in DEVICE_CONFIG_1 register.
        DCONF1_AVG_MASK = 0b11100,

        /// Mask for the operating mode in DEVICE_CONFIG_2 register.
        DCONF2_OPERATING_MODE_MASK = 0b11,

        SCONF1_MAG_CH_MASK     = 0b11110000, ///< Channel enable mask.
        SCONF1_SLEEP_TIME_MASK = 0b00001111, ///< sleep time mask.

        /// Mask for MAG_GAIN_CH bits in the SENSOR_CONFIG_2 register.
        SCONF2_MAG_GAIN_CH_MASK = 1 << 4,
        SCONF2_MAG_GAIN_CH_ADJ1 = 0,
        SCONF2_MAG_GAIN_CH_ADJ2 = SCONF2_MAG_GAIN_CH_MASK,

        /// Mask for ANGLE_EN bits in the SENSOR_CONFIG_2 register.
        SCONF2_ANGLE_EN_MASK = 0b1100,

        DEVICE_ID_RESERVED_MASK = 0b11111100, ///< reserved bits.
        DEVICE_ID_VERSION_MASK  = 0b00000011, ///< version bits.
    };

    /// Device register address offsets.
    enum Registers
    {
        DEVICE_CONFIG_1 = 0x0,     // Configure Device Operation Modes
        DEVICE_CONFIG_2 = 0x1,     // Configure Device Operation Modes
        SENSOR_CONFIG_1 = 0x2,     // Sensor Device Operation Modes
        SENSOR_CONFIG_2 = 0x3,     // Sensor Device Operation Modes
        X_THR_CONFIG = 0x4,        // X Threshold Configuration
        Y_THR_CONFIG = 0x5,        // Y Threshold Configuration
        Z_THR_CONFIG = 0x6,        // Z Threshold Configuration
        T_CONFIG = 0x7,            // Temp Sensor Configuration
        INT_CONFIG_1 = 0x8,        // Configure Device Operation Modes
        MAG_GAIN_CONFIG = 0x9,     // Configure Device Operation Modes
        MAG_OFFSET_CONFIG_1 = 0xA, // Configure Device Operation Modes
        MAG_OFFSET_CONFIG_2 = 0xB, // Configure Device Operation Modes
        I2C_ADDRESS = 0xC,         // I2C Address Register
        DEVICE_ID = 0xD,           // ID for the device die
        MANUFACTURER_ID_LSB = 0xE, // Manufacturer ID lower byte
        MANUFACTURER_ID_MSB = 0xF, // Manufacturer ID upper byte
        T_MSB_RESULT = 0x10,       // Conversion Result Register
        T_LSB_RESULT = 0x11,       // Conversion Result Register
        X_MSB_RESULT = 0x12,       // Conversion Result Register
        X_LSB_RESULT = 0x13,       // Conversion Result Register
        Y_MSB_RESULT = 0x14,       // Conversion Result Register
        Y_LSB_RESULT = 0x15,       // Conversion Result Register
        Z_MSB_RESULT = 0x16,       // Conversion Result Register
        Z_LSB_RESULT = 0x17,       // Conversion Result Register
        CONV_STATUS = 0x18,        // Conversion Status Register
        ANGLE_RESULT_MSB = 0x19,   // Conversion Result Register
        ANGLE_RESULT_LSB = 0x1A,   // Conversion Result Register
        MAGNITUDE_RESULT = 0x1B,   // Conversion Result Register
        DEVICE_STATUS = 0x1C,      // Device_Diag Status Register
    };

    /// Reads one or more (sequential) registers.
    /// @param reg starting register offset.
    /// @param data where to write payload
    /// @param len number of registers to read (1 byte each).
    /// Returns when the read is complete.
    /// @return -1 on error (see errno), 0 on success
    int register_read(uint8_t reg, uint8_t *data, uint16_t len)
    {
        struct i2c_msg msgs[] = {
            {.addr = i2cAddress_, .flags = 0, .len = 1, .buf = &reg},
            {.addr = i2cAddress_, .flags = I2C_M_RD, .len = len, .buf = data}};

        struct i2c_rdwr_ioctl_data ioctl_data = {
            .msgs = msgs, .nmsgs = ARRAYSIZE(msgs)};

        return ::ioctl(fd_, I2C_RDWR, &ioctl_data);
    }

    /// Writes one or more (sequential) register.
    /// @param reg starting register offset.
    /// @param data payload to write
    /// @param len number of bytes to write.
    /// Returns when the write is complete.
    void register_write(uint8_t reg, const uint8_t *data, uint16_t len)
    {
        HASSERT(len <= 2);
        uint8_t dat[3];
        dat[0] = reg;
        dat[1] = data[0];
        if (len > 1)
        {
            dat[2] = data[1];
        }
        struct i2c_msg msgs[] = {{.addr = i2cAddress_,
            .flags = 0,
            .len = (uint16_t)(len + 1),
            .buf = dat}};

        struct i2c_rdwr_ioctl_data ioctl_data = {
            .msgs = msgs, .nmsgs = ARRAYSIZE(msgs)};

        ::ioctl(fd_, I2C_RDWR, &ioctl_data);
    }

    /// Writes one register.
    /// @param reg starting register offset.
    /// @param value payload to write
    /// Returns when the write is complete.
    void register_write(uint8_t reg, uint8_t value)
    {
        register_write(reg, &value, 1);
    }

    /// Reads a single register.
    ///
    /// @param reg register number.
    ///
    /// @return the register's current value.
    ///
    uint8_t register_read(uint8_t reg)
    {
        uint8_t ret;
        register_read(reg, &ret, 1);
        return ret;
    }

    /// Modifies a register value in place.
    ///
    /// @param reg register address to modify.
    /// @param mask bitmask of which bits to modify
    /// @param value new value of these bits (shifted to the right position).
    /// Any bits set outside the mask will be ignored.
    ///
    void register_modify(uint8_t reg, uint8_t mask, uint8_t value)
    {
        uint8_t current = register_read(reg);
        current &= ~mask;
        current |= (value & mask);
        register_write(reg, current);
    }

    /// I2C device.
    int fd_ = -1;
    /// 7-bit address, right aligned.
    const uint8_t i2cAddress_;
};

/// '|' operator for DeviceStatus.
/// @param a left hand operand
/// @param b right hand operand
inline TMAG5273::DeviceStatus operator|(
    const TMAG5273::DeviceStatus &a, const TMAG5273::DeviceStatus &b)
{
    return static_cast<TMAG5273::DeviceStatus>(
        static_cast<uint8_t>(a) | static_cast<uint8_t>(b));
}

/// '|' operator for InterruptConfig.
/// @param a left hand operand
/// @param b right hand operand
inline TMAG5273::InterruptConfig operator|(
    const TMAG5273::InterruptConfig &a, const TMAG5273::InterruptConfig &b)
{
    return static_cast<TMAG5273::InterruptConfig>(
        static_cast<uint8_t>(a) | static_cast<uint8_t>(b));
}

#endif // _FREERTOS_DRIVERS_COMMON_TMAG5273_HXX_

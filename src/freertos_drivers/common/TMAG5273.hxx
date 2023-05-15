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

#include "i2c-dev.h"
#include "i2c.h"

class TMAG5273 : private Atomic
{
public:
    /// Supported I2C addresses.
    enum I2CAddress : uint8_t
    {
        ADDR_A = 0x35, /// A1 and A2 device address
        ADDR_B = 0x22, /// B1 and B2 device address
        ADDR_C = 0x78, /// C1 and C2 device address
        ADDR_D = 0x44, /// D1 and D2 device address
    };

    /// Constructor
    /// @param address is the 7-bit address (right aligned), typically 0x35,
    /// 0x22, 0x78 or 0x44). Default is the A1/A2 device.
    TMAG5273(uint8_t address = 0x35)
        : i2cAddress_(address)
    { }

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

    enum BitMasks
    {
        /// Mask for averaging field in DEVICE_CONFIG_1 register.
        DCONF1_AVG_MASK = 0b11100,
        /// Average 1 saples.
        DCONF1_AVG_1 = (0 << 2),
        /// Average 2 saples.
        DCONF1_AVG_2 = (1 << 2),
        /// Average 4 saples.
        DCONF1_AVG_4 = (2 << 2),
        /// Average 8 saples.
        DCONF1_AVG_8 = (3 << 2),
        /// Average 16 saples.
        DCONF1_AVG_16 = (4 << 2),
        /// Average 32 saples.
        DCONF1_AVG_32 = (5 << 2),

        /// Mask for MAG_GAIN_CH bits in the SENSOR_CONFIG_2 register.
        SCONF2_MAG_GAIN_CH_MASK = 1 << 4,
        SCONF2_MAG_GAIN_CH_ADJ1 = 0,
        SCONF2_MAG_GAIN_CH_ADJ2 = SCONF2_MAG_GAIN_CH_MASK,

        /// Mask for ANGLE_EN bits in the SENSOR_CONFIG_2 register.
        SCONF2_ANGLE_EN_MASK = 0b1100,
        SCONF2_ANGLE_EN_OFF = 0 << 2,
        SCONF2_ANGLE_EN_XY = 1 << 2,
        SCONF2_ANGLE_EN_YZ = 2 << 2,
        SCONF2_ANGLE_EN_XZ = 3 << 2,
    };

    /// Checks whether the magnetic sensor is present.
    /// @return true if it is there and responding.
    bool is_present()
    {
        uint8_t rc[3];
        // We start with a dummy read to ensure I2C is in a known state.
        register_read(TMAG5273::DEVICE_ID, rc, 3);
        if (register_read(TMAG5273::DEVICE_ID, rc, 3) < 0)
        {
            // Error reading from i2c.
            LOG(VERBOSE, "error read");
            return false;
        }
        if (rc[1] != 0x49 || rc[2] != 0x54)
        {
            LOG(VERBOSE, "read %x %x %x", rc[0], rc[1], rc[2]);
            return false;
        }
        return true;
    }

    /// Sets the oversampling+averaging mode.
    ///
    /// @param val DCONF1_AVG_* to determine what the oversampling should be.
    ///
    void set_oversampling(BitMasks val)
    {
        register_modify(DEVICE_CONFIG_1, DCONF1_AVG_MASK, val);
    }

    /// Sets whether angle measurement should be enabled.
    ///
    /// @param val SCONF2_ANGLE_EN_* to determine which axis to enable angle
    /// measurement on.
    ///
    void set_angle_en(BitMasks val)
    {
        register_modify(SENSOR_CONFIG_2, SCONF2_ANGLE_EN_MASK, val);
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

    /// @return device ID (die version / sensitivity range).
    uint8_t get_device_id()
    {
        return register_read(DEVICE_ID);
    }

private:
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
        HASSERT(len <= 2);
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

#endif // _FREERTOS_DRIVERS_COMMON_TMAG5273_HXX_

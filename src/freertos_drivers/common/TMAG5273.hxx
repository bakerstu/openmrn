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
    /// Constructor
    /// @param address is the 7-bit address (right aligned), typically 0x35,
    /// 0x22, 0x78 or 0x44). Default is the A1/A2 device.
    TMAG5273(uint8_t address = 0x35)
        : address_(address << 1)
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
    };

    
    /// @return the I2C address for reads.
    uint8_t read_address()
    {
        return address_ | 1;
    }
    /// @return the I2C address for writes.
    uint8_t write_address()
    {
        return address_;
    }

    /// I2C device.
    int fd_ = -1;
    /// 7-bit address, left aligned.
    const uint8_t address_;
};

#endif // _FREERTOS_DRIVERS_COMMON_TMAG5273_HXX_


/** \copyright
 * Copyright (c) 2023, Mike Dunston
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
 * \file Esp32HardwareI2C.hxx
 *
 * I2C driver implementation for OpenMRN. This leverages the ESP-IDF I2C driver
 * implementation and provides a VFS interface.
 *
 * @author Mike Dunston
 * @date 8 Feb 2023
 */

#ifndef _FREERTOS_DRIVERS_ESP32_ESP32HARDWAREI2C_HXX_
#define _FREERTOS_DRIVERS_ESP32_ESP32HARDWAREI2C_HXX_

#include "i2c.h"
#include "i2c-dev.h"
#include <unistd.h>
#include <driver/i2c.h>
#include <hal/gpio_types.h>
#include <utils/Atomic.hxx>
#include <vector>

namespace openmrn_arduino
{

/// The ESP32 has one or two built-in I2C controller interfaces, this code
/// provides a VFS interface that can be used by various I2C drivers to access
/// the underlying I2C bus.
///
/// Only one instance of this class should be created in the application so the
/// memory overhead is minimized and performance of the VFS minimally impacted.
///
/// Example usage (scanner):
/// ```
/// Esp32HardwareI2C i2c;
///
/// void setup()
/// {
///   i2c.hw_init(GPIO_NUM_4, GPIO_NUM_5, 100000);
///   i2c.scan();
/// }
/// ```
///
/// Example usage (MCP23017):
/// ```
/// Esp32HardwareI2C i2c;
/// Executor<1> io_executor("io_thread", 1, 1300);
/// MCP23017 expander(&io_executor, 0, 0, 0);
///
/// void setup()
/// {
///   i2c.hw_init(GPIO_NUM_4, GPIO_NUM_5, 100000);
///   expander.init("/dev/i2c/0");
///   ...
/// }
/// ```
class Esp32HardwareI2C : private Atomic
{
public:
    /// Constructor.
    ///
    /// @param path Base path to use for I2C drivers.
    Esp32HardwareI2C(const char * const path = "/dev/i2c");

    /// Destructor.
    ~Esp32HardwareI2C();

    /// Initializes the underlying I2C controller hardware and VFS interface.
    ///
    /// @param sda GPIO pin to use for I2C data transfer.
    /// @param scl GPIO pin to use for I2C clock.
    /// @param bus_speed I2C clock frequency.
    /// @param port I2C controller to initialize.
    ///
    /// For hardware which supports more than one I2C controller, this method
    /// must be called once per controller being used. 
    ///
    /// NOTE: This must be called prior to usage of any other methods for each
    /// I2C controller being used.
    void hw_init(const gpio_num_t sda, const gpio_num_t scl,
                 const uint32_t bus_speed, const i2c_port_t port = I2C_NUM_0);

    /// Utility method to perform a scan for all devices on the I2C bus.
    ///
    /// @param port Hardware I2C controller port number to use for scanning.
    ///
    /// NOTE: The hw_init method must have been run prior to calling this
    /// method. Failure to do so will result in an error being raised.
    void scan(const i2c_port_t port);

    /// VFS implementation of write(fd, buf, size)
    ///
    /// @param fd is the file descriptor being written to.
    /// @param buf is the buffer containing the data to be written.
    /// @param size is the size of the buffer.
    ///
    /// @return number of bytes written or -1 if there is the write would be a
    /// blocking operation.
    ///
    /// NOTE: The provided fd is used internally to determine which I2C
    /// controller should be used and the address to write the data to. If the
    /// address has not been set 
    ssize_t write(int fd, const void *buf, size_t size);

    /// VFS implementation of read(fd, buf, size)
    ///
    /// @param fd is the file descriptor being read from.
    /// @param buf is the buffer to write into.
    /// @param size is the size of the buffer.
    /// @return number of bytes read or -1 if there is the read would be a
    /// blocking operation.
    ssize_t read(int fd, void *buf, size_t size);

    /// VFS implementation of open(path, flags, mode).
    ///
    /// @param path is the path to the file being opened.
    /// @param flags are the flags to use for opened file.
    /// @param mode is the mode to use for the opened file.
    ///
    /// When this method is invoked it will enable the TWAI driver and start the
    /// periodic timer used for RX/TX of frame data.
    ///
    /// @return 0 upon success, -1 upon failure with errno containing the cause.
    int open(const char *path, int flags, int mode);

    /// VFS implementation of close(fd).
    ///
    /// @param fd is the file descriptor to close.
    ///
    /// When this method is invoked it will disable the TWAI driver and stop the
    /// periodic timer used for RX/TX of frame data if it is running.
    ///
    /// @return zero upon success, negative value with errno for failure.
    int close(int fd);

    /// VFS implementation of ioctl.
    ///
    /// @param fd is the file descriptor to operate on.
    /// @param cmd is the command to execute.
    /// @param args is the args for the command.
    ///
    /// @return zero upon success, negative value with errno for failure.
    int ioctl(int fd, int cmd, va_list args);

private:
    /// VFS Mount point.
    const char * const path_;

    /// Timeout to use for I2C operations, default is one second.
    static constexpr TickType_t I2C_OP_TIMEOUT = pdMS_TO_TICKS(1000);

    /// Timeout to use for I2C device scanning, default is 50 milliseconds.
    static constexpr TickType_t I2C_SCAN_TIMEOUT = pdMS_TO_TICKS(50);

    /// ISR flags to use for I2C, this defaults to allowing usage of a shared
    /// interupt.
    static constexpr int I2C_ISR_FLAGS = ESP_INTR_FLAG_SHARED;

    /// I2C "Slave" RX buffer size, we do not use/support this feature so it is
    /// set to zero.
    static constexpr size_t I2C_SLAVE_RX_BUF_SIZE = 0;

    /// I2C "Slave" TX buffer size, we do not use/support this feature so it is
    /// set to zero.
    static constexpr size_t I2C_SLAVE_TX_BUF_SIZE = 0;

    /// Tracking structure used to map file handles to an I2C controller and
    /// address.
    struct i2c_device_t
    {
        /// I2C Controller that this file handle will use.
        i2c_port_t port;

        /// I2C address to use when greater than zero.
        int address;

        /// Assigned file handle for this entry.
        int fd;
    };

    /// Collection of I2C devices that have been opened.
    std::vector<i2c_device_t> devices_;

    /// Internal tracking for initialization of the underlying I2C hardware.
    bool i2cInitialized_[SOC_I2C_NUM];

    /// Internal tracking for the VFS adapter layer.
    bool vfsInitialized_{false};

    /// Transfers multiple payloads to I2C devices.
    ///
    /// @param port Underlying @ref i2c_port_t to use for I2C transfers.
    /// @param msgs payloads to be transfered, this includes the target device
    /// address, data to transfer and length of data.
    /// @param num Number of transfers to perform.
    ///
    /// @return Number of bytes transfered or an error code (negative value).
    int transfer_messages(const i2c_port_t port, struct i2c_msg *msgs, int num);
};

}; // namespace openmrn_arduino

using openmrn_arduino::Esp32HardwareI2C;

#endif // _FREERTOS_DRIVERS_ESP32_ESP32HARDWAREI2C_HXX_
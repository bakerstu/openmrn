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
 * \file Esp32HardwareI2C.cxx
 *
 * I2C driver implementation for OpenMRN. This leverages the ESP-IDF I2C driver
 * implementation and provides a VFS interface.
 *
 * @author Mike Dunston
 * @date 8 Feb 2023
 */

#if defined(ESP_PLATFORM)

#include "Esp32HardwareI2C.hxx"
// stropts.h must be included before i2c.h
#include "stropts.h"
#include "i2c.h"
#include "i2c-dev.h"
#include "sdkconfig.h"
#include "utils/format_utils.hxx"
#include "utils/logging.h"
#include "utils/StringPrintf.hxx"

#include <algorithm>

#if CONFIG_VFS_SUPPORT_TERMIOS
// remove defines added by arduino-esp32 core/esp32/binary.h which are
// duplicated in sys/termios.h which may be included by esp_vfs.h
#undef B110
#undef B1000000
#endif // CONFIG_VFS_SUPPORT_TERMIOS
#include <esp_vfs.h>
#include <string>
#include <sys/errno.h>

namespace openmrn_arduino
{

extern "C"
{

/// VFS adapter for write(fd, buf, size)
///
/// @param ctx is the @ref Esp32HardwareI2C instance to invoke.
/// @param fd is the file descriptor being written to.
/// @param buf is the buffer containing the data to be written.
/// @param size is the size of the buffer.
/// @return number of bytes written or -1 if there is the write would be a
/// blocking operation.
static ssize_t i2c_vfs_write(void *ctx, int fd, const void *buf, size_t size)
{
    HASSERT(ctx != NULL);
    Esp32HardwareI2C *i2c = reinterpret_cast<Esp32HardwareI2C *>(ctx);
    return i2c->write(fd, buf, size);
}

/// VFS adapter for read(fd, buf, size)
///
/// @param ctx is the @ref Esp32HardwareI2C instance to invoke.
/// @param fd is the file descriptor being read from.
/// @param buf is the buffer to write into.
/// @param size is the size of the buffer.
/// @return number of bytes read or -1 if there is the read would be a
/// blocking operation.
static ssize_t i2c_vfs_read(void *ctx, int fd, void *buf, size_t size)
{
    HASSERT(ctx != NULL);
    Esp32HardwareI2C *i2c = reinterpret_cast<Esp32HardwareI2C *>(ctx);
    return i2c->read(fd, buf, size);
}

/// VFS adapter for open(path, flags, mode).
///
/// @param ctx is the @ref Esp32HardwareI2C instance to invoke.
/// @param path is the path to the file being opened.
/// @param flags are the flags to use for opened file.
/// @param mode is the mode to use for the opened file.
///
/// When this method is invoked it will enable the TWAI driver and start the
/// periodic timer used for RX/TX of frame data.
///
/// @return 0 upon success, -1 upon failure with errno containing the cause.
static int i2c_vfs_open(void *ctx, const char *path, int flags, int mode)
{
    HASSERT(ctx != NULL);
    Esp32HardwareI2C *i2c = reinterpret_cast<Esp32HardwareI2C *>(ctx);
    return i2c->open(path, flags, mode);
}

/// VFS adapter for close(fd).
///
/// @param ctx is the @ref Esp32HardwareI2C instance to invoke.
/// @param fd is the file descriptor to close.
///
/// When this method is invoked it will disable the TWAI driver and stop the
/// periodic timer used for RX/TX of frame data if it is running.
///
/// @return zero upon success, negative value with errno for failure.
static int i2c_vfs_close(void *ctx, int fd)
{
    HASSERT(ctx != NULL);
    Esp32HardwareI2C *i2c = reinterpret_cast<Esp32HardwareI2C *>(ctx);
    return i2c->close(fd);
}

/// VFS adapter for ioctl.
///
/// @param ctx is the @ref Esp32HardwareI2C instance to invoke.
/// @param fd is the file descriptor to operate on.
/// @param cmd is the command to execute.
/// @param args is the args for the command.
///
/// @return zero upon success, negative value with errno for failure.
static int i2c_vfs_ioctl(void *ctx, int fd, int cmd, va_list args)
{
    HASSERT(ctx != NULL);
    Esp32HardwareI2C *i2c = reinterpret_cast<Esp32HardwareI2C *>(ctx);
    return i2c->ioctl(fd, cmd, args);
}

/// VFS adapter for fcntl(fd, cmd, arg).
///
/// @param ctx is the @ref Esp32HardwareI2C instance to invoke.
/// @param fd to operate on.
/// @param cmd to be executed.
/// @param arg arg to be used for the operation.
///
/// This method is currently a NO-OP.
///
/// @return zero upon success, negative value with errno for failure.
static int i2c_vfs_fcntl(void *ctx, int fd, int cmd, int arg)
{
    HASSERT(ctx != NULL);
    return 0;
}

} // extern "C"

Esp32HardwareI2C::Esp32HardwareI2C(const char * const path)
    : path_(path)
{
}

Esp32HardwareI2C::~Esp32HardwareI2C()
{
    for (size_t idx = 0; idx < SOC_I2C_NUM; idx++)
    {
        if (i2cInitialized_[idx])
        {
            ESP_ERROR_CHECK(i2c_driver_delete(static_cast<i2c_port_t>(idx)));
        }
    }
    if (vfsInitialized_)
    {
        ESP_ERROR_CHECK(esp_vfs_unregister(path_));
    }
}

void Esp32HardwareI2C::hw_init(const gpio_num_t sda, const gpio_num_t scl,
                               const uint32_t bus_speed, const i2c_port_t port)
{
    if (!i2cInitialized_[port])
    {
        i2c_config_t i2c_config = {};
        i2c_config.mode = I2C_MODE_MASTER;
        i2c_config.sda_io_num = sda;
        i2c_config.sda_pullup_en = GPIO_PULLUP_ENABLE;
        i2c_config.scl_io_num = scl;
        i2c_config.scl_pullup_en = GPIO_PULLUP_ENABLE;
        i2c_config.master.clk_speed = bus_speed;

        LOG(INFO,
            "[I2C] Initializing I2C%d using SDA:%d, SCL:%d, "
            "bus-speed: %" PRIu32, port, sda, scl, bus_speed);
        ESP_ERROR_CHECK(i2c_param_config(port, &i2c_config));
        ESP_ERROR_CHECK(i2c_driver_install(port, I2C_MODE_MASTER,
            I2C_SLAVE_RX_BUF_SIZE, I2C_SLAVE_TX_BUF_SIZE, I2C_ISR_FLAGS));

        i2cInitialized_[port] = true;
    }
    else
    {
        LOG_ERROR("[I2C] I2C%d has already been initialized!", port);
    }

    if (!vfsInitialized_)
    {
        esp_vfs_t vfs = {};
        vfs.write_p = i2c_vfs_write;
        vfs.read_p = i2c_vfs_read;
        vfs.open_p = i2c_vfs_open;
        vfs.close_p = i2c_vfs_close;
        vfs.fcntl_p = i2c_vfs_fcntl;
        vfs.ioctl_p = i2c_vfs_ioctl;
        vfs.flags = ESP_VFS_FLAG_CONTEXT_PTR;
        ESP_ERROR_CHECK(esp_vfs_register(path_, &vfs, this));

        vfsInitialized_ = true;
    }
}

void Esp32HardwareI2C::scan(const i2c_port_t port)
{
    // Scan the I2C bus and dump the output of devices that respond
    std::string scanresults =
        "     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n"
        "00: ";
    scanresults.reserve(256);

    HASSERT(i2cInitialized_[port]);
    for (uint8_t addr = 0; addr < 0x7F; addr++)
    {
        if (addr % 16 == 0)
        {
            scanresults.append(StringPrintf("\n%02x: ", addr));
        }
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(port, cmd, I2C_SCAN_TIMEOUT);
        i2c_cmd_link_delete(cmd);
        if (ret == ESP_OK)
        {
            scanresults.append(StringPrintf("%02x ", addr));
        }
        else if (ret == ESP_ERR_TIMEOUT)
        {
            scanresults.append("?? ");
        }
        else
        {
            scanresults.append("-- ");
        }
    }
    LOG(INFO, scanresults.c_str());
}

ssize_t Esp32HardwareI2C::write(int fd, const void *buf, size_t size)
{
    uint8_t address;
    i2c_port_t port;

    {
        AtomicHolder h(this);

        auto entry = std::find_if(devices_.begin(), devices_.end(),
            [fd](const auto &device)
            {
                return device.fd == fd;
            });

        if (entry == devices_.end())
        {
            // file handle not found, return an error.
            errno = EBADF;
            return -EBADF;
        }
        else if (entry->fd < 0)
        {
            // no address has been defined for this file handle, return an error.
            errno = EINVAL;
            return -EINVAL;
        }
        address = entry->address;
        port = entry->port;
    }

    esp_err_t res = ESP_ERROR_CHECK_WITHOUT_ABORT(
        i2c_master_write_to_device(port, address, (uint8_t *)buf, size,
            I2C_OP_TIMEOUT));

    if (res == ESP_ERR_TIMEOUT)
    {
        return -ETIMEDOUT;
    }
    else if (res == ESP_ERR_INVALID_STATE)
    {
        return -EINVAL;
    }
    else if (res != ESP_OK)
    {
        return -EIO;
    }
    return size;
}

ssize_t Esp32HardwareI2C::read(int fd, void *buf, size_t size)
{
    uint8_t address;
    i2c_port_t port;

    {
        AtomicHolder h(this);

        auto entry = std::find_if(devices_.begin(), devices_.end(),
            [fd](const auto &device)
            {
                return device.fd == fd;
            });

        if (entry == devices_.end())
        {
            // file handle not found, return an error.
            errno = EBADF;
            return -EBADF;
        }
        else if (entry->fd < 0)
        {
            // no address has been defined for this file handle, return an error.
            errno = EINVAL;
            return -EINVAL;
        }
        address = entry->address;
        port = entry->port;
    }


    esp_err_t res = ESP_ERROR_CHECK_WITHOUT_ABORT(
        i2c_master_read_from_device(port, address, (uint8_t *)buf, size,
            I2C_OP_TIMEOUT));

    if (res == ESP_ERR_TIMEOUT)
    {
        return -ETIMEDOUT;
    }
    else if (res == ESP_ERR_INVALID_STATE)
    {
        return -EINVAL;
    }
    else if (res != ESP_OK)
    {
        return -EIO;
    }
    return size;
}

int Esp32HardwareI2C::open(const char *path, int flags, int mode)
{
    std::string path_str = path;
    i2c_device_t new_dev =
    {
        .port = I2C_NUM_0,
        .address = -1,
        .fd = 0,
    };

    if (path_str.back() == '0')
    {
        new_dev.port = I2C_NUM_0;
    }
#if SOC_I2C_NUM > 1
    else if (path_str.back() == '1')
    {
        new_dev.port = I2C_NUM_1;
    }
#endif // SOC_I2C_NUM > 1
    else
    {
        LOG_ERROR("[I2C] Unsupported I2C path: %s", path);
        return -ENOENT;
    }
    
    if (!i2cInitialized_[new_dev.port])
    {
        LOG_ERROR("[I2C] Uninitialized I2C path: %s", path);
        return -ENODEV;
    }

    // scan existing devices to find a unique file handle number to return to
    // the caller. The file handle starts with zero and is set to the maximum
    // file handle found plus one. When a file handle is reclaimed there will
    // be gaps in the file handles which will not be reclaimed until entries
    // with higher file handle numbers are also reclaimed.
    {
        AtomicHolder h(this);
    
        if (!devices_.empty())
        {
            for (auto &entry: devices_)
            {
                if (entry.fd >= new_dev.fd)
                {
                    new_dev.fd = entry.fd + 1;
                }
            }
        }
        devices_.push_back(new_dev);
    }

    LOG(INFO, "[I2C] Using fd: %d (I2C%d) for %s", new_dev.fd, new_dev.port,
        path);

    return new_dev.fd;
}

int Esp32HardwareI2C::close(int fd)
{
    AtomicHolder h(this);

    auto entry = std::find_if(devices_.begin(), devices_.end(),
    [fd](const auto &device)
    {
        return device.fd == fd;
    });

    // only delete the device if has been found.
    if (entry != devices_.end())
    {
        devices_.erase(entry);
    }
    return 0;
}

int Esp32HardwareI2C::ioctl(int fd, int cmd, va_list args)
{
    AtomicHolder h(this);
    HASSERT(IOC_TYPE(cmd) == I2C_MAGIC);

    auto entry = std::find_if(devices_.begin(), devices_.end(),
        [fd](const auto &device)
        {
            return device.fd == fd;
        });
    if (entry == devices_.end())
    {
        errno = EBADF;
        return -EBADF;
    }

    switch(static_cast<unsigned int>(cmd))
    {
        default:
            return -EINVAL;
        case I2C_SLAVE:
            entry->address = static_cast<int>(va_arg(args, int));
            return 0;
        case I2C_RDWR:
            struct i2c_rdwr_ioctl_data *data =
                reinterpret_cast<struct i2c_rdwr_ioctl_data *>(va_arg(args, uintptr_t));
            return transfer_messages(entry->port, data->msgs, data->nmsgs);
    }

    return 0;
}

int Esp32HardwareI2C::transfer_messages(
    const i2c_port_t port, struct i2c_msg *msgs, int num)
{
    int total_len = 0;
    esp_err_t res = ESP_OK;

    for (int idx = 0; idx < num; idx++)
    {
        struct i2c_msg *msg = msgs + idx;
        if (msg->flags & I2C_M_RD)
        {
            res =
                ESP_ERROR_CHECK_WITHOUT_ABORT(
                    i2c_master_read_from_device(port, msg->addr,
                        (uint8_t *)msg->buf, msg->len, I2C_OP_TIMEOUT));
        }
        else
        {
            res =
                ESP_ERROR_CHECK_WITHOUT_ABORT(
                    i2c_master_write_to_device(port, msg->addr,
                        (uint8_t *)msg->buf, msg->len, I2C_OP_TIMEOUT));
        }
        if (res == ESP_ERR_TIMEOUT)
        {
            return -ETIMEDOUT;
        }
        else if (res == ESP_ERR_INVALID_STATE)
        {
            return -EINVAL;
        }
        else if (res != ESP_OK)
        {
            return -EIO;
        }
        total_len += msg->len;
    }
    return total_len;
}

} // namespace openmrn_arduino

#endif // ESP_PLATFORM
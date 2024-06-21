/** \copyright
 * Copyright (c) 2015, Stuart W Baker
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
 * \file i2c-dev.h
 * This file implements the OS independent I2C defines.
 *
 * @author Stuart W. Baker
 * @date 5 January 2015
 */

#ifndef _I2C_DEV_HXX_
#define _I2C_DEV_HXX_

#if defined (__linux__)
    #include <linux/i2c-dev.h>
#elif defined (__FreeRTOS__) || defined(ESP_PLATFORM)
    #include <stdint.h>
    /** magic number for this driver's ioctl calls */
    #define I2C_MAGIC ('i')

    /** Slave address to use.  Slave address is 7 or 10 bits, but 10-bit
     * addresses are NOT supported!
     */
    #define I2C_SLAVE IOW(I2C_MAGIC, 1, sizeof(long))

    /** Combined R/W transfer, one stop only. */
    #define I2C_RDWR  IOWR(I2C_MAGIC, 2, sizeof(void*))

    /** Combined R/W transfer, one stop only. */
    #define I2C_SMBUS IOWR(I2C_MAGIC, 3, sizeof(void*))

    /** This is the structure as used in the I2C_SMBUS ioctl call */
    struct i2c_smbus_ioctl_data
    {
        uint8_t read_write;
        uint8_t command;
        uint32_t size;
        union i2c_smbus_data *data;
    };

    /** This is the structure as used in the I2C_RDWR ioctl call */
    struct i2c_rdwr_ioctl_data
    {
        struct i2c_msg *msgs; /**< pointers to i2c_msgs */
        uint32_t nmsgs; /**< number of i2c_msgs */
    };

    /** maximum number of message segments in @ref i2c_rdwr_ioctl_data struct */
    #define  I2C_RDRW_IOCTL_MAX_MSGS 42
#else
    #error I2C drivers not supported on this OS
#endif

#endif /* _I2C_DEV_HXX_ */

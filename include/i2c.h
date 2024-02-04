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
 * \file i2c.h
 * This file implements the OS independent I2C defines.
 *
 * @author Stuart W. Baker
 * @date 5 January 2015
 */

#ifndef _I2C_HXX_
#define _I2C_HXX_

#if defined (__linux__)
    #include <linux/i2c.h>
#elif defined (__FreeRTOS__) || defined(ESP_PLATFORM)
    #include <stdint.h>
    /** Used in @ref i2c_rdwr_ioctl_data to describe a transaction segment. */
    struct i2c_msg
    {
        uint16_t addr; /**< slave address */
        uint16_t flags; /**< control flags */
            #define I2C_M_RD 0x0001 /**< read data, from slave to master */
        uint16_t len; /**< msg length */
        uint8_t *buf; /**< pointer to msg data */
    };

    #define I2C_SMBUS_BLOCK_MAX 32  /* As specified in SMBus standard */

    /** Data for SMBus Messages */
    union i2c_smbus_data {
        uint8_t byte;
        uint16_t word;
        uint8_t block[I2C_SMBUS_BLOCK_MAX + 2]; /* block[0] is used for length */
                       /* and one more for user-space compatibility */
    };

/* i2c_smbus_xfer read or write markers */
#define I2C_SMBUS_READ  1
#define I2C_SMBUS_WRITE 0

/* SMBus transaction types (size parameter in the above functions)
   Note: these no longer correspond to the (arbitrary) PIIX4 internal codes! */
#define I2C_SMBUS_QUICK         0
#define I2C_SMBUS_BYTE          1
#define I2C_SMBUS_BYTE_DATA     2
#define I2C_SMBUS_WORD_DATA     3
#define I2C_SMBUS_PROC_CALL     4
#define I2C_SMBUS_BLOCK_DATA        5
#define I2C_SMBUS_I2C_BLOCK_BROKEN  6
#define I2C_SMBUS_BLOCK_PROC_CALL   7       /* SMBus 2.0 */
#define I2C_SMBUS_I2C_BLOCK_DATA    8
#else
    #error I2C drivers not supported on this OS
#endif

#endif /* _I2C_HXX_ */

/** \copyright
 * Copyright (c) 2016, Stuart W Baker
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
 * \file spidev.h
 * This file implements the OS independent SPI defines.
 *
 * @author Stuart W. Baker
 * @date 29 May 2016
 */

#ifndef _INCLUDE_SPI_SPIDEV_H_
#define _INCLUDE_SPI_SPIDEV_H_

#if defined (__linux__)
    #include <linux/spi/spidev.h>
#elif defined (__FreeRTOS__)
    #include <stdint.h>

    /** SPI phase mask
     */
    #define SPI_CPHA 0x01

    /** SPI polarity mask
     */
    #define SPI_CPOL 0x02

    /** SPI mode 0 (phase = 0, polarity = 0)
     */
    #define SPI_MODE_0 (0 | 0)

    /** SPI mode 1 (phase = 1, polarity = 0)
     */
    #define SPI_MODE_1 (0 | SPI_CPHA)

    /** SPI mode 2 (phase = 0, polarity = 1)
     */
    #define SPI_MODE_2 (SPI_CPOL | 0)

    /** SPI mode 3 (phase = 1, polarity = 1)
     */
    #define SPI_MODE_3 (SPI_CPOL | SPI_CPHA)

    /** magic number for this driver's ioctl calls */
    #define SPI_IOC_MAGIC ('s')

    /** This is the structure as used in the I2C_SMBUS ioctl call */
    struct spi_ioc_transfer
    {
        /** This will be cast from a pointer to a stream of characters */
        uint64_t tx_buf;

        /** This will be cast from a pointer to a stream of characters */
        uint64_t rx_buf;

        uint32_t len; /**< length of transaction in bytes */
        uint32_t speed_hz; /**< temporary override of the bitrate */

        /** if nonzero, how long to delay after the last bit transfer before
         * optionally deselecting the device before the next transfer
         */
        uint32_t delay_usec;
        uint8_t  bits_per_word; /**< temporary override of the wordsize */

        /** true to deselect device before starting the next transfer */
        uint8_t  cs_change;
        uint32_t pad; /**< padding word for alignment */
    };

    /** Size of the SPI transfer message */
    #define SPI_MSGSIZE(N)  \
        ((((N)*(sizeof (struct spi_ioc_transfer))) < (1 << IOC_SIZEBITS)) \
            ? ((N)*(sizeof (struct spi_ioc_transfer))) : 0)

    /** Perform a SPI message transfer */
    #define SPI_IOC_MESSAGE(N) \
        IOW(SPI_IOC_MAGIC, 0, sizeof(char[SPI_MSGSIZE(N)]))

    /** Read SPI mode: @ref SPI_MODE_0, @ref SPI_MODE_1, @ref SPI_MODE_2, or
     * @ref SPI_MODE_3
     */
    #define SPI_IOC_RD_MODE IOR(SPI_IOC_MAGIC, 1, sizeof(uint8_t))

    /** Write SPI mode: @ref SPI_MODE_0, @ref SPI_MODE_1, @ref SPI_MODE_2, or
     * @ref SPI_MODE_3
     */
    #define SPI_IOC_WR_MODE IOW(SPI_IOC_MAGIC, 1, sizeof(uint8_t))

    /** Read SPI bit justification
     */
    #define SPI_IOC_RD_LSB_FIRST IOR(SPI_IOC_MAGIC, 2, sizeof(uint8_t))

    /** Write SPI bit justification
     */
    #define SPI_IOC_WR_LSB_FIRST IOW(SPI_IOC_MAGIC, 2, sizeof(uint8_t))

    /** Read SPI bit word length in bits
     */
    #define SPI_IOC_RD_BITS_PER_WORD IOR(SPI_IOC_MAGIC, 3, sizeof(uint8_t))

    /** Write SPI word length in bits
     */
    #define SPI_IOC_WR_BITS_PER_WORD IOW(SPI_IOC_MAGIC, 3, sizeof(uint8_t))

    /** Read SPI default max speed in Hz
     */
    #define SPI_IOC_RD_MAX_SPEED_HZ IOR(SPI_IOC_MAGIC, 4, sizeof(uint32_t))

    /** Write SPI default max speed in Hz
     */
    #define SPI_IOC_WR_MAX_SPEED_HZ IOW(SPI_IOC_MAGIC, 4, sizeof(uint32_t))

#else
    #error SPI drivers not supported on this OS
#endif

#endif /* _INCLUDE_SPI_SPIDEV_H_ */

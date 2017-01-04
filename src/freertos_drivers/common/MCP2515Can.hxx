/** @copyright
 * Copyright (c) 2017 Stuart W Baker
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
 * @file MCP2515Can.hxx
 * This file implements the CAN driver for the MCP2515 CAN Controller.
 *
 * @author Stuart W. Baker
 * @date 3 January 2017
 */

#ifndef _FREERTOS_DRIVERS_COMMON_MCP2515CAN_HXX_
#define _FREERTOS_DRIVERS_COMMON_MCP2515CAN_HXX_

#include <unistd.h>

#include "Can.hxx"
#include "SPI.hxx"

#include "os/OS.hxx"

/** Specialization of CAN driver for Tiva CAN.
 */
class MCP2515Can : public Can, public OSThread
{
public:
    /** Constructor.
     * @param name name of this device instance in the file system
     * @param spi_name spi interface that the MCP2515Can is on
     * @param interrupt_enable callback to enable the interrupt
     * @param interrupt_disable callback to disable the interrupt
     */
    MCP2515Can(const char *name, const char *spi_name,
               void (*interrupt_enable)(void),
               void (*interrupt_disable)(void));

    /** Destructor.
     */
    ~MCP2515Can()
    {
    }

    /** Handle an interrupt.  Called by user provided interrupt handler.
     */
    void interrupt_handler();

private:
    /** SPI registers */
    enum Registers
    {
        RXF0SIDH = 0,
        RXF0SIDL,
        RXF0SEID8,
        RXF0EID0,
        RXF1SIDH,
        RXF1SIDL,
        RXF1SEID8,
        RXF1EID0,
        RXF2SIDH,
        RXF2SIDL,
        RXF2SEID8,
        RXF2EID0,
        BFPCTRL,
        TXRTSCTRL,
        CANSTAT,
        CANCTRL,

        RXF3SIDH = 16,
        RXF3SIDL,
        RXF3EID8,
        RXF3EID0,
        RXF4SIDH,
        RXF4SIDL,
        RXF4EID8,
        RXF4EID0,
        RXF5SIDH,
        RXF5SIDL,
        RXF5EID8,
        RXF5EID0,
        TEC,
        RED,

        RXM0SIDH = 32,
        RXM0SIDL,
        RXM0EID8,
        RXM0EID0,
        RXM1SIDH,
        RXM1SIDL,
        RXM1EID8,
        RXM1EID0,
        CNF3,
        CNF2,
        CNF1,
        CANINTE,
        CANINTF,
        EFLG,

        TXB0CTRL = 48,
        TXB0SIDH,
        TXB0SIDL,
        TXB0EID8,
        TXB0EID0,
        TXB0DLC,
        TXB0D0,
        TXB0D1,
        TXB0D2,
        TXB0D3,
        TXB0D4,
        TXB0D5,
        TXB0D6,
        TXB0D7,

        TXB1CTRL = 64,
        TXB1SIDH,
        TXB1SIDL,
        TXB1EID8,
        TXB1EID0,
        TXB1DLC,
        TXB1D0,
        TXB1D1,
        TXB1D2,
        TXB1D3,
        TXB1D4,
        TXB1D5,
        TXB1D6,
        TXB1D7,

        TXB2CTRL = 80,
        TXB2SIDH,
        TXB2SIDL,
        TXB2EID8,
        TXB2EID0,
        TXB2DLC,
        TXB2D0,
        TXB2D1,
        TXB2D2,
        TXB2D3,
        TXB2D4,
        TXB2D5,
        TXB2D6,
        TXB2D7,

        RXB0CTRL = 96,
        RXB0SIDH,
        RXB0SIDL,
        RXB0EID8,
        RXB0EID0,
        RXB0DLC,
        RXB0D0,
        RXB0D1,
        RXB0D2,
        RXB0D3,
        RXB0D4,
        RXB0D5,
        RXB0D6,
        RXB0D7,

        RXB1CTRL = 112,
        RXB1SIDH,
        RXB1SIDL,
        RXB1EID8,
        RXB1EID0,
        RXB1DLC,
        RXB1D0,
        RXB1D1,
        RXB1D2,
        RXB1D3,
        RXB1D4,
        RXB1D5,
        RXB1D6,
        RXB1D7,
    };

    /** interrupt flag masks */
    enum InterruptFlags
    {
        RX0I = 0x01, /**< receive buffer 0 interrupt bit */
        RX1I = 0x02, /**< receive buffer 1 interrupt bit */
        TX0I = 0x04, /**< transmit buffer 0 interrupt bit */
        TX1I = 0x08, /**< transmit buffer 1 interrupt bit */
        TX2I = 0x01, /**< transmit buffer 2 interrupt bit */
        ERRI = 0x02, /**< error interrupt bit */
        WAKI = 0x02, /**< wake-up interrupt bit */
        MERR = 0x02, /**< message error interrupt bit */
    };

    /** SPI transaction instructions */
    enum Instructions
    {
        RESET       = 0xC0, /**< resets internal registers to default state */
        READ        = 0x03, /**< read data from register at selected address */
        READ_RX_BUF = 0x90, /**< read a receive buffer */
        WRITE       = 0x02, /**< write data to register at selected address */
        LOAD_TX_BUF = 0x40, /**< load a transmit buffer */
        STATUS      = 0xA0, /**< read status */
        RX_STATUS   = 0xB0, /**< read rx status */
        RTS         = 0x80, /**< request to send a tramsmit buffer */
        BIT_MODIFY  = 0x05, /**< perform a bit manipulation */
    };

    struct Buffer
    {
        uint8_t sidh; /**< standard identifier high byte */
        uint8_t sidl; /**< standard identifier low byte */
        uint8_t eid8; /**< extended identifier high byte */
        uint8_t eid0; /**< extended identifier low byte */
        uint8_t dlc; /**< data length code */
        uint8_t d0; /**< data 0 byte */
        uint8_t d1; /**< data 1 byte */
        uint8_t d2; /**< data 2 byte */
        uint8_t d3; /**< data 3 byte */
        uint8_t d4; /**< data 4 byte */
        uint8_t d5; /**< data 5 byte */
        uint8_t d6; /**< data 6 byte */
        uint8_t d7; /**< data 7 byte */
    };

    /** User entry point for the created thread.
     * @return exit status
     */
    void *entry() override; /**< entry point to thread */

    void (*interrupt_enable)(void);
    void (*interrupt_disable)(void);

    void enable() override; /**< function to enable device */
    void disable() override; /**< function to disable device */
    void tx_msg() override; /**< function to try and transmit a message */

    /** Function to receive a message.
     * @param index buffer index, 0 or 1
     */
    void rx_msg(int index);

    /** Reset the device.
     */
    void reset()
    {
        uint8_t reset = RESET;
        ::write(spi, &reset, 1);
    }

    /** Read from a SPI register.
     * @param address address to read from
     * @return data read
     */
    uint8_t register_read(Registers address)
    {
        spi_ioc_transfer xfer[2];
        memset(xfer, 0, sizeof(xfer));
        uint8_t wr_data[2] = {READ, address};
        uint8_t rd_data[1];
        xfer[0].tx_buf = (unsigned long)wr_data;
        xfer[0].len = sizeof(wr_data);
        xfer[1].rx_buf = (unsigned long)rd_data;
        xfer[1].len = sizeof(rd_data);
        ::ioctl(spi, SPI_IOC_MESSAGE(2), xfer);

        return rd_data[0];
    }

    /** Read from a RX buffer.
     * @param index buffer index to read from (valid values are 0 and 1)
     * @param buffer pointer to a buffer structure to fill in with the result
     */
    void buffer_read(int index, Buffer *buffer)
    {
        spi_ioc_transfer xfer[2];
        memset(xfer, 0, sizeof(xfer));
        uint8_t wr_data[1];
        wr_data[0] = READ_RX_BUF | (index == 0 ? 0x00 : 0x40);
        xfer[0].tx_buf = (unsigned long)wr_data;
        xfer[0].len = sizeof(wr_data);
        xfer[1].rx_buf = (unsigned long)buffer;
        xfer[1].len = sizeof(buffer);
        ::ioctl(spi, SPI_IOC_MESSAGE(2), xfer);
    }

    /** Write to a SPI register.
     * @param address address to write to
     * @param data data to write
     */
    void register_write(Registers address, uint8_t data)
    {
        uint8_t payload[] = {WRITE, address, data};
        ::write(spi, payload, sizeof(payload));
    }

    /** Write to a TX buffer.
     * @param index buffer index to write to (valid values are 0 through 2)
     * @param buffer pointer to a buffer structure to fill in with the result
     */
    void buffer_write(int index, Buffer *buffer)
    {
        spi_ioc_transfer xfer[2];
        memset(xfer, 0, sizeof(xfer));
        uint8_t instruction[1];
        instruction[0] = LOAD_TX_BUF + (0x01 << index);
        xfer[0].tx_buf = (unsigned long)instruction;
        xfer[0].len = sizeof(instruction);
        xfer[1].tx_buf = (unsigned long)buffer;
        xfer[1].len = sizeof(buffer);
        ::ioctl(spi, SPI_IOC_MESSAGE(2), xfer);
    }

    /** Bit modify to a SPI register.
     * @param address address to modify
     * @param data data to modify
     * @param mask mask of data to modify
     */
    void bit_modify(Registers address, uint8_t data, uint8_t mask)
    {
        uint8_t payload[] = {BIT_MODIFY, address, mask, data};
        ::write(spi, payload, sizeof(payload));
    }

    /** Request a transmit buffer to send.
     * @param index buffer index to request (valid values are 0 through 2)
     */
    void request_to_send(int index)
    {
        uint8_t rts = RTS | (0x01 << index);
        ::write(spi, &rts, 1);
    }

    unsigned txPending; /**< transmission in flight */
    int spi; /**< SPI bus that accesses MCP2515 */
    OSSem sem; /**< semaphore for posting events */

    /** Default constructor.
     */
    MCP2515Can();

    DISALLOW_COPY_AND_ASSIGN(MCP2515Can);
};

#endif /* _FREERTOS_DRIVERS_COMMON_MCP2515CAN_HXX_ */

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

#include "os/Gpio.hxx"
#include "os/OS.hxx"

#define MCP2515_DEBUG 0

class MCP2515GPO;
class MCP2515GPI;

/** Specialization of CAN driver for Tiva CAN.
 */
class MCP2515Can : public Can, public OSThread
{
public:
    /** Constructor.
     * @param name name of this device instance in the file system
     * @param interrupt_enable callback to enable the interrupt
     * @param interrupt_disable callback to disable the interrupt
     */
    MCP2515Can(const char *name,
               void (*interrupt_enable)(), void (*interrupt_disable)())
        : Can(name)
        , OSThread()
        , interrupt_enable(interrupt_enable)
        , interrupt_disable(interrupt_disable)
        , txPending(0)
        , gpoData(0x0)
        , gpiData(0x7)
        , ioPending(false)
        , spi(-1)
        , sem()
    {
    }

    /** Destructor.
     */
    ~MCP2515Can()
    {
    }

    /** Initialize CAN device settings.  Typically called in hw_postinit(), not
     * hw_preinit() or hw_init().
     * @param spi_name spi interface that the MCP2515Can is on
     * @param freq frequency in Hz that the MCP2515 clock runs at
     * @param baud target baud rate in Hz
     */
    void init(const char *spi_name, uint32_t freq, uint32_t baud);

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
        TXRTSCTRL, /**< TX RTS I/O control */
        CANSTAT,   /**< status */
        CANCTRL,   /**< control */

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
        TEC,      /**< transmit error counter */
        RED,      /**< receive error counter */

        RXM0SIDH = 32,
        RXM0SIDL,
        RXM0EID8,
        RXM0EID0,
        RXM1SIDH,
        RXM1SIDL,
        RXM1EID8,
        RXM1EID0,
        CNF3,     /**< configuration register 3 */
        CNF2,     /**< configuration register 3 */
        CNF1,     /**< configuration register 3 */
        CANINTE,  /**< interrupt enable */
        CANINTF,  /**< interrupt flags */
        EFLG,     /**< error flags */

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

    /** Fields of the control register */
    enum ControlFields
    {
        CLKPRE = 0x03, /**< clockout pin prescaler */
        CLKEN  = 0x04, /**< clockout pin enable */
        OSM    = 0x08, /**< one-shot mode */
        ABAT   = 0x10, /**< abort all pending transmissions */
        REQOP  = 0xE0, /**< request operation mode */
    };

    /** interrupt flag masks */
    enum InterruptFlags
    {
        RX0I = 0x01, /**< receive buffer 0 interrupt bit */
        RX1I = 0x02, /**< receive buffer 1 interrupt bit */
        TX0I = 0x04, /**< transmit buffer 0 interrupt bit */
        TX1I = 0x08, /**< transmit buffer 1 interrupt bit */
        TX2I = 0x10, /**< transmit buffer 2 interrupt bit */
        ERRI = 0x20, /**< error interrupt bit */
        WAKI = 0x40, /**< wake-up interrupt bit */
        MERR = 0x80, /**< message error interrupt bit */
    };

    /** interrupt flag masks */
    enum ErrorFlags
    {
        EWARN  = 0x01, /**< set when TEC or REC is equal to or greater than 96 */
        RXWARN = 0x02, /**< set when REC is equal to or greater than 96 */
        TXWARN = 0x04, /**< set when TEC is equal to or greater than 96 */
        RXEP   = 0x08, /**< set when REC is equal to or greater than 128 */
        TXEP   = 0x10, /**< set when TEC is equal to or greater than 128 */
        TXBO   = 0x20, /**< set when TEC reaches 255 (bus-off) */
        RX0OVR = 0x40, /**< receiver buffer 0 overflow flag */
        RX1OVR = 0x80, /**< receiver buffer 1 overflow flag */
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

    /** Configuration resgister 1 structure */
    struct Config1
    {
        /** Constructor.
         * @param brp initial value
         * @param sjw initial value
         */
        Config1(uint8_t brp, uint8_t sjw)
            : brp(brp)
            , sjw(sjw)
        {
        }

        union
        {
            uint8_t data; /**< raw data for register */
            struct
            {
                uint8_t brp : 6; /**< baud rate prescaler bits */
                uint8_t sjw : 2; /**< synchronization jump width length bits */
            };
        };
    };

    /** Configuration resgister 2 structure */
    struct Config2
    {
        /** Constructor.
         * @param prseg initial value
         * @param prseg1 initial value
         * @param sam initial value
         * @param bltmode initial value
         */
        Config2(uint8_t prseg, uint8_t prseg1, uint8_t sam, uint8_t bltmode)
            : prseg(prseg)
            , prseg1(prseg1)
            , sam(sam)
            , bltmode(bltmode)
        {
        }

        union
        {
            uint8_t data; /**< raw data for register */
            struct
            {
                uint8_t prseg   : 3; /**< propagation segment length bits */
                uint8_t prseg1  : 3; /**< PS1 length bits */
                uint8_t sam     : 1; /**< sample point configuration bit */
                uint8_t bltmode : 1; /**< PS2 bit time length bit */
            };
        };
    };

    /** Configuration resgister 3 structure */
    struct Config3
    {
        /** Constructor.
         * @param phseg2 initial value
         * @param wakfil initial value
         * @param sof initial value
         */
        Config3(uint8_t phseg2, uint8_t wakfil, uint8_t sof)
            : phseg2(phseg2)
            , unused(0)
            , wakfil(wakfil)
            , sof(sof)
        {
        }

        union
        {
            uint8_t data; /**< raw data for register */
            struct
            {
                uint8_t phseg2 : 3; /**< PS2 length bits */
                uint8_t unused : 3; /**< unused bits */
                uint8_t wakfil : 1; /**< wake-up filter bit */
                uint8_t sof    : 1; /**< start of frame signal bit */
            };
        };
    };

    /** Baud rate table entry */
    struct MCP2515Baud
    {
        uint32_t freq; /**< incoming frequency */
        uint32_t baud; /**< target baud rate */
        Config1 cnf1; /**< Configuration registers CNF1 */
        Config2 cnf2; /**< Configuration registers CNF2 */
        Config3 cnf3; /**< Configuration registers CNF3 */
    };

    /** CAN TX and RX buffer structure */
    class Buffer
    {
    public:
        /** Constructor.
         * @param can_frame reference to a can_frame metadata structure.
         */
        Buffer(struct can_frame *can_frame)
            : sidh(can_frame->can_eff ? (can_frame->can_id & 0x1FE00000) >> 21 :
                                        (can_frame->can_id & 0x000007F8) >> 3)
            , eid(can_frame->can_eff ? (can_frame->can_id & 0x00030000) >> 16 :
                                       0)
            , exide(can_frame->can_eff)
            , sid(can_frame->can_eff ? (can_frame->can_id & 0x001C0000) >> 18 :
                                       (can_frame->can_id & 0x000007F8) >> 3)
            , eid8(can_frame->can_eff ? (can_frame->can_id & 0x0000FF00) >> 8 :
                                        0)
            , eid0(can_frame->can_eff ? (can_frame->can_id & 0x000000FF) >> 0 :
                                        0)
            , dlc(can_frame->can_dlc)
            , rtr(can_frame->can_rtr)
        {
            memcpy(data, can_frame->data, 8);
        }

        /** Constructor.
         */
        Buffer()
        {
        }

        /** Take the contents of the Buffer and fill in a can_frame structure.
         * @param can_frame CAN frame structure to fill in
         */
        void build_struct_can_frame(struct can_frame *can_frame)
        {
            can_frame->can_eff = exide;
            if (exide)
            {
                /* extended frame */
                can_frame->can_rtr = rtr;
                can_frame->can_id = ((uint32_t)eid0 <<  0) +
                                    ((uint32_t)eid8 <<  8) +
                                    ((uint32_t)eid  << 16) +
                                    ((uint32_t)sid  << 18) +
                                    ((uint32_t)sidh << 21);
            }
            else
            {
                /* standard frame */
                can_frame->can_rtr = srr;
                can_frame->can_id = ((uint32_t)sid  << 0) +
                                    ((uint32_t)sidh << 3);
            }
            can_frame->can_err = 0;
            memcpy(can_frame->data, data, 8);
            can_frame->can_dlc = dlc;
        }

        /** Get a pointer to the buffer payload.
         * @return pointer to the buffer payload
         */
        void *get_payload()
        {
            return &sidh;
        }

    private:
        uint8_t sidh; /**< standard identifier high byte */
        struct
        {
            uint8_t eid     : 2; /**< extended identifier bits 16 and 17 */
            uint8_t unused1 : 1; /**< unused bit */
            uint8_t exide   : 1; /**< extended identifer enable */
            uint8_t srr     : 1; /**< standard frame RTR */
            uint8_t sid     : 3; /**< standard identifier low bits */
        };
        uint8_t eid8; /**< extended identifier high byte */
        uint8_t eid0; /**< extended identifier low byte */
        struct
        {
            uint8_t dlc     : 4; /**< data length code */
            uint8_t unused3 : 2; /**< unused bits */
            uint8_t rtr     : 1; /**< remote transmit request bit */
            uint8_t unused4 : 1; /**< unused bit */
        };
        uint8_t data[8]; /** all 8 data bytes */
    };

    /** User entry point for the created thread.
     * @return exit status
     */
    void *entry() override; /**< entry point to thread */

    void (*interrupt_enable)(); /**< enable interrupt callback */
    void (*interrupt_disable)(); /**< disable interrupt callback */

    void enable() override; /**< function to enable device */
    void disable() override; /**< function to disable device */

    /** Function to try and transmit a message. */
    void tx_msg() override
    {
        /* The caller has us in a critical section, we need to exchange a
         * critical section lock for a mutex lock since event handling happens
         * in thread context and not in interrupt context like it would in a
         * typical CAN device driver.
         */
        portEXIT_CRITICAL();
        lock_.lock();
        tx_msg_locked();
        lock_.unlock();
        portENTER_CRITICAL();
    }

    /** Function to try and transmit a message while holding a lock. */
    void tx_msg_locked();

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
        xfer[1].cs_change = 1;
        ::ioctl(spi, SPI_IOC_MESSAGE(2), xfer);

        return rd_data[0];
    }

    /** Read from a RX buffer.
     * @param index buffer index to read from (valid values are 0 and 1)
     * @param buffer pointer to a buffer structure to fill in with the result
     */
    void buffer_read(int index, void *buffer)
    {
        spi_ioc_transfer xfer[2];
        memset(xfer, 0, sizeof(xfer));
        uint8_t wr_data[1];
        wr_data[0] = READ_RX_BUF | (index == 0 ? 0x00 : 0x40);
        xfer[0].tx_buf = (unsigned long)wr_data;
        xfer[0].len = sizeof(wr_data);
        xfer[1].rx_buf = (unsigned long)buffer;
        xfer[1].len = 13;
        xfer[1].cs_change = 1;
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
    void buffer_write(int index, void *buffer)
    {
        spi_ioc_transfer xfer[2];
        memset(xfer, 0, sizeof(xfer));
        uint8_t instruction[1];
        instruction[0] = LOAD_TX_BUF + (index << 1);
        xfer[0].tx_buf = (unsigned long)instruction;
        xfer[0].len = sizeof(instruction);
        xfer[1].tx_buf = (unsigned long)buffer;
        xfer[1].len = 13;
        xfer[1].cs_change = 1;
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

    void request_io_operation()
    {
        if (!ioPending)
        {
            ioPending = true;
            sem.post();
        }
    }

    unsigned txPending : 2; /**< transmission in flight */
    unsigned gpoData   : 2; /**< local copy of the I/O expansion output data */
    unsigned gpiData   : 3; /**< local copy of the I/O expansion input data */
    unsigned ioPending : 1; /**< true if an I/O update is pending */
    unsigned reserved  : 24; /**< unused bits */

    int spi; /**< SPI bus that accesses MCP2515 */
    OSSem sem; /**< semaphore for posting events */
#if MCP2515_DEBUG
    volatile uint8_t regs[128]; /**< debug copy of MCP2515 registers */
#endif

    /** baud rate settings table */
    static const MCP2515Baud baudTable[];

    /** Default constructor.
     */
    MCP2515Can();

    /** Allow access to MCP2515Can from MCP2515GPO */
    friend class MCP2515GPO;

    /** Allow access to MCP2515Can from MCP2515GPI */
    friend class MCP2515GPI;

    DISALLOW_COPY_AND_ASSIGN(MCP2515Can);
};

/** General Purpose Output (GPO) instance on the MCP2515.
 */
class MCP2515GPO : public Gpio
{
public:
    /** Constructor.
     * @param instance parrent MCP2515Can instance that "owns" the interface.
     * @param bit bit index (0 through 1) of the output.
     */
    MCP2515GPO(MCP2515Can *instance, uint8_t bit)
        : Gpio()
        , instance_(instance)
        , bit_(bit)
    {
        HASSERT(bit < 2);
    }

    /** Writes a GPO pin (set or clear to a specific state).
     * @param new_state the desired output state.  See @ref Value.
     */
    void write(Value new_state) const override
    {
        new_state ? set() : clr();
    }

    /** Retrieves the current @ref Value of a GPO output sate (requested).
     * @return @ref SET if currently high, @ref CLR if currently low
     */
    Value read() const override
    {
        return instance_->gpoData & (0x1 << bit_) ? Gpio::SET : Gpio::CLR;
    }

    /** Sets the GPO pin to high.
     */
    void set() const override
    {
        portENTER_CRITICAL();
        instance_->gpoData |= 0x1 << bit_;
        portEXIT_CRITICAL();
        instance_->request_io_operation();
    }

    /** Clears the GPO pin to low.
     */
    void clr() const override
    {
        portENTER_CRITICAL();
        instance_->gpoData &= ~(0x1 << bit_);
        portEXIT_CRITICAL();
        instance_->request_io_operation();
    }

    /** Sets the GPO direction (does nothing).
     * @param dir @ref INPUT or @ref OUTPUT
     */
    void set_direction(Gpio::Direction dir) const override
    {
        HASSERT(dir == Gpio::Direction::OUTPUT);
    }

    /** Gets the GPO direction.
     * @return always returns @ref OUTPUT
     */
    Direction direction() const override
    {
        return Gpio::Direction::OUTPUT;
    }

private:
    /** reference to chip instance */
    MCP2515Can *instance_;

    /** bit number representative of the bit */
    uint8_t bit_;

    DISALLOW_COPY_AND_ASSIGN(MCP2515GPO);
};

/** General Purpose Input (GPI) instance on the MCP2515.
 */
class MCP2515GPI : public Gpio
{
public:
    /** Constructor.
     * @param instance parrent MCP2515Can instance that "owns" the interface.
     * @param bit bit index (0 through 2) of the input.
     */
    MCP2515GPI(MCP2515Can *instance, uint8_t bit)
        : Gpio()
        , instance_(instance)
        , bit_(bit)
    {
        HASSERT(bit < 3);
    }

    /** Retrieves the current @ref Value of a GPI input pin.
     * @return @ref SET if currently high, @ref CLR if currently low
     */
    Value read() const override
    {
        instance_->request_io_operation();
        return instance_->gpiData & (0x1 << bit_) ? Gpio::SET : Gpio::CLR;
    }

    /** Sets the GPI direction (does nothing).
     * @param dir @ref INPUT or @ref OUTPUT
     */
    void set_direction(Gpio::Direction dir) const override
    {
        HASSERT(dir == Gpio::Direction::INPUT);
    }

    /** Gets the GPI direction.
     * @return always returns @ref INPUT
     */
    Direction direction() const override
    {
        return Gpio::Direction::INPUT;
    }

private:
    /** reference to chip instance */
    MCP2515Can *instance_;

    /** bit number representative of the bit */
    uint8_t bit_;

    DISALLOW_COPY_AND_ASSIGN(MCP2515GPI);
};

#endif /* _FREERTOS_DRIVERS_COMMON_MCP2515CAN_HXX_ */

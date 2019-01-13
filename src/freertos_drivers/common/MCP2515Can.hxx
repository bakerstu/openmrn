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
#include <cstddef>

#include "Can.hxx"
#include "SPI.hxx"

#include "os/Gpio.hxx"
#include "os/OS.hxx"

#include "can_ioctl.h"

#define MCP2515_DEBUG 0
#define MCP2515_NULL_TX 0

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
        , interruptEnable_(interrupt_enable)
        , interruptDisable_(interrupt_disable)
        , state_(CAN_STATE_STOPPED)
        , txPending_(0)
        , gpoData_(0x0)
        , gpiData_(0x7)
        , ioPending_(false)
        , spiFd_(-1)
        , spi_(nullptr)
        , sem_()
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

    /** Return a mutex that can be used by another SPI driver instance sharing
     * the same bus as its bus lock.
     * @return a reference to a mutex that can be used as a bus lock
     */
    OSMutex *get_spi_bus_lock()
    {
        return &lock_;
    }

private:
    /** maximum SPI clock speed in Hz */
    static constexpr uint32_t SPI_MAX_SPEED_HZ = 10000000;

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
            : brp_(brp)
            , sjw_(sjw)
        {
        }

        union
        {
            uint8_t data_; /**< raw data for register */
            struct
            {
                uint8_t brp_ : 6; /**< baud rate prescaler bits */
                uint8_t sjw_ : 2; /**< synchronization jump width length bits */
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
            : prseg_(prseg)
            , prseg1_(prseg1)
            , sam_(sam)
            , bltmode_(bltmode)
        {
        }

        union
        {
            uint8_t data_; /**< raw data for register */
            struct
            {
                uint8_t prseg_   : 3; /**< propagation segment length bits */
                uint8_t prseg1_  : 3; /**< PS1 length bits */
                uint8_t sam_     : 1; /**< sample point configuration bit */
                uint8_t bltmode_ : 1; /**< PS2 bit time length bit */
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
            : phseg2_(phseg2)
            , unused_(0)
            , wakfil_(wakfil)
            , sof_(sof)
        {
        }

        union
        {
            uint8_t data_; /**< raw data for register */
            struct
            {
                uint8_t phseg2_ : 3; /**< PS2 length bits */
                uint8_t unused_ : 3; /**< unused bits */
                uint8_t wakfil_ : 1; /**< wake-up filter bit */
                uint8_t sof_    : 1; /**< start of frame signal bit */
            };
        };
    };

    /** Baud rate table entry */
    struct MCP2515Baud
    {
        uint32_t freq_; /**< incoming frequency */
        uint32_t baud_; /**< target baud rate */
        Config1 cnf1_; /**< Configuration registers CNF1 */
        Config2 cnf2_; /**< Configuration registers CNF2 */
        Config3 cnf3_; /**< Configuration registers CNF3 */
    };

    /** CAN TX and RX buffer structure */
    class Buffer
    {
    public:
        /** The SPI transfer size in bytes of a buffer transfer */
        static const size_t TRANSFER_SIZE = 14;

        /** Take the contents of the Buffer and fill in a can_frame structure.
         * @param can_frame CAN frame structure to fill in
         */
        __attribute__((optimize("-O3")))
        void build_struct_can_frame(struct can_frame *can_frame)
        {
            can_frame->can_eff = exide_;
            if (LIKELY(exide_))
            {
                /* extended frame */
                can_frame->can_rtr = rtr_;
                can_frame->can_id = ((uint32_t)eid0_ <<  0) +
                                    ((uint32_t)eid8_ <<  8) +
                                    ((uint32_t)eid_  << 16) +
                                    ((uint32_t)sid_  << 18) +
                                    ((uint32_t)sidh_ << 21);
            }
            else
            {
                /* standard frame */
                can_frame->can_rtr = srr_;
                can_frame->can_id = ((uint32_t)sid_  << 0) +
                                    ((uint32_t)sidh_ << 3);
            }
            can_frame->can_err = 0;
            can_frame->can_dlc = dlc_;
            static_assert(offsetof(Buffer, data_) == 8, "data_ misaligned");
            can_frame->data64 = data64_;
        }

        /** Get a pointer to the buffer payload.
         * @return pointer to the buffer payload
         */
        void *get_payload()
        {
            return &command_;
        }

    protected:
        /** Constructor.
         * @param can_frame reference to a can_frame metadata structure.
         * @param command corresponding command for the buffer
         */
        __attribute__((optimize("-O3")))
        Buffer(struct can_frame *can_frame, uint8_t command)
            : command_(command)
            , dlc_(can_frame->can_dlc)
            , rtr_(can_frame->can_rtr)
        {
            if (LIKELY(can_frame->can_eff))
            {
                sidh_ = (can_frame->can_id & 0x1FE00000) >> 21;
                eid_ = (can_frame->can_id & 0x00030000) >> 16;
                exide_ = 1;
                sid_ = (can_frame->can_id & 0x001C0000) >> 18;
                eid8_ = (can_frame->can_id & 0x0000FF00) >> 8;
                eid0_ = (can_frame->can_id & 0x000000FF) >> 0;
            }
            else
            {
                sidh_ = (can_frame->can_id & 0x000007F8) >> 3;
                eid_ = 0;
                exide_ = 0;
                sid_ = (can_frame->can_id & 0x000007F8) >> 3;
                eid8_ = 0;
                eid0_ = 0;
            }
            static_assert(offsetof(Buffer, data_) == 8, "data_ misaligned");
            data64_ = can_frame->data64;
        }

        /** Constructor.
         * @param command corresponding command for the buffer
         */
        Buffer( uint8_t command)
            : command_(command)
        {
        }

        uint8_t pad_[2];          /**< force the data to 32/64-bit allign */
        uint8_t command_;         /**< the transaction command */
        uint8_t sidh_;            /**< standard identifier high byte */
        struct
        {
            uint8_t eid_     : 2; /**< extended identifier bits 16 and 17 */
            uint8_t unused1_ : 1; /**< unused bit */
            uint8_t exide_   : 1; /**< extended identifer enable */
            uint8_t srr_     : 1; /**< standard frame RTR */
            uint8_t sid_     : 3; /**< standard identifier low bits */
        };
        uint8_t eid8_;            /**< extended identifier high byte */
        uint8_t eid0_;            /**< extended identifier low byte */
        struct
        {
            uint8_t dlc_     : 4; /**< data length code */
            uint8_t unused3_ : 2; /**< unused bits */
            uint8_t rtr_     : 1; /**< remote transmit request bit */
            uint8_t unused4_ : 1; /**< unused bit */
        };
        union
        {
            uint64_t data64_; /**< 64-bit data */
            uint8_t  data_[8];     /**< all 8 data bytes */
        };
    };

    /** Setup a buffer read transfer structure.
     */
    class BufferRead : public Buffer
    {
    public:
        /** Constructor.
         * @param index buffer index to read from (valid values are 0 and 1)
         */
        BufferRead(int index)
            : Buffer(READ_RX_BUF | (index == 0 ? 0x00 : 0x04))
        {
        }
    };

    /** Setup a buffer read transfer structure.
     */
    class BufferWrite : public Buffer
    {
    public:
        /** Constructor.
         * @param index buffer index to read from (valid values are 0 and 1)
         * @param can_frame reference to a can_frame metadata structure.
         */
        BufferWrite(int index, struct can_frame *can_frame)
            : Buffer(can_frame, LOAD_TX_BUF + (index << 1))
        {
        }
    };

    /** Request an ioctl transaction.
     * @param file file reference for this device
     * @param key ioctl key
     * @param data key data
     * @return >= 0 upon success, -errno upon failure
     */
    int ioctl(File *file, unsigned long int key, unsigned long data) override;

    /** User entry point for the created thread.
     * @return exit status
     */
    void *entry() override; /**< entry point to thread */

    void enable() override; /**< function to enable device */
    void disable() override; /**< function to disable device */

    /** Function to try and transmit a message. */
    __attribute__((optimize("-O3")))
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

    /** Reset the device.
     */
    void reset()
    {
        spi_ioc_transfer xfer;
        uint8_t reset = RESET;

        xfer.tx_buf = (unsigned long)&reset;
        xfer.rx_buf = 0;
        xfer.len = sizeof(reset);

        spi_->transfer_with_cs_assert_polled(&xfer);
    }

    /** Read from a SPI register.
     * @param address address to read from
     * @return data read
     */
    __attribute__((optimize("-O3")))
    uint8_t register_read(Registers address)
    {
        spi_ioc_transfer xfer;
        uint8_t data[3] = {READ, address, 0};

        xfer.tx_buf = (unsigned long)data;
        xfer.rx_buf = (unsigned long)data;
        xfer.len = sizeof(data);

        spi_->transfer_with_cs_assert_polled(&xfer);

        return data[2];
    }

    /** Write to a SPI register.
     * @param address address to write to
     * @param data data to write
     */
    __attribute__((optimize("-O3")))
    void register_write(Registers address, uint8_t data)
    {
        spi_ioc_transfer xfer;
        uint8_t payload[] = {WRITE, address, data};

        xfer.tx_buf = (unsigned long)payload;
        xfer.rx_buf = 0;
        xfer.len = sizeof(payload);

        spi_->transfer_with_cs_assert_polled(&xfer);
    }

    /** Bit modify to a SPI register.
     * @param address address to modify
     * @param data data to modify
     * @param mask mask of data to modify
     */
    void bit_modify(Registers address, uint8_t data, uint8_t mask)
    {
        spi_ioc_transfer xfer;
        uint8_t payload[] = {BIT_MODIFY, address, mask, data};

        xfer.tx_buf = (unsigned long)payload;
        xfer.rx_buf = 0;
        xfer.len = sizeof(payload);
        spi_->transfer_with_cs_assert_polled(&xfer);
    }

    /** Read a message to into a receive buffer.
     * @param buf BuffeRead object to fill in
     */
    void buffer_read(BufferRead *buf)
    {
        spi_ioc_transfer xfer;
        xfer.tx_buf = (unsigned long)buf->get_payload();
        xfer.rx_buf = (unsigned long)buf->get_payload();
        xfer.len = buf->TRANSFER_SIZE;
        spi_->transfer_with_cs_assert_polled(&xfer);
    }

    /** Write a message to a transmit buffer.
     * @param buf BufferWrite object to write out over SPI
     * @param can_frame reference to a can_frame metadata structure.
     */
    void buffer_write(BufferWrite *buf, struct can_frame *can_frame)
    {
        spi_ioc_transfer xfer;
        xfer.tx_buf = (unsigned long)buf->get_payload();
        xfer.rx_buf = 0;
        xfer.len = buf->TRANSFER_SIZE;
        spi_->SPI::transfer_with_cs_assert_polled(&xfer);
    }

    /** Request that the GPIO cache be refreshed.
     */
    void request_gpio_operation()
    {
        if (!ioPending_)
        {
            ioPending_ = true;
            sem_.post();
        }
    }

    void (*interruptEnable_)(); /**< enable interrupt callback */
    void (*interruptDisable_)(); /**< disable interrupt callback */
    unsigned state_     : 4; /**< present bus state */
    unsigned txPending_ : 2; /**< transmission in flight */
    unsigned gpoData_   : 2; /**< local copy of the I/O expansion output data */
    unsigned gpiData_   : 3; /**< local copy of the I/O expansion input data */
    unsigned ioPending_ : 1; /**< true if an I/O update is pending */
    int spiFd_; /**< SPI bus that accesses MCP2515 */
    SPI *spi_; /**< pointer to a SPI object instance */
    OSSem sem_; /**< semaphore for posting events */
#if MCP2515_DEBUG
    volatile uint8_t regs_[128]; /**< debug copy of MCP2515 registers */
#endif

    /** baud rate settings table */
    static const MCP2515Baud BAUD_TABLE[];

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
        return instance_->gpoData_ & (0x1 << bit_) ? Gpio::SET : Gpio::CLR;
    }

    /** Sets the GPO pin to high.
     */
    void set() const override
    {
        if (!(instance_->gpoData_ & (0x1 << bit_)))
        {
            portENTER_CRITICAL();
            instance_->gpoData_ |= 0x1 << bit_;
            portEXIT_CRITICAL();
            instance_->request_gpio_operation();
        }
    }

    /** Clears the GPO pin to low.
     */
    void clr() const override
    {
        if ((instance_->gpoData_ & (0x1 << bit_)))
        {
            portENTER_CRITICAL();
            instance_->gpoData_ &= ~(0x1 << bit_);
            portEXIT_CRITICAL();
            instance_->request_gpio_operation();
        }
    }

    /** Sets the GPO direction (does nothing).
     * @param dir @ref INPUT or @ref OUTPUT
     */
    void set_direction(Gpio::Direction dir) const override
    {
        HASSERT(dir == Gpio::Direction::DOUTPUT);
    }

    /** Gets the GPO direction.
     * @return always returns @ref OUTPUT
     */
    Direction direction() const override
    {
        return Gpio::Direction::DOUTPUT;
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
        instance_->request_gpio_operation();
        return instance_->gpiData_ & (0x1 << bit_) ? Gpio::SET : Gpio::CLR;
    }

    /** Sets the GPI direction (does nothing).
     * @param dir @ref INPUT or @ref OUTPUT
     */
    void set_direction(Gpio::Direction dir) const override
    {
        HASSERT(dir == Gpio::Direction::DINPUT);
    }

    /** Gets the GPI direction.
     * @return always returns @ref INPUT
     */
    Direction direction() const override
    {
        return Gpio::Direction::DINPUT;
    }

private:
    /** reference to chip instance */
    MCP2515Can *instance_;

    /** bit number representative of the bit */
    uint8_t bit_;

    DISALLOW_COPY_AND_ASSIGN(MCP2515GPI);
};

#endif /* _FREERTOS_DRIVERS_COMMON_MCP2515CAN_HXX_ */

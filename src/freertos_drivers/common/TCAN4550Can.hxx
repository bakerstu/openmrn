/** @copyright
 * Copyright (c) 2020 Stuart W Baker
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
 * @file TCAN4550Can.hxx
 * This file implements the CAN driver for the TCAN4550 CAN Controller.
 *
 * @author Stuart W. Baker
 * @date 26 February 2020
 */

#ifndef _FREERTOS_DRIVERS_COMMON_TCAN4550CAN_HXX_
#define _FREERTOS_DRIVERS_COMMON_TCAN4550CAN_HXX_

#include "Can.hxx"
#include "SPI.hxx"

#include "os/OS.hxx"

/// Specification of CAN driver for the TCAN4550.
/// @todo The TCAN4550 uses the Bosch MCAN IP. If we end up supporting other
///       devices that also use this IP, then some of the generic MCAN related
///       content can be factored out into a common location.
class TCAN4550Can : public Can, public OSThread
{
public:
    /// Constructor.
    /// @param name name of this device instance in the file system
    /// @param interrupt_enable callback to enable the interrupt
    /// @param interrupt_disable callback to disable the interrupt
    TCAN4550Can(const char *name,
                void (*interrupt_enable)(), void (*interrupt_disable)())
        : Can(name)
        , OSThread()
        , interruptEnable_(interrupt_enable)
        , interruptDisable_(interrupt_disable)
        , spiFd_(-1)
        , spi_(nullptr)
        , sem_()
    {
    }

    /// Destructor.
    ~TCAN4550Can()
    {
    }

    /// Initialize CAN device settings.  Typically called in hw_postinit(), not
    /// hw_preinit() or hw_init().
    /// @param spi_name spi interface that the TCAN4550Can is on
    /// @param freq frequency in Hz that the TCAN4550 clock runs at
    /// @param baud target baud rate in Hz
    void init(const char *spi_name, uint32_t freq, uint32_t baud);

    /// Handle an interrupt.  Called by user provided interrupt handler.
    void interrupt_handler();

    /// Return a mutex that can be used by another SPI driver instance sharing
    /// the same bus as its bus lock.
    /// @return a reference to a mutex that can be used as a bus lock
    OSMutex *get_spi_bus_lock()
    {
        return &lock_;
    }

private:
    /// maximum SPI clock speed in Hz
    static constexpr uint32_t SPI_MAX_SPEED_HZ = 18000000;

    static constexpr size_t MRAM_SIZE_WORDS = (2 * 1024) / 4;

    /// SPI Registers, word addressing, not byte addressing
    enum Registers : uint16_t
    {
        DEVICE_IDL = 0x0, ///< device ID "TCAN"
        DEVICE_IDH,       ///< device ID "4550"
        REVISION,         ///< silicon revision
        STATUS,           ///< status

        MODE = 0x200,        ///< modes of operation and pin configurations
        TIMESTAMP_PRESCALER, ///< timestamp presacaler
        TEST,                ///< read and write test registers, scratchpad
        ECC,                 ///< ECC error detection and testing

        INTERRUPT_STATUS = 0x208, ///< interrupt and diagnostic flags
        MCAN_INTERRUPT_STATUS,    ///< interrupt flags related to MCAN core

        INTERRUPT_ENABLE = 0x20C, ///< interrupt and diagnostic flags
        MCAN_INTERRUPT_ENABLE,    ///< interrupt flags related to MCAN core

        CREL = 0x400, ///< core release
        ENDN,         ///< endianess
        CUST,         ///< customer
        DBTP,         ///< data bit timing and prescaler
        TEST2,        ///< test
        RWD,          ///< RAM watchdog
        CCCR,         ///< CC control
        NBTP,         ///< nominal bit timing and prescaler
        TSCC,         ///< timestamp counter configuration
        TSCV,         ///< timestamp counter value
        TOCC,         ///< timeout counter configuration
        TOCV,         ///< timeout counter value
        RSVD1,        ///< reserved
        RSVD2,        ///< reserved
        RSVD3,        ///< reserved
        RSVD4,        ///< reserved
        ECR,          ///< error count
        PSR,          ///< protocol status
        TDCR,         ///< transmitter delay compensation
        RSVD5,        ///< reserved
        IR,           ///< interrupt status
        IE,           ///< interrupt enable
        ILS,          ///< interrupt line select
        ILE,          ///< interrupt line enable
        RSVD6,        ///< reserved
        RSVD7,        ///< reserved
        RSVD8,        ///< reserved
        RSVD9,        ///< reserved
        RSVD10,       ///< reserved
        RSVD11,       ///< reserved
        RSVD12,       ///< reserved
        RSVD13,       ///< reserved
        GFC,          ///< global filter configuration
        SIDFC,        ///< standard ID filter configuration
        XIDFC,        ///< extended ID filter configuration
        RSVD14,       ///< reserved
        XIDAM,        ///< extended ID and mask
        HPMS,         ///< high prioirty message status
        NDAT1,        ///< new data 1
        NDAT2,        ///< new data 2
        RXF0C,        ///< RX FIFO 0 configuration
        RXF0S,        ///< RX FIFO 0 status
        RXF0A,        ///< RX FIFO 0 Acknowledge
        RXBC,         ///< RX buffer configuration
        RXF1C,        ///< RX FIFO 1 configuration
        RXF1S,        ///< RX FIFO 1 status
        RXF1A,        ///< RX FIFO 1 acknowledge
        RXESC,        ///< RX buffer/FIFO element size configuration
        TXBC,         ///< TX buffer configuration
        TXFQS,        ///< TX FIFO/queue status
        TXESC,        ///< TX buffer ellement size configuration
        TXBRP,        ///< TX buffer request pending
        TXBAR,        ///< TX buffer add request
        TXBCR,        ///< TX buffer cancellation request
        TXBTO,        ///< TX buffer transmission occurred
        TXBCF,        ///< TX buffer cancellation finished
        TXBTIE,       ///< TX buffer transmission interrupt enable
        TXBCIE,       ///< TX buffer cancellation finished interrupt enable
        RSVD15,       ///< reserved
        RSVD16,       ///< reserved
        TXEFC,        ///< TX event FIFO configuration
        TXEFS,        ///< TX event FIFO status
        TXEFA,        ///< TX event FIFO acknowledge
        RSVD17,       ///< reserved
    };

    enum Command : uint8_t
    {
        WRITE = 0x61, ///< write one or more addresses
        READ  = 0x41, ///< read one or more addresses
    };

    /// Mode register definition
    struct Mode
    {
        /// Constructor. Sets the reset value.
        Mode()
            : data(0xC8000468)
        {
        }

        union
        {
            uint32_t data; ///< raw word value
            struct
            {
                uint32_t testModeConfig : 1; ///< test mode configuration
                uint32_t sweDis         : 1; ///< sleep wake error disable
                uint32_t reset          : 1; ///< device reset
                uint32_t wdEnable       : 1; ///< watchdog enable
                uint32_t reserved1      : 2; ///< reserved
                uint32_t modeSel        : 2; ///< mode of operation select
                uint32_t nWkrqConfig    : 1; ///< nWKRQ pin function
                uint32_t inhDisable     : 1; ///< INH pin disable
                uint32_t gpio1GpoConfig : 2; ///< GPIO1 output function select
                uint32_t reserved2      : 1; ///< reserved
                uint32_t failSafeEnable : 1; ///< fail safe mode enable
                uint32_t gpio1Config    : 2; ///< GPIO1 pin function select
                uint32_t wdAction       : 2; ///< selected watchdog action
                uint32_t wdBitSet       : 1; ///< write a '1' to reset timer
                uint32_t nWkrqVoltage   : 1; ///< nWKRQ pin GPO buffer voltage
                uint32_t reserved3      : 1; ///< reserved
                uint32_t testModeEn     : 1; ///< test mode enable
                uint32_t gpo2Config     : 2; ///< GPO2 pin configuration
                uint32_t reserved4      : 3; ///< reserved
                uint32_t clkRef         : 1; ///< CLKIN/crystal freq reference
                uint32_t wdTimer        : 2; ///< watchdog timer
                uint32_t wakeConfig     : 2; ///< Wake pin configuration
            };
        };
    };

    /// Data bit timing and prescaler register definition
    struct Dbpt
    {
        uint32_t dsjw      : 4; ///< data (re)synchronization jump width
        uint32_t dtseg2    : 4; ///< data time segment before sample point
        uint32_t dtseg1    : 5; ///< data time segment after sample point
        uint32_t reserved1 : 3; ///< reserved
        uint32_t dbrp      : 5; ///< data bit rate prescaler
        uint32_t reserved2 : 2; ///< reserved
        uint32_t tdc       : 1; ///< trasmitter delay compensation
        uint32_t reserved3 : 8; ///< reserved
    };

    /// CC control register definition
    struct Cccr
    {
        /// Constructor. Sets the reset value.
        Cccr()
            : data(0x00000001)
        {
        }

        union
        {
            uint32_t data; ///< raw word value
            struct
            {
                uint32_t init  : 1; ///< initialzation
                uint32_t cce   : 1; ///< configuration change enable
                uint32_t asm_  : 1; ///< restricted operation mode
                uint32_t csa   : 1; ///< clock stop acknowledge
                uint32_t csr   : 1; ///< clock stop request
                uint32_t mon   : 1; ///< bus monitorying mode is disabled
                uint32_t dar   : 1; ///< disable automatic retransmission
                uint32_t test  : 1; ///< test mode enable

                uint32_t fdoe  : 1; ///< FD operation enable
                uint32_t brse  : 1; ///< bit rate switch enable
                uint32_t rsvd1 : 2; ///< reserved
                uint32_t pxhd  : 1; ///< protocol exception handling disable
                uint32_t efbi  : 1; ///< edge filtering during bus integration
                uint32_t txp   : 1; ///< transmitter pause
                uint32_t niso  : 1; ///< non ISO operation

                uint32_t rsvd2 : 16; ///< reserved
            };
        };
    };

    /// Buad rate table entry
    struct TCAN4550Baud
    {
        uint32_t freq; ///< incoming frequency
        uint32_t baud; ///< target baud rate
        Dbpt     dbpt; ///< data bit timing and prescaler
    };

    /// SPI message for read/write commands
    struct SPIMessage
    {
        union
        {
            uint8_t payload[8]; ///< raw payload
            struct
            {
                uint8_t cmd;    ///< command
                uint8_t addrH;  ///< register address MSB
                uint8_t addrL;  ///< register address LSB
                uint8_t length; ///< length in words
                uint32_t data;  ///< data word
            };
        };
    };

    /// MRAM SPI message for read/write data
    struct MRAMMessage
    {
        union
        {
            uint8_t payload[0]; ///< raw payload
            struct
            {
                uint8_t cmd;    ///< command
                uint8_t addrH;  ///< register address MSB
                uint8_t addrL;  ///< register address LSB
                uint8_t length; ///< length in words
            };
        };
    };

    /// Structure that helps us clear the MRAM
    struct MRAMMessageClear : public MRAMMessage
    {
        static constexpr uint8_t DATA_SIZE = 32; ///< size of data in words

        /// Constructor.
        MRAMMessageClear()
        {
            length = DATA_SIZE;
            memset(data, 0, sizeof(data));
        }

        uint32_t data[DATA_SIZE]; ///< data words
    };

    /// RX Buffer structure
    struct MRAMRXBuffer
    {
        uint32_t id  : 29; ///< CAN identifier
        uint32_t rtr :  1; ///< remote transmission request
        uint32_t xtd :  1; ///< extended identifier
        uint32_t esi :  1; ///< error state indicator

        uint32_t rxts : 16; ///< receive timestamp
        uint32_t dlc  :  4; ///< data length code
        uint32_t brs  :  1; ///< bit rate switch
        uint32_t fdf  :  1; ///< FD format
        uint32_t rsvd :  2; ///< reserved
        uint32_t fidx :  7; ///< filter index that message mached if ANMF = 0
        uint32_t anmf :  1; ///< accepted non-matching frame of filter element

        uint32_t data[2]; ///< data payload
    };

    /// TX Buffer structure
    struct MRAMTXBuffer
    {
        uint32_t id  : 29; ///< CAN identifier
        uint32_t rtr :  1; ///< remote transmission request
        uint32_t xtd :  1; ///< extended identifier
        uint32_t esi :  1; ///< error state indicator

        uint32_t rsvd1 : 16; ///< receive timestamp
        uint32_t dlc   :  4; ///< data length code
        uint32_t brs   :  1; ///< bit rate switch
        uint32_t fdf   :  1; ///< FD format
        uint32_t rsvd2 :  1; ///< reserved
        uint32_t efc   :  1; ///< event FIFO control
        uint32_t mm    :  8; ///< message marker

        uint32_t data[2]; ///< data payload
    };

    /// TX Event FIFO Element structure
    struct MRAMTXEventFIFOElement
    {
        uint32_t id  : 29; ///< CAN identifier
        uint32_t rtr :  1; ///< remote transmission request
        uint32_t xtd :  1; ///< extended identifier
        uint32_t esi :  1; ///< error state indicator

        uint32_t txts : 16; ///< transmit timestacmp
        uint32_t dlc  :  4; ///< data length code
        uint32_t brs  :  1; ///< bit rate switch
        uint32_t fdf  :  1; ///< FD format
        uint32_t et   :  2; ///< event type
        uint32_t mm   :  8; ///< message marker
    };

    /// User entry point for the created thread.
    /// @return exit status
    void *entry() override;

    void enable() override; ///< function to enable device
    void disable() override; ///< function to disable device

    /// Function to try and transmit a message.
    __attribute__((optimize("-O3")))
    void tx_msg() override
    {
    }

    /// Read from a SPI register.
    /// @param address address to read from
    /// @return data read
    __attribute__((optimize("-O3")))
    uint32_t register_read(Registers address)
    {
        SPIMessage msg;
        msg.cmd = READ;
        msg.addrH = address >> 6;
        msg.addrL = (address << 2) & 0xFF;
        msg.length = 1;

        spi_ioc_transfer xfer;
        xfer.tx_buf = (unsigned long)(&msg);
        xfer.rx_buf = (unsigned long)(&msg);
        xfer.len = sizeof(msg);

        spi_->transfer_with_cs_assert_polled(&xfer);

        return be32toh(msg.data);
    }

    /// Write to a SPI register.
    /// @param address address to write to
    /// @param data data to write
    __attribute__((optimize("-O3")))
    void register_write(Registers address, uint32_t data)
    {
        SPIMessage msg;
        msg.cmd = WRITE;
        msg.addrH = address >> 6;
        msg.addrL = (address << 2) & 0xFF;
        msg.length = 1;
        msg.data = htobe32(data);

        spi_ioc_transfer xfer;
        xfer.tx_buf = (unsigned long)(&msg);
        xfer.rx_buf = (unsigned long)(&msg);
        xfer.len = sizeof(msg);

        spi_->transfer_with_cs_assert_polled(&xfer);
    }


    /// Read from a SPI register.
    /// @param offset word offset to read from
    /// @param msg message to send
    /// @param xfer_size total SPI transfer size
    __attribute__((optimize("-O3")))
    void mram_read(uint16_t offset, MRAMMessage *msg, uint32_t xfer_size)
    {
        uint16_t address = offset + 0x8000;
        msg->cmd = READ;
        msg->addrH = address >> 8;
        msg->addrL = 0xFF;

        spi_ioc_transfer xfer;
        xfer.tx_buf = (unsigned long)(msg);
        xfer.rx_buf = (unsigned long)(msg);
        xfer.len = sizeof(msg);

        spi_->transfer_with_cs_assert_polled(&xfer);
    }

    /// Write to a SPI register.
    /// @param offset word offset to write to
    /// @param msg message to send
    /// @param xfer_size total SPI transfer size
    __attribute__((optimize("-O3")))
    void mram_write(uint16_t offset, MRAMMessage *msg, uint32_t xfer_size)
    {
        uint16_t address = offset + 0x8000;
        msg->cmd = WRITE;
        msg->addrH = address >> 8;
        msg->addrL = 0xFF;

        spi_ioc_transfer xfer;
        xfer.tx_buf = (unsigned long)(msg);
        xfer.rx_buf = (unsigned long)(msg);
        xfer.len = xfer_size;

        spi_->transfer_with_cs_assert_polled(&xfer);
    }

    void (*interruptEnable_)(); ///< enable interrupt callback
    void (*interruptDisable_)(); ///< disable interrupt callback
    int spiFd_; ///< SPI bus that accesses MCP2515
    SPI *spi_; ///< pointer to a SPI object instance
    OSSem sem_; ///< semaphore for posting events

    /// baud rate settings table
    static const TCAN4550Baud BAUD_TABLE[];

    /// Default Constructor.
    TCAN4550Can();

    DISALLOW_COPY_AND_ASSIGN(TCAN4550Can);
};

#endif // _FREERTOS_DRIVERS_COMMON_TCAN4550CAN_HXX_


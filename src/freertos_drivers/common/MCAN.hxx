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
#include "DummyGPIO.hxx"
#include "SPI.hxx"

#include "os/Gpio.hxx"
#include "os/OS.hxx"
#include "utils/Atomic.hxx"

#include "can_ioctl.h"

#define TCAN4550_DEBUG 0

/// Specification of CAN driver for the TCAN4550.
/// @todo The TCAN4550 uses the Bosch MCAN IP. If we end up supporting other
///       devices that also use this IP, then some of the generic MCAN related
///       content can be factored out into a common location.
class TCAN4550Can : public Can, public OSThread, private Atomic
{
public:
    /// Constructor.
    /// @param name name of this device instance in the file system
    /// @param interrupt_enable callback to enable the interrupt
    /// @param interrupt_disable callback to disable the interrupt
    /// @param test_pin test GPIO pin for instrumenting the code
    TCAN4550Can(const char *name,
                void (*interrupt_enable)(), void (*interrupt_disable)(),
#if TCAN4550_DEBUG
                const Gpio *test_pin = DummyPinWithRead::instance()
#else
                const Gpio *test_pin = nullptr
#endif
               )
        : Can(name, 0, 0)
        , OSThread()
        , interruptEnable_(interrupt_enable)
        , interruptDisable_(interrupt_disable)
        , spiFd_(-1)
        , spi_(nullptr)
        , sem_()
        , mcanInterruptEnable_()
        , txCompleteMask_(0)
        , state_(CAN_STATE_STOPPED)
        , txPending_(false)
        , rxPending_(false)
#if TCAN4550_DEBUG
        , testPin_(test_pin)
#endif
    {
#if TCAN4550_DEBUG
        testPin_->set();
#endif
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
    /// @param rx_timeout_bits timeout in CAN bit periods for rx interrupt
    void init(const char *spi_name, uint32_t freq, uint32_t baud,
              uint16_t rx_timeout_bits);

    /// Handle an interrupt.  Called by user provided interrupt handler.
    __attribute__((optimize("-O3")))
    void interrupt_handler()
    {
        int woken = false;
        interruptDisable_();
        sem_.post_from_isr(&woken);
        os_isr_exit_yield_test(woken);
    }

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

    /// size in words of the MRAM memory
    static constexpr size_t MRAM_SIZE_WORDS = (2 * 1024) / 4;

    // ---- Memory layout ----
    //
    // +-----------------------+
    // | RX FIFO 0, buf 0      | 0x0000
    // | ...                   |
    // | RX FIFO 0, buf 63     | 0x03F0
    // +-----------------------+
    // | TX Event 0 (FIFO)     | 0x0400
    // | ...                   |
    // | TX Event 15 (FIFO)    | 0x047F
    // +-----------------------+
    // | TX Buf 0              | 0x0480
    // | ...                   |
    // | TX Buf 15             | 0x057F
    // +-----------------------+
    // | TX Buf 16 (FIFO)      | 0x0580
    // | ...                   |
    // | TX Buf 31 (FIFO)      | 0x067F
    // +-----------------------+
    // | Unused                | 0x0680
    // +-----------------------+

    /// size in elements for the RX FIFO
    static constexpr uint32_t RX_FIFO_SIZE = 64;

    /// size in elements for the TX event FIFO
    static constexpr uint32_t TX_EVENT_FIFO_SIZE = 16;

    /// size in elements for the dedicated TX buffers
    static constexpr uint32_t TX_DEDICATED_BUFFER_COUNT = 16;

    /// size in elements for the TX FIFO
    static constexpr uint32_t TX_FIFO_SIZE = 16;

    /// mask of all the TX buffers used in the TX FIFO
    static constexpr uint32_t TX_FIFO_BUFFERS_MASK = 0xFFFF0000;

    /// start address of RX FIFO 0 in MRAM
    static constexpr uint16_t RX_FIFO_0_MRAM_ADDR = 0x0000;

    /// start address of TX Event FIFO in MRAM
    static constexpr uint16_t TX_EVENT_FIFO_MRAM_ADDR = 0x0400;

    /// start address of TX BUFFERS in MRAM
    static constexpr uint16_t TX_BUFFERS_MRAM_ADDR = 0x0480;

    /// start address of TX FIFO in MRAM
    static constexpr uint16_t TX_FIFO_BUFFERS_MRAM_ADDR = 0x0580;

    /// Offset of the MRAM address over SPI
    static constexpr uint16_t MRAM_ADDR_OFFSET = 0x8000;

    /// SPI Registers, word addressing, not byte addressing.
    /// This means that the values here need to be multiplied by 4 to get the
    /// actual address.
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
                      //   From here on the register offsets differ between
                      //   implementations:
                      //   TCAN STM32 < register offset
        GFC,          ///< 0x80 0x80 global filter configuration
        SIDFC,        ///< 0x84 ---- standard ID filter configuration
        XIDFC,        ///< 0x88 ---- extended ID filter configuration
        RSVD14,       ///< 0x8C ---- reserved
        XIDAM,        ///< 0x90 0x84 extended ID and mask
        HPMS,         ///< 0x94 0x88 high prioirty message status
        NDAT1,        ///< 0x98 ---- new data 1
        NDAT2,        ///< 0x9C ---- new data 2
        RXF0C,        ///< 0xA0 ---- RX FIFO 0 configuration
        RXF0S,        ///< 0xA4 0x90 RX FIFO 0 status
        RXF0A,        ///< 0xA8 0x94 RX FIFO 0 Acknowledge
        RXBC,         ///< 0xAC ---- RX buffer configuration
        RXF1C,        ///< 0xB0 ---- RX FIFO 1 configuration
        RXF1S,        ///< 0xB4 0x98 RX FIFO 1 status
        RXF1A,        ///< 0xB8 0x9C RX FIFO 1 acknowledge
        RXESC,        ///< 0xBC ---- RX buffer/FIFO element size configuration
        TXBC,         ///< 0xC0 0xC0 TX buffer configuration
        TXFQS,        ///< 0xC4 0xC4 TX FIFO/queue status
        TXESC,        ///< 0xC8 ---- TX buffer element size configuration
        TXBRP,        ///< 0xCC 0xC8 TX buffer request pending
        TXBAR,        ///< 0xD0 0xCC TX buffer add request
        TXBCR,        ///< 0xD4 0xD0 TX buffer cancellation request
        TXBTO,        ///< 0xD8 0xD4 TX buffer transmission occurred
        TXBCF,        ///< 0xDC 0xD8 TX buffer cancellation finished
        TXBTIE,       ///< 0xE0 0xDC TX buffer transmission interrupt enable
        TXBCIE,       ///< 0xE4 0xE0 TX buffer cancellation finished interrupt enable
        RSVD15,       ///< 0xE8 ---- reserved
        RSVD16,       ///< 0xEC ---- reserved
        TXEFC,        ///< 0xF0 ---- TX event FIFO configuration
        TXEFS,        ///< 0xF4 0xE4 TX event FIFO status
        TXEFA,        ///< 0xF8 0xE8 TX event FIFO acknowledge
        RSVD17,       ///< reserved

        MRAM = 0x2000, ///< MRAM offset
    };

    // Check alignment
    static_assert(TXEFS * 4 == 0x10F4, "register enum misaligned");
    static_assert(ILE * 4 == 0x105C, "register enum misaligned");
    static_assert(MCAN_INTERRUPT_STATUS * 4 == 0x0824,
                  "register enum misaligned");

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
    struct Dbtp
    {
        /// Constructor.
        /// @param dsjw data (re)synchronization jump width
        /// @param dtseg2 data time segment before sample point
        /// @param dtseg1 data time segment after sample point
        /// @param dbrp data bit rate prescaler
        /// @param tdc trasmitter delay compensation
        Dbtp(uint32_t dsjw, uint32_t dtseg2, uint32_t dtseg1, uint32_t dbrp,
             uint32_t tdc)
            : dsjw(dsjw)
            , dtseg2(dtseg2)
            , dtseg1(dtseg1)
            , dbrp(dbrp)
            , tdc(tdc)
        {
        }

        union
        {
            uint32_t data; ///< raw word value
            struct
            {
                uint32_t dsjw      : 4; ///< data (re)synchronization jump width
                uint32_t dtseg2    : 4; ///< data time segment before sample
                uint32_t dtseg1    : 5; ///< data time segment after sample
                uint32_t reserved1 : 3; ///< reserved
                uint32_t dbrp      : 5; ///< data bit rate prescaler
                uint32_t reserved2 : 2; ///< reserved
                uint32_t tdc       : 1; ///< trasmitter delay compensation
                uint32_t reserved3 : 8; ///< reserved
            };
        };
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

    /// Nominal bit timing & prescaler register definition
    struct Nbtp
    {
        /// Constructor.
        /// @param dsjw data (re)synchronization jump width
        /// @param dtseg2 data time segment before sample point
        /// @param dtseg1 data time segment after sample point
        /// @param dbrp data bit rate prescaler
        /// @param tdc trasmitter delay compensation
        Nbtp(uint32_t sjw, uint32_t tseg2, uint32_t tseg1, uint32_t brp)
            : ntseg2(tseg2)
            , ntseg1(tseg1)
            , nbrp(brp)
            , nsjw(sjw)
        {
        }

        union
        {
            uint32_t data; ///< raw word value
            struct
            {
                uint32_t ntseg2    : 7; ///< time segment before sample
                uint32_t rsvd1     : 1; ///< reserved
                uint32_t ntseg1    : 8; ///< time segment after sample
                uint32_t nbrp      : 9; ///< bit rate prescaler
                uint32_t nsjw      : 7; ///< re-synchronization jump width
            };
        };
    };

    /// Timestamp counter configuration register definition
    struct Tscc
    {
        /// Constructor. Sets the reset value.
        Tscc()
            : data(0x00000000)
        {
        }

        union
        {
            uint32_t data; ///< raw word value
            struct
            {
                uint32_t tss   :  2; ///< timestamp select
                uint32_t rsvd1 : 14; ///< reserved

                uint32_t tcp   :  4; ///< timestamp counter prescaler
                uint32_t rsvd2 : 12; ///< reserved
            };
        };
    };

    /// Timestamp counter value register definition
    struct Tscv
    {
        union
        {
            uint32_t data; ///< raw word value
            struct
            {
                uint32_t tsc  : 16; ///< timestamp counter
                uint32_t rsvd : 16; ///< reserved
            };
        };
    };

    /// Timeout counter configuration register definition
    struct Tocc
    {
        /// Constructor. Sets the reset value.
        Tocc()
            : data(0xFFFF0000)
        {
        }

        union
        {
            uint32_t data; ///< raw word value
            struct
            {
                uint32_t etoc  : 1; ///< enable timeout counter
                uint32_t tos   : 2; ///< timeout select
                uint32_t rsvd1 : 5; ///< reserved

                uint32_t rsvd2 : 8; ///< reserved

                uint32_t top : 16; ///< timeout period
            };
        };
    };

    /// Timeout counter value register definition
    struct Tocv
    {
        union
        {
            uint32_t data; ///< raw word value
            struct
            {
                uint32_t toc  : 16; ///< timeout counter
                uint32_t rsvd : 16; ///< reserved
            };
        };
    };

    /// Protocol status register definition
    struct Psr
    {
        union
        {
            uint32_t data; ///< raw word value
            struct
            {
                uint32_t lec : 3; ///< last error code
                uint32_t act : 2; ///< activity
                uint32_t ep  : 1; ///< error passive
                uint32_t ew  : 1; ///< warning status
                uint32_t bo  : 1; ///< bus-off status

                uint32_t dlec  : 3; ///< data phase last error code
                uint32_t resi  : 1; ///< ESI of last received CAN FD message
                uint32_t rbrs  : 1; ///< BRS of last received CAN FD message
                uint32_t rfdf  : 1; ///< received a CAN FD message
                uint32_t pxe   : 1; ///< protocol exception event
                uint32_t rsvd1 : 1; ///< reserved

                uint32_t tdcv  : 7; ///< transmitter delauy compenation value
                uint32_t rsvd2 : 1; ///< reserved

                uint32_t rsvd3 : 8; ///< reserved
            };
        };
    };

    /// RX FIFO x configuraation register definition
    struct Rxfxc
    {
        /// Constructor. Sets the reset value.
        Rxfxc()
            : data(0x00000000)
        {
        }

        union
        {
            uint32_t data; ///< raw word value
            struct
            {
                uint32_t fsa : 16; ///< RX FIFO start address

                uint32_t fs   : 7; ///< RX FIFO size
                uint32_t rsvd : 1; ///< reserved

                uint32_t fwm : 7; ///< RX FIFO high water mark
                uint32_t fom : 1; ///< RX FIFO operation mode
            };
        };
    };

    /// RX FIFO x status register definition
    struct Rxfxs
    {
        union
        {
            uint32_t data; ///< raw word value
            struct
            {
                uint32_t ffl   : 7; ///< RX FIFO fill level
                uint32_t rsvd1 : 1; ///< reserved

                uint32_t fgi   : 6; ///< RX FIFO get index
                uint32_t rsvd2 : 2; ///< reserved

                uint32_t fpi   : 6; ///< RX FIFO put index
                uint32_t rsvd3 : 2;

                uint32_t ff    : 1; ///< RX FIFO full
                uint32_t rfl   : 1; ///< RX FIFO message lost
                uint32_t rsvd4 : 6; ///< reserved
            };
        };
    };

    /// RX FIFO x acknowledge register definition
    struct Rxfxa
    {
        /// Constructor. Sets the reset value.
        Rxfxa()
            : data(0x00000000)
        {
        }

        union
        {
            uint32_t data; ///< raw word value
            struct
            {
                uint32_t fai  :  6; ///< RX FIFO acknowledge index
                uint32_t rsvd : 26; ///< reserved
            };
        };
    };

    /// TX Buffer configuraation register definition
    struct Txbc
    {
        /// Constructor. Sets the reset value.
        Txbc()
            : data(0x00000000)
        {
        }

        union
        {
            uint32_t data; ///< raw word value
            struct
            {
                uint32_t tbsa : 16; ///< TX buffers start address

                uint32_t ndtb  : 6; ///< number of dediated transmit buffers
                uint32_t rsvd1 : 2; ///< reserved

                uint32_t tfqs  : 6; ///< TX FIFO/queue size
                uint32_t tfqm  : 1; ///< TX FIFO/queue mode
                uint32_t rsvd2 : 1; ///< reserved
            };
        };
    };

    /// TX FIFO/queue status register definition
    struct Txfqs
    {
        union
        {
            uint32_t data; ///< raw word value
            struct
            {
                uint32_t tffl  : 6; ///< TX FIFO free level
                uint32_t rsvd1 : 2; ///< reserved

                uint32_t tfgi  : 5; ///< TX FIFO/queue get index
                uint32_t rsvd2 : 3; ///< reserved

                uint32_t tfqpi : 5; ///< TX FIFO/queue put index
                uint32_t tfqf  : 1; ///< TX FIFO/queue full
                uint32_t rsvd3 : 2; ///< reserved

                uint32_t rsvd4 : 8; ///< reserved
            };
        };
    };

    /// TX buffer element size configurataion register definition
    struct Txesc
    {
        /// Constructor. Sets the reset value.
        Txesc()
            : data(0x00000000)
        {
        }

        union
        {
            uint32_t data; ///< raw word value
            struct
            {
                uint32_t tbds :  3; ///< TX buffer data field size
                uint32_t rsvd : 29; ///< reserved
            };
        };
    };

    /// TX event FIFO configuration register definition
    struct Txefc
    {
        /// Constructor. Sets the reset value.
        Txefc()
            : data(0x00000000)
        {
        }

        union
        {
            uint32_t data; ///< raw word value
            struct
            {
                uint32_t efsa : 16; ///< event FIFO start address

                uint32_t efs   : 6; ///< event FIFO size
                uint32_t rsvd1 : 2; ///< reserved

                uint32_t efwm  : 6; ///< event FIFO watermark
                uint32_t rsvd2 : 2; ///< reserved
            };
        };
    };

    /// TX event FIFO status register definition
    struct Txefs
    {
        union
        {
            uint32_t data; ///< raw word value
            struct
            {
                uint32_t effl  : 6; ///< event FIFO fill level
                uint32_t rsvd1 : 2; ///< reserved

                uint32_t efgi : 5; ///< event FIFO get index
                uint32_t rsvd2 : 3; ///< reserved

                uint32_t efpi  : 5; ///< event FIFO put index
                uint32_t rsvd3 : 3; ///< reserved

                uint32_t eff   : 1; ///< event FIFO full
                uint32_t tefl  : 1; ///< TX event FIFO element lost
                uint32_t rsvd4 : 6; ///< reserved
            };
        };
    };

    /// TX event FIFO acknowledge register definition
    struct Txefa
    {
        /// Constructor. Sets the reset value.
        Txefa()
            : data(0x00000000)
        {
        }

        union
        {
            uint32_t data; ///< raw word value
            struct
            {
                uint32_t efai :  5; ///< TX event FIFO acknowledge index
                uint32_t rsvd : 27; ///< reserved
            };
        };
    };

    /// TCAN4550 interrupt registers (INTERRUPT_ENABLE/STATUS)
    struct Interrupt
    {
        /// Constructor. Sets the reset value.
        Interrupt()
            : data(0x00000000)
        {
        }

        union
        {
            uint32_t data; ///< raw word value
            struct
            {
                uint32_t vtwd      : 1; ///< global voltage, temp or wdto
                uint32_t mcanint   : 1; ///< M_CAN global interrupt
                uint32_t rsvd1     : 1; ///< reserved
                uint32_t spierr    : 1; ///< SPI error
                uint32_t rsvd2     : 1; ///< reserved
                uint32_t canerr    : 1; ///< CAN eror
                uint32_t wkrq      : 1; ///< wake request
                uint32_t globalerr : 1; ///< global error (any fault)

                uint32_t candom    : 1; ///< CAN stuck dominant
                uint32_t rsvd3     : 1; ///< reserved
                uint32_t canslnt   : 1; ///< CAN silent
                uint32_t rsvd4     : 2; ///< reserved
                uint32_t wkerr     : 1; ///< wake error
                uint32_t lwu       : 1; ///< local wake up
                uint32_t canint    : 1; ///< CAN bus wake up interrupt

                uint32_t eccerr    : 1; ///< uncorrectable ECC error detected
                uint32_t rsvd5     : 1; ///< reserved
                uint32_t wdto      : 1; ///< watchdog timeout
                uint32_t tsd       : 1; ///< thermal shutdown
                uint32_t pwron     : 1; ///< power on
                uint32_t uvio      : 1; ///< under voltage VIO
                uint32_t uvsup     : 1; ///< under voltage VSUP and UVCCOUT
                uint32_t sms       : 1; ///< sleep mode status

                uint32_t rsvd6     : 7; ///< reserved
                uint32_t canbusnom : 1; ///< CAN bus normal
            };
        };
    };

    /// MCAN interrupt registers (IR, IE, and ILS) definition
    struct MCANInterrupt
    {
        /// Constructor. Sets the reset value.
        MCANInterrupt()
            : data(0x00000000)
        {
        }

        union
        {
            uint32_t data; ///< raw word value
            struct
            {
                uint32_t rf0n : 1; ///< RX FIFO 0 new message
                uint32_t rf0w : 1; ///< RX FIFO 0 watermark reached
                uint32_t rf0f : 1; ///< RX FIFO 0 full
                uint32_t rf0l : 1; ///< RX FIFO 0 message lost
                uint32_t rf1n : 1; ///< RX FIFO 1 new message
                uint32_t rf1w : 1; ///< RX FIFO 1 watermark reached
                uint32_t rf1f : 1; ///< RX FIFO 1 full
                uint32_t rf1l : 1; ///< RX FIFO 1 message lost

                uint32_t hpm  : 1; ///< high priority message
                uint32_t tc   : 1; ///< transmission completed
                uint32_t tcf  : 1; ///< transmission cancellation finished
                uint32_t tfe  : 1; ///< TX FIFO empty
                uint32_t tefn : 1; ///< TX event FIFO new entry
                uint32_t tefw : 1; ///< TX event FIFO watermark reached
                uint32_t teff : 1; ///< TX event FIFO full
                uint32_t tefl : 1; ///< TX event FIFO event lost

                uint32_t tsw  : 1; ///< timestamp wraparound
                uint32_t mraf : 1; ///< message RAM access failure
                uint32_t too  : 1; ///< timeout occurred
                uint32_t drx  : 1; ///< message stored to dedicated RX buffer
                uint32_t bec  : 1; ///< bit error corrected
                uint32_t beu  : 1; ///< bit error uncorrected
                uint32_t elo  : 1; ///< error logging overflow
                uint32_t ep   : 1; ///< error passive

                uint32_t ew   : 1; ///< warning status
                uint32_t bo   : 1; ///< bus-off status
                uint32_t wdi  : 1; ///< watchdog
                uint32_t pea  : 1; ///< protocol error in arbitration phase
                uint32_t ped  : 1; ///< protocol error in data phase
                uint32_t ara  : 1; ///< access to reserved address
                uint32_t rsvd : 2; ///< reserved
            };
        };
    };

    /// MCAN interrupt line enable register definition
    struct Ile
    {
        /// Constructor. Sets the reset value.
        Ile()
            : data(0x00000000)
        {
        }

        union
        {
            uint32_t data; ///< raw word value
            struct
            {
                uint32_t eint0 :  1; ///< enable interrupt line 0
                uint32_t eint1 :  1; ///< enable interrupt line 1
                uint32_t rsvd  : 30; ///< reserved
            };
        };
    };

    /// Buad rate table entry
    struct TCAN4550Baud
    {
        uint32_t freq; ///< incoming frequency
        uint32_t baud; ///< target baud rate
        Nbtp     nbtp; ///< data bit timing and prescaler
    };

    /// SPI message for read/write commands
    struct SPIMessage
    {
        union
        {
            uint64_t payload64;    ///< raw payload as 64-bit value
            uint32_t payload32[2]; ///< raw paylaod as 32-bit array
            uint8_t payload[8];    ///< raw payload
            struct
            {
                uint8_t length; ///< length in words
                uint8_t addrL;  ///< register address LSB
                uint8_t addrH;  ///< register address MSB
                union
                {
                    uint8_t cmd;    ///< command
                    uint8_t status; ///< bits 0..7 of INTERRUPT_STATUS
                };
                uint32_t data;  ///< data word
            };
        };
    };

    /// MRAM SPI message for read/write commands
    struct MRAMSPIMessage
    {
        union
        {
            uint32_t payload32; ///< raw paylaod as 32-bit value
            uint8_t payload[4]; ///< raw payload
            struct
            {
                uint8_t length; ///< length in words
                uint8_t addrL;  ///< register address LSB
                uint8_t addrH;  ///< register address MSB
                union
                {
                    uint8_t cmd;    ///< command
                    uint8_t status; ///< bits 0..7 of INTERRUPT_STATUS
                };
            };
        };
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

        union
        {
            uint64_t data64; ///< data payload (64-bit)
            uint32_t data32[2]; ///< data payload (0 - 1 word)
            uint16_t data16[4]; ///< data payload (0 - 3 half word)
            uint8_t  data[8]; ///< data payload (0 - 8 byte)
        };
    };

    /// TX Buffer structure
    struct MRAMTXBuffer
    {
        uint32_t id  : 29; ///< CAN identifier
        uint32_t rtr :  1; ///< remote transmission request
        uint32_t xtd :  1; ///< extended identifier
        uint32_t esi :  1; ///< error state indicator

        uint32_t rsvd1 : 16; ///< reserved
        uint32_t dlc   :  4; ///< data length code
        uint32_t brs   :  1; ///< bit rate switch
        uint32_t fdf   :  1; ///< FD format
        uint32_t rsvd2 :  1; ///< reserved
        uint32_t efc   :  1; ///< event FIFO control
        uint32_t mm    :  8; ///< message marker

        union
        {
            uint64_t data64;  ///< data payload 64-bit
            uint32_t data32[2]; ///< data payload (0 - 1 word)
            uint16_t data16[4]; ///< data payload (0 - 3 half word)
        };
    };

    /// TX Event FIFO Element structure
    struct MRAMTXEventFIFOElement
    {
        uint32_t id  : 29; ///< CAN identifier
        uint32_t rtr :  1; ///< remote transmission request
        uint32_t xtd :  1; ///< extended identifier
        uint32_t esi :  1; ///< error state indicator

        uint32_t txts : 16; ///< transmit timestamp
        uint32_t dlc  :  4; ///< data length code
        uint32_t brs  :  1; ///< bit rate switch
        uint32_t fdf  :  1; ///< FD format
        uint32_t et   :  2; ///< event type
        uint32_t mm   :  8; ///< message marker
    };

    /// Structure for writing multiple TX buffers in one SPI transaction.
    struct MRAMTXBufferMultiWrite
    {
        static_assert(sizeof(MRAMSPIMessage) == sizeof(uint32_t),
                      "unexpected MRAMSPIMessage size");

        uint32_t padding; ///< padding for 8-byte alignment
        MRAMSPIMessage header; ///< message header
        MRAMTXBuffer txBuffers[TX_FIFO_SIZE]; ///< buffer payload
    };

    /// Called after disable.
    void flush_buffers() override;

    /// Read from a file or device.
    /// @param file file reference for this device
    /// @param buf location to place read data
    /// @param count number of bytes to read
    /// @return number of bytes read upon success, -1 upon failure with errno
    ///         containing the cause
    ssize_t read(File *file, void *buf, size_t count) override;

    /// Write to a file or device.
    /// @param file file reference for this device
    /// @param buf location to find write data
    /// @param count number of bytes to write
    /// @return number of bytes written upon success, -1 upon failure with errno
    ///         containing the cause
    ssize_t write(File *file, const void *buf, size_t count) override;

    /// Request an ioctl transaction.
    /// @param file file reference for this device
    /// @param key ioctl key
    /// @param data key data
    /// @return >= 0 upon success, -errno upon failure
    int ioctl(File *file, unsigned long int key, unsigned long data) override;

    /// Device select method. Default impementation returns true.
    /// @param file reference to the file
    /// @param mode FREAD for read active, FWRITE for write active, 0 for
    ///        exceptions
    /// @return true if active, false if inactive
    bool select(File* file, int mode) override;

    /// User entry point for the created thread.
    /// @return exit status
    void *entry() override;

    void enable() override; ///< function to enable device
    void disable() override; ///< function to disable device

    /// Function to try and transmit a message.
    void tx_msg() override
    {
        // unused in this implementation
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

#if TCAN4550_DEBUG
        HASSERT((msg.status & 0x8) == 0);
#endif

        return msg.data;
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
        msg.data = data;

        spi_ioc_transfer xfer;
        xfer.tx_buf = (unsigned long)(&msg);
        xfer.rx_buf = (unsigned long)(&msg);
        xfer.len = sizeof(msg);

        spi_->transfer_with_cs_assert_polled(&xfer);
#if TCAN4550_DEBUG
        HASSERT((msg.status & 0x8) == 0);
#endif
    }

    /// Read one or more RX buffers.
    /// @param offset word offset in the MRAM to read from
    /// @param buf location to read into
    /// @param count number of buffers to read
    __attribute__((optimize("-O3")))
    void rxbuf_read(uint16_t offset, MRAMRXBuffer *buf, size_t count)
    {
        uint16_t address = offset + MRAM_ADDR_OFFSET;
        SPIMessage msg;
        msg.cmd = READ;
        msg.addrH = address >> 8;
        msg.addrL = address & 0xFF;
        msg.length = count * (sizeof(MRAMRXBuffer) / 4);

        spi_ioc_transfer xfer[2];
        xfer[0].tx_buf = (unsigned long)(&msg);
        xfer[0].rx_buf = (unsigned long)(&msg);
        xfer[0].len = 4; //sizeof(SPIMessage);
        xfer[1].tx_buf = (unsigned long)(nullptr);
        xfer[1].rx_buf = (unsigned long)(buf);
        xfer[1].len = count * sizeof(MRAMRXBuffer);

        spi_->transfer_with_cs_assert_polled(xfer, 2);
#if TCAN4550_DEBUG
        HASSERT((msg.status & 0x8) == 0);
#endif
    }

    /// Write one or more TX buffers.
    /// @param offset word offset in the MRAM to write to
    /// @param buf location to write from
    /// @param count number of buffers to write
    __attribute__((optimize("-O3")))
    void txbuf_write(uint16_t offset, MRAMTXBufferMultiWrite *buf, size_t count)
    {
        static_assert(sizeof(MRAMTXBuffer) == 16,
                      "Unexpected MRAMTXBuffer size");

        uint16_t address = offset + MRAM_ADDR_OFFSET;
        buf->header.cmd = WRITE;
        buf->header.addrH = address >> 8;
        buf->header.addrL = address & 0xFF;
        buf->header.length = count * (sizeof(MRAMTXBuffer) / 4);

        spi_ioc_transfer xfer;
        xfer.tx_buf = (unsigned long)&buf->header;
        xfer.rx_buf = (unsigned long)&buf->header;
        xfer.len = sizeof(buf->header) + (count * sizeof(MRAMTXBuffer));

        spi_->transfer_with_cs_assert_polled(&xfer);
#if TCAN4550_DEBUG
        HASSERT((buf->header.status & 0x8) == 0);
#endif
    }

    void (*interruptEnable_)(); ///< enable interrupt callback
    void (*interruptDisable_)(); ///< disable interrupt callback
    int spiFd_; ///< SPI bus that accesses TCAN4550
    SPI *spi_; ///< pointer to a SPI object instance
    OSSem sem_; ///< semaphore for posting events
    MCANInterrupt mcanInterruptEnable_; ///< shadow for the interrupt enable
    uint32_t txCompleteMask_; ///< shadow for the transmit complete buffer mask
    uint8_t state_; ///< present bus state

    // These bitmasks are protected from a read-modify-write race condition
    // because they are only set from a thread context that holds the SPI bus
    // mutex.
    uint8_t txPending_ : 1; ///< waiting on a TX active event
    uint8_t rxPending_ : 1; ///< waiting on a RX active event

    /// Allocating this buffer here avoids having to put it on the
    /// TCAN4550Can::write() caller's stack.
    MRAMTXBufferMultiWrite txBufferMultiWrite_ __attribute__((aligned(8)));
#if TCAN4550_DEBUG
    volatile uint32_t regs_[64]; ///< debug copy of TCAN4550 registers
    volatile uint32_t status_;
    volatile uint32_t enable_;
    volatile uint32_t spiStatus_;
    const Gpio *testPin_; ///< test GPIO pin for instrumenting the code
#endif

    /// baud rate settings table
    static const TCAN4550Baud BAUD_TABLE[];

    /// Default Constructor.
    TCAN4550Can();

    DISALLOW_COPY_AND_ASSIGN(TCAN4550Can);
};

#endif // _FREERTOS_DRIVERS_COMMON_TCAN4550CAN_HXX_


/** @copyright
 * Copyright (c) 2025 Balazs Racz
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
 * @file Stm32MCan.hxx
 *
 * This file implements the CAN driver for the FDCAN controller in newer Stm32
 * microcontrollers, which is based on the Bosch MCAN IP.
 *
 * @author Balazs Racz
 * @date 19 Jan 2025
 */

#ifndef _FREERTOS_DRIVERS_ST_STM32MCAN_HXX_
#define _FREERTOS_DRIVERS_ST_STM32MCAN_HXX_

#include "stm32f_hal_conf.hxx"
#include <cstddef>

#include "freertos_drivers/common/MCAN.hxx"

/// Register addresses for the STM32 FDCAN controller. These addresses are word
/// indexes, i.e., to access in the memory map, they must be multipled by 4.
///
/// These values are referenced to FDCAN1_BASE or FDCAN2_BASE.
enum class Stm32FDCANRegisters : uint16_t
{
    CREL = 0, ///< core release
    ENDN,     ///< endianess
    RSVDx1,   ///< reserved
    DBTP,     ///< data bit timing and prescaler
    TEST2,    ///< test
    RWD,      ///< RAM watchdog
    CCCR,     ///< CC control
    NBTP,     ///< nominal bit timing and prescaler
    TSCC,     ///< timestamp counter configuration
    TSCV,     ///< timestamp counter value
    TOCC,     ///< timeout counter configuration
    TOCV,     ///< timeout counter value
    RSVD1,    ///< reserved
    RSVD2,    ///< reserved
    RSVD3,    ///< reserved
    RSVD4,    ///< reserved
    ECR,      ///< error count
    PSR,      ///< protocol status
    TDCR,     ///< transmitter delay compensation
    RSVD5,    ///< reserved
    IR,       ///< interrupt status
    IE,       ///< interrupt enable
    ILS,      ///< interrupt line select
    ILE,      ///< interrupt line enable
    RSVD6,    ///< reserved
    RSVD7,    ///< reserved
    RSVD8,    ///< reserved
    RSVD9,    ///< reserved
    RSVD10,   ///< reserved
    RSVD11,   ///< reserved
    RSVD12,   ///< reserved
    RSVD13,   ///< reserved
              //   From here on the register offsets differ between
              //   implementations:
              //   TCAN STM32 < register offset
    GFC,      ///< 0x80 0x80 global filter configuration
    XIDAM,    ///< 0x90 0x84 extended ID and mask
    HPMS,     ///< 0x94 0x88 high prioirty message status
    RSVDx2,   ///< 0xA0 ---- RX FIFO 0 configuration
    RXF0S,    ///< 0xA4 0x90 RX FIFO 0 status
    RXF0A,    ///< 0xA8 0x94 RX FIFO 0 Acknowledge
    RXF1S,    ///< 0xB4 0x98 RX FIFO 1 status
    RXF1A,    ///< 0xB8 0x9C RX FIFO 1 acknowledge
    RSVDx3,
    RSVDx4,
    RSVDx5,
    RSVDx6,
    RSVDx7,
    RSVDx8,
    RSVDx9,
    RSVDx10,
    TXBC,   ///< 0xC0 0xC0 TX buffer configuration
    TXFQS,  ///< 0xC4 0xC4 TX FIFO/queue status
    TXBRP,  ///< 0xCC 0xC8 TX buffer request pending
    TXBAR,  ///< 0xD0 0xCC TX buffer add request
    TXBCR,  ///< 0xD4 0xD0 TX buffer cancellation request
    TXBTO,  ///< 0xD8 0xD4 TX buffer transmission occurred
    TXBCF,  ///< 0xDC 0xD8 TX buffer cancellation finished
    TXBTIE, ///< 0xE0 0xDC TX buffer transmission interrupt enable
    TXBCIE, ///< 0xE4 0xE0 TX buffer cancellation finished interrupt enable
    TXEFS,  ///< 0xF4 0xE4 TX event FIFO status
    TXEFA,  ///< 0xF8 0xE8 TX event FIFO acknowledge
};

static_assert(uint16_t(Stm32FDCANRegisters::CREL) * 4 ==
        offsetof(FDCAN_GlobalTypeDef, CREL),
    "Registers misaligned.");
static_assert(uint16_t(Stm32FDCANRegisters::NBTP) * 4 ==
        offsetof(FDCAN_GlobalTypeDef, NBTP),
    "Registers misaligned.");
static_assert(uint16_t(Stm32FDCANRegisters::ILE) * 4 ==
        offsetof(FDCAN_GlobalTypeDef, ILE),
    "Registers misaligned.");
static_assert(uint16_t(Stm32FDCANRegisters::HPMS) * 4 ==
        offsetof(FDCAN_GlobalTypeDef, HPMS),
    "Registers misaligned.");
static_assert(uint16_t(Stm32FDCANRegisters::RXF1S) * 4 ==
        offsetof(FDCAN_GlobalTypeDef, RXF1S),
    "Registers misaligned.");
static_assert(uint16_t(Stm32FDCANRegisters::TXBC) * 4 ==
        offsetof(FDCAN_GlobalTypeDef, TXBC),
    "Registers misaligned.");
static_assert(uint16_t(Stm32FDCANRegisters::TXEFA) * 4 ==
        offsetof(FDCAN_GlobalTypeDef, TXEFA),
    "Registers misaligned.");

struct Stm32FDCANDefs
{
    typedef MCANCommonDefs<64> Common;
    typedef Common::MRAMRXBuffer MRAMRXBuffer;
    typedef Common::MRAMTXBuffer MRAMTXBuffer;
    typedef Common::MRAMTXEventFIFOElement MRAMTXEventFIFOElement;

    typedef Stm32FDCANRegisters Registers;

    void init_fdcan(FDCAN_GlobalTypeDef *instance)
    {
        base_ = reinterpret_cast<uint32_t *>(instance);
        if (instance == FDCAN1)
        {
            mram_ = reinterpret_cast<uint32_t *>(SRAMCAN_BASE);
        }
        else if (instance == FDCAN2)
        {
            mram_ = reinterpret_cast<uint32_t *>(SRAMCAN_BASE + 0x350);
        }
    }

    /// size in elements for the RX FIFO
    static constexpr uint32_t RX_FIFO_SIZE = 3;

    /// size in elements for the TX event FIFO
    static constexpr uint32_t TX_EVENT_FIFO_SIZE = 3;

    /// size in elements for the dedicated TX buffers
    static constexpr uint32_t TX_DEDICATED_BUFFER_COUNT = 0;

    /// size in elements for the TX FIFO
    static constexpr uint32_t TX_FIFO_SIZE = 3;

    /// mask of all the TX buffers used in the TX FIFO
    static constexpr uint32_t TX_FIFO_BUFFERS_MASK = 0b111;

    /// start address of RX FIFO 0 in MRAM
    static constexpr uint16_t RX_FIFO_0_MRAM_ADDR = 0x00B0;

    /// start address of TX Event FIFO in MRAM
    static constexpr uint16_t TX_EVENT_FIFO_MRAM_ADDR = 0x0260;

    /// start address of TX BUFFERS in MRAM
    static constexpr uint16_t TX_BUFFERS_MRAM_ADDR = 0x0278;

    /// start address of TX FIFO in MRAM
    static constexpr uint16_t TX_FIFO_BUFFERS_MRAM_ADDR = 0x0278;

    /// Offset of the MRAM address over SPI
    // static constexpr uint16_t MRAM_ADDR_OFFSET = 0x8000;

    /// TX Buffer configuration register definition
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
                uint32_t resvd : 24; ///< reserved

                uint32_t tfqm : 1; ///< TX FIFO/queue mode
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
                uint32_t rf0f : 1; ///< RX FIFO 0 full
                uint32_t rf0l : 1; ///< RX FIFO 0 message lost
                uint32_t rf1n : 1; ///< RX FIFO 1 new message
                uint32_t rf1f : 1; ///< RX FIFO 1 full
                uint32_t rf1l : 1; ///< RX FIFO 1 message lost
                uint32_t hpm : 1;  ///< high priority message
                uint32_t tc : 1;   ///< transmission completed

                uint32_t tcf : 1;  ///< transmission cancellation finished
                uint32_t tfe : 1;  ///< TX FIFO empty
                uint32_t tefn : 1; ///< TX event FIFO new entry
                uint32_t teff : 1; ///< TX event FIFO full
                uint32_t tefl : 1; ///< TX event FIFO event lost
                uint32_t tsw : 1;  ///< timestamp wraparound
                uint32_t mraf : 1; ///< message RAM access failure
                uint32_t too : 1;  ///< timeout occurred

                uint32_t elo : 1; ///< error logging overflow
                uint32_t ep : 1;  ///< error passive
                uint32_t ew : 1;  ///< warning status
                uint32_t bo : 1;  ///< bus-off status
                uint32_t wdi : 1; ///< watchdog
                uint32_t pea : 1; ///< protocol error in arbitration phase
                uint32_t ped : 1; ///< protocol error in data phase
                uint32_t ara : 1; ///< access to reserved address
            };
        };
    };

protected:
    struct MRAMTXBufferMultiWrite
    {
        MRAMTXBuffer txBuffers[TX_FIFO_SIZE]; ///< buffer payload
    } txBufferMultiWrite_;

    /// Read from an FDCAN register.
    /// @param address address to read from
    /// @return data read
    __attribute__((optimize("-O3"))) uint32_t register_read(Registers address)
    {
        return base_[uint16_t(address)];
    }

    /// Write to an FDCAN register.
    /// @param address address to write to
    /// @param data data to write
    __attribute__((optimize("-O3"))) void register_write(
        Registers address, uint32_t data)
    {
        base_[uint16_t(address)] = data;
    }

    /// Read one or more RX buffers.
    /// @param offset word offset in the MRAM to read from
    /// @param buf location to read into
    /// @param count number of buffers to read
    __attribute__((optimize("-O3"))) void rxbuf_read(
        uint16_t offset, struct can_frame *buf, size_t count)
    {
        // This prints the size of the struct. Use it when the below assertion
        // fails.
        // char (*__errorprint)[sizeof(MRAMRXBuffer)] = 1;

        // The magic constant here comes from the hal_fdcan.c file in the STM32
        // middleware. It is not exported in a header.
        static_assert(
            sizeof(MRAMRXBuffer) == 18u * 4u, "RX buffer size mismatch");
        for (unsigned i = 0; i < count; ++i)
        {
            unsigned mram_idx = ((offset + i * sizeof(MRAMRXBuffer)) >> 2);
            unsigned len = sizeof(struct can_frame) >> 2;
            uint32_t *dst = reinterpret_cast<uint32_t *>(&buf[i]);
            while (len > 0)
            {
                *dst++ = mram_[mram_idx++];
                --len;
            }
            Common::rx_buf_to_struct_can_frame(&buf[i]);
        }
    }

    /// Write one or more TX buffers.
    /// @param offset word offset in the MRAM to write to
    /// @param buf location to write from
    /// @param count number of buffers to write
    __attribute__((optimize("-O3"))) void txbuf_write(
        uint16_t offset, MRAMTXBufferMultiWrite *buf, size_t count)
    {
        // char (*__errorprint)[sizeof(MRAMTXBuffer)] = 1;
        static_assert(
            sizeof(MRAMTXBuffer) == (18U * 4U), "TX Buffer size mismatch");
        const uint32_t *src = reinterpret_cast<uint32_t *>(buf);
        unsigned len = (count * sizeof(MRAMTXBuffer)) >> 2;
        offset >>= 2;
        while (len > 0)
        {
            mram_[offset++] = *src++;
            --len;
        }
        // memcpy(mram_ + (offset >> 2), buf, count * sizeof(MRAMTXBuffer));
    }

    /// Unused hook.
    void clear_global_interrupt_flags()
    {
    }

    void preinit_hook(uint32_t freq)
    {
    }

    void init_hook()
    {
        {
            Txbc txbc;
            txbc.tfqm = 0;
            register_write(Registers::TXBC, txbc.data);
        }
    }

private:
    FDCAN_GlobalTypeDef *inst()
    {
        return reinterpret_cast<FDCAN_GlobalTypeDef *>(
            const_cast<uint32_t *>(base_));
    }

    volatile uint32_t *base_ {nullptr};
    uint32_t *mram_ {nullptr};
};

class Stm32MCan : public MCAN<Stm32FDCANDefs>
{
public:
    /// Constructor.
    ///
    /// @param name device name, e.g. "/dev/can0"
    /// @param inst instance pointer for memory mapped registers, FDCAN1 or
    /// FDCAN2
    /// @param interrupt interrupt number for the module.
    Stm32MCan(const char *name, FDCAN_GlobalTypeDef *inst, IRQn_Type interrupt)
        : MCAN(name, &Stm32MCan::interrupt_enable,
              &Stm32MCan::interrupt_disable, this)
        , interrupt_(interrupt)
    {
        MCAN::init_fdcan(inst);
    }

    void interrupt_handler()
    {
        MCAN::interrupt_handler();
    }

private:
    IRQn_Type interrupt_;

    static void interrupt_disable(void *inst)
    {
        auto intn = static_cast<Stm32MCan *>(inst)->interrupt_;
        HAL_NVIC_DisableIRQ(intn);
    }
    static void interrupt_enable(void *inst)
    {
        auto intn = static_cast<Stm32MCan *>(inst)->interrupt_;
        HAL_NVIC_EnableIRQ(intn);
    }
};

#endif // _FREERTOS_DRIVERS_ST_STM32MCAN_HXX_

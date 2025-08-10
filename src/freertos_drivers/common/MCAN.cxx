/** @copyright
 * Copyright (c) 2020-2025 Stuart W Baker, Balazs Racz
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
 * @file MCAN.cxx
 * This file implements the CAN driver for the MCAN CAN Controller.
 *
 * @author Stuart W. Baker
 * @date 26 February 2020
 */

#ifndef _DEFAULT_SOURCE
#define _DEFAULT_SOURCE
#endif

#include "MCAN.hxx"

#include <fcntl.h>
#include <unistd.h>

#include <atomic>

template <class Defs, typename Registers>
const typename MCAN<Defs, Registers>::MCANBaud
    MCAN<Defs, Registers>::BAUD_TABLE[] = {
        /* 20 MHz clock source
         * TQ = BRP / freq = 10 / 20 MHz = 500 nsec
         * Baud = 125 kHz
         * bit time = 1 / 125 kHz = 8 usec = 16 TQ
         * SyncSeg = 1 TQ
         * PropSeg = 7 TQ
         * Seg1 = 4 TQ
         * Seg2 = 4 TQ
         * sample time = (1 TQ + 7 TQ + 4 TQ) / 16 TQ = 75%
         * SJW = Seg - 1 = 4 - 1 = 3
         * SJW = 3 * 500 nsec = 1.5 usec
         *
         * Oscillator Tolerance:
         *     4 / (2 * ((13 * 16) - 4)) = 0.980%
         *     3 / (20 * 16) = 0.938%
         *     = 0.938%
         */
        {20000000, 125000, {(3 - 1), (4 - 1), (11 - 1), (10 - 1)}},
        /* 40 MHz clock source
         * TQ = BRP / freq = 20 / 40 MHz = 500 nsec
         * Baud = 125 kHz
         * bit time = 1 / 125 kHz = 8 usec = 16 TQ
         * SyncSeg = 1 TQ
         * PropSeg = 7 TQ
         * Seg1 = 4 TQ
         * Seg2 = 4 TQ
         * sample time = (1 TQ + 7 TQ + 4 TQ) / 16 TQ = 75%
         * SJW = Seg - 1 = 4 - 1 = 3
         * SJW = 3 * 500 nsec = 1.5 usec
         *
         * Oscillator Tolerance:
         *     4 / (2 * ((13 * 16) - 4)) = 0.980%
         *     3 / (20 * 16) = 0.938%
         *     = 0.938%
         */
        {40000000, 125000, {(3 - 1), (4 - 1), (11 - 1), (20 - 1)}},
        /* 64 MHz clock source
         * TQ = BRP / freq = 32 / 64 MHz = 500 nsec
         * Baud = 125 kHz
         * bit time = 1 / 125 kHz = 8 usec = 16 TQ
         * SyncSeg = 1 TQ
         * PropSeg = 7 TQ
         * Seg1 = 4 TQ
         * Seg2 = 4 TQ
         * sample time = (1 TQ + 7 TQ + 4 TQ) / 16 TQ = 75%
         * SJW = Seg - 1 = 4 - 1 = 3
         * SJW = 3 * 500 nsec = 1.5 usec
         *
         * Oscillator Tolerance:
         *     4 / (2 * ((13 * 16) - 4)) = 0.980%
         *     3 / (20 * 16) = 0.938%
         *     = 0.938%
         */
        {64000000, 125000, {(3 - 1), (4 - 1), (11 - 1), (32 - 1)}},
        /* 4 MHz clock source
         * TQ = BRP / freq = 2 / 4 MHz = 500 nsec
         * Baud = 125 kHz
         * bit time = 1 / 125 kHz = 8 usec = 16 TQ
         * SyncSeg = 1 TQ
         * PropSeg = 7 TQ
         * Seg1 = 4 TQ
         * Seg2 = 4 TQ
         * sample time = (1 TQ + 7 TQ + 4 TQ) / 16 TQ = 75%
         * SJW = Seg - 1 = 4 - 1 = 3
         * SJW = 3 * 500 nsec = 1.5 usec
         *
         * Oscillator Tolerance:
         *     4 / (2 * ((13 * 16) - 4)) = 0.980%
         *     3 / (20 * 16) = 0.938%
         *     = 0.938%
         */
        {4000000, 125000, {(3 - 1), (4 - 1), (11 - 1), (2 - 1)}},
        /* 8 MHz clock source
         * TQ = BRP / freq = 4 / 8 MHz = 500 nsec
         * Baud = 125 kHz
         * bit time = 1 / 125 kHz = 8 usec = 16 TQ
         * SyncSeg = 1 TQ
         * PropSeg = 7 TQ
         * Seg1 = 4 TQ
         * Seg2 = 4 TQ
         * sample time = (1 TQ + 7 TQ + 4 TQ) / 16 TQ = 75%
         * SJW = Seg - 1 = 4 - 1 = 3
         * SJW = 3 * 500 nsec = 1.5 usec
         *
         * Oscillator Tolerance:
         *     4 / (2 * ((13 * 16) - 4)) = 0.980%
         *     3 / (20 * 16) = 0.938%
         *     = 0.938%
         */
        {8000000, 125000, {(3 - 1), (4 - 1), (11 - 1), (4 - 1)}},
};

void TCAN4550Defs::init_spi(const char *spi_name, uint32_t freq)
{
    spiFd_ = ::open(spi_name, O_RDWR);
    HASSERT(spiFd_ >= 0);

    ::ioctl(spiFd_, SPI_IOC_GET_OBJECT_REFERENCE, &spi_);
    HASSERT(spi_);

    // configure SPI bus settings
    uint8_t spi_mode = SPI_MODE_0;
    uint8_t spi_bpw = 32;
    uint32_t spi_max_speed_hz = freq / 2;
    if (spi_max_speed_hz > SPI_MAX_SPEED_HZ)
    {
        spi_max_speed_hz = SPI_MAX_SPEED_HZ;
    }
    ::ioctl(spiFd_, SPI_IOC_WR_MODE, &spi_mode);
    ::ioctl(spiFd_, SPI_IOC_WR_BITS_PER_WORD, &spi_bpw);
    ::ioctl(spiFd_, SPI_IOC_WR_MAX_SPEED_HZ, &spi_max_speed_hz);
}

//
// init()
//
template <class Defs, typename Registers>
void MCAN<Defs, Registers>::init(
    uint32_t freq, uint32_t baud, uint16_t rx_timeout_bits)
{
    // lock SPI bus access
    OSMutexLock locker(&lock_);

    // read/clear TCAN status flags
    Defs::clear_global_interrupt_flags();
    Defs::preinit_hook(freq);
    {
        // enter configuration mode
        Cccr cccr; // default is initialization mode
        Defs::register_write(Registers::CCCR, cccr.data);
        do
        {
            cccr.data = Defs::register_read(Registers::CCCR);
        } while (cccr.init == 0);

        cccr.cce = 1; // configuration change enable
        Defs::register_write(Registers::CCCR, cccr.data);
        do
        {
            cccr.data = Defs::register_read(Registers::CCCR);
        } while (cccr.cce == 0);
    }

    Defs::init_hook();

    {
        // setup timestamp counter
        // make sure that the timeout is reasonable
        HASSERT(rx_timeout_bits >= 10);

        Tscc tscc;    // default TCP = (1 - 1) = 0
        tscc.tss = 1; // value incremented according to TCP
        Defs::register_write(Registers::TSCC, tscc.data);

        Tocc tocc;
        tocc.etoc = 1;                    // enable timeout counter
        tocc.tos = 2;                     // timeout controlled by RX FIFO 0
        tocc.top = (rx_timeout_bits - 1); // timeout counts in bit periods
        Defs::register_write(Registers::TOCC, tocc.data);
    }

    // Setup timing
    for ( size_t i = 0; i < ARRAYSIZE(BAUD_TABLE); ++i)
    {
        if (BAUD_TABLE[i].freq == freq && BAUD_TABLE[i].baud == baud)
        {
            Defs::register_write(Registers::NBTP, BAUD_TABLE[i].nbtp.data);

            Cccr cccr;
            do
            {
                cccr.data = 0;
                Defs::register_write(Registers::CCCR, cccr.data);
                cccr.data = Defs::register_read(Registers::CCCR);
            } while (cccr.init == 1);

            return;
        }
    }

    // unsupported frequency
    HASSERT(0);
}

//
// enable()
//
template <class Defs, typename Registers> void MCAN<Defs, Registers>::enable()
{
    // There is a mutex lock of lock_ above us, so the following sequence is
    // thread safe.
    if (!is_created())
    {
        // start the thread at the highest priority in the system
        start(name, get_priority_max(), 512);

        /// @todo The stack size of 512 bytes was chosen based on the debugger
        ///       reporting a high watter mark of 232 bytes used. The test
        ///       platform was a CC3220 with GCC compiler (ARMv7m). It is
        ///       likely that this high water mark will vary based on CPU
        ///       architecture, and this stack size should probably be
        ///       paramatizable in the future.
    }

    txCompleteMask_ = 0;
    txPending_ = false;
    rxPending_ = true; // waiting on an RX message
    {
        // clear MCAN interrupts
        MCANInterrupt mcan_interrupt;
        mcan_interrupt.data = 0x3FFFFFFF;
        Defs::register_write(Registers::IR, mcan_interrupt.data);
    }
    {
        // enable MCAN interrupts
        mcanInterruptEnable_.data = 0; // start with all interrupts disabled
        mcanInterruptEnable_.rf0l = 1; // RX FIFO 0 message lost
        mcanInterruptEnable_.tc = 1;   // transmission complete
        mcanInterruptEnable_.too = 1;  // timeout occured (RX timeout)
        mcanInterruptEnable_.ep = 1;   // error passive
        mcanInterruptEnable_.bo = 1;   // bus-off status
        Defs::register_write(Registers::IE, mcanInterruptEnable_.data);
    }
    {
        // enable interrupt line 0
        Ile ile;
        ile.eint0 = 1;
        Defs::register_write(Registers::ILE, ile.data);
    }
    {
        // enable normal operation mode
        Cccr cccr;
        cccr.init = 0; // normal operation mode, enables CAN bus access
        Defs::register_write(Registers::CCCR, cccr.data);
    }

    state_ = CAN_STATE_ACTIVE;

    interruptEnable_(interruptArg_);
}

//
// disable()
//
template <class Defs, typename Registers> void MCAN<Defs, Registers>::disable()
{
    // There is a mutex lock of lock_ above us, so the following sequence is
    // thread safe.
    interruptDisable_(interruptArg_);

    state_ = CAN_STATE_STOPPED;

    {
        // enable initalization mode
        Cccr cccr;
        cccr.init = 1; // initialization mode, disables CAN bus access
        Defs::register_write(Registers::CCCR, cccr.data);
    }
    {
        // disable interrupt line 0
        Ile ile;
        Defs::register_write(Registers::ILE, ile.data);
    }
    {
        // disable MCAN interrupts
        mcanInterruptEnable_.data = 0;
        Defs::register_write(Registers::IE, mcanInterruptEnable_.data);
    }

    flush_buffers();
}

//
// MCAN::flush_buffers()
//
template <class Defs, typename Registers>
void MCAN<Defs, Registers>::flush_buffers()
{
    // lock SPI bus access
    OSMutexLock locker(&lock_);

    // cancel TX FIFO buffers
    Defs::register_write(Registers::TXBCR, Defs::TX_FIFO_BUFFERS_MASK);

    // get the rx status (FIFO fill level)
    Rxfxs rxf0s;
    rxf0s.data = Defs::register_read(Registers::RXF0S);

    /// @todo this could be made more efficeint, but does it matter?
    while (rxf0s.ffl)
    {
        // acknowledge the next FIFO index
        Rxfxa rxf0a;
        rxf0a.fai = rxf0s.fgi;
        Defs::register_write(Registers::RXF0A, rxf0a.data);

        // increment index and count
        if (++rxf0s.fgi >= Defs::RX_FIFO_SIZE)
        {
            rxf0s.fgi = 0;
        }
        --rxf0s.ffl;
    }
}

//
// MCAN::read()
//
template <class Defs, typename Registers>
ssize_t MCAN<Defs, Registers>::read(File *file, void *buf, size_t count)
{
    HASSERT((count % sizeof(struct can_frame)) == 0);

    struct can_frame *data = (struct can_frame*)buf;
    ssize_t result = 0;

    count /= sizeof(struct can_frame);

    while (count)
    {
        size_t frames_read = 0;

        {
            // lock SPI bus access
            OSMutexLock locker(&lock_);
            Rxfxs rxf0s;
            rxf0s.data = Defs::register_read(Registers::RXF0S);
            if (rxf0s.ffl)
            {

                // clip to the continous buffer memory available
                frames_read =
                    std::min(Defs::RX_FIFO_SIZE - rxf0s.fgi, rxf0s.ffl);

                // clip to the number of asked for frames
                frames_read = std::min(frames_read, count);

                // read from MRAM
                Defs::rxbuf_read(Defs::RX_FIFO_0_MRAM_ADDR +
                        (rxf0s.fgi * sizeof(MRAMRXBuffer)),
                    data, frames_read);

                // acknowledge the last FIFO index read
                Rxfxa rxf0a;
                rxf0a.fai = rxf0s.fgi + (frames_read - 1);
                Defs::register_write(Registers::RXF0A, rxf0a.data);
            }
            if (frames_read == rxf0s.ffl)
            {
                // all of the data was pulled out, need to re-enable RX
                // timeout interrupt

                // set pending flag
                rxPending_ = true;

                // enable receive timeout interrupt
                mcanInterruptEnable_.too = 1;
                Defs::register_write(Registers::IE, mcanInterruptEnable_.data);
            }
        }
        for (size_t i = 0; i < frames_read; ++i)
        {
        }

        if (frames_read == 0)
        {
            // no more data to receive
            if ((file->flags & O_NONBLOCK) || result > 0)
            {
                break;
            }
            else
            {
                // wait for data to come in
                rxBuf->block_until_condition(file, true);
            }
        }

        count -= frames_read;
        result += frames_read;
        data += frames_read;
    }

    if (!result && (file->flags & O_NONBLOCK))
    {
        return -EAGAIN;
    }

    return result * sizeof(struct can_frame);
}

//
// MCAN::write()
//
template <class Defs, typename Registers>
ssize_t MCAN<Defs, Registers>::write(File *file, const void *buf, size_t count)
{
    HASSERT((count % sizeof(struct can_frame)) == 0);

    const struct can_frame *data = (const struct can_frame*)buf;
    ssize_t result = 0;

    count /= sizeof(struct can_frame);

    while (count)
    {
        size_t frames_written = 0;

        {
            // lock SPI bus access
            OSMutexLock locker(&lock_);

            if (state_ != CAN_STATE_ACTIVE)
            {
                // cancel pending TX FIFO buffers to make room
                Defs::register_write(
                    Registers::TXBCR, Defs::TX_FIFO_BUFFERS_MASK);

                /// @todo It is possible that the tramsmit FIFO writes which
                ///       follow will be stuck in the FIFO until we pass
                ///       through this code again (could be another time
                ///       time through the loop or a future call to
                ///       MCAN::write()). This could be a long time,
                ///       resulting in stale data going out on the bus once the
                ///       error state is removed. A possible future enhancement
                ///       would be to use the MCAN timeout counter to flush the
                ///       FIFO again when it expires (suggested 3 second
                ///       timeout).
            }

            Txfqs txfqs;
            txfqs.data = Defs::register_read(Registers::TXFQS);
            if (txfqs.tffl)
            {
                // clip to the continous buffer memory available
                frames_written = std::min(Defs::TX_FIFO_SIZE -
                        (txfqs.tfqpi - Defs::TX_DEDICATED_BUFFER_COUNT),
                    txfqs.tffl);

                // clip to the number of provided frames
                frames_written = std::min(frames_written, count);

                uint32_t txbar = 0;
                uint32_t put_index = txfqs.tfqpi;
                MRAMTXBuffer *tx_buffers = Defs::txBufferMultiWrite_.txBuffers;
                // shuffle data for structure translation
                for (size_t i = 0; i < frames_written; ++i, ++put_index)
                {
                    if (data[i].can_eff)
                    {
                        // extended frame
                        tx_buffers[i].id = data[i].can_id;
                    }
                    else
                    {
                        // standard frame
                        tx_buffers[i].id = data[i].can_id << 18;
                    }
                    tx_buffers[i].rtr = data[i].can_rtr;
                    tx_buffers[i].xtd = data[i].can_eff;
                    tx_buffers[i].esi = data[i].can_err;
                    tx_buffers[i].dlc = data[i].can_dlc;
                    tx_buffers[i].brs = 0;
                    tx_buffers[i].fdf = 0;
                    tx_buffers[i].efc = 0;
                    tx_buffers[i].mm = 0;
                    tx_buffers[i].data64[0] = data[i].data64;
                    txbar |= 0x1 << put_index;
                }

                // write to MRAM
                Defs::txbuf_write(Defs::TX_BUFFERS_MRAM_ADDR +
                        (txfqs.tfqpi * sizeof(MRAMTXBuffer)),
                    &this->txBufferMultiWrite_, frames_written);

                // add transmission requests
                Defs::register_write(Registers::TXBAR, txbar);
            }
            if (frames_written == txfqs.tffl)
            {
                // No space left in the TX FIFO. Setup a software emulation for
                // a transmit watermark half way through the FIFO

                // set pending flag
                txPending_ = true;

                uint32_t watermark_index =
                    txfqs.tfgi + (Defs::TX_FIFO_SIZE / 2);
                if (watermark_index >=
                    (Defs::TX_FIFO_SIZE + Defs::TX_DEDICATED_BUFFER_COUNT))
                {
                    watermark_index -= Defs::TX_FIFO_SIZE;
                }
                txCompleteMask_ |= 0x1 << watermark_index;

                // enable TX FIFO buffer interrupt
                Defs::register_write(Registers::TXBTIE, txCompleteMask_);
            }
        }

        if (frames_written == 0)
        {
            // no more data to transmit
            if ((file->flags & O_NONBLOCK) || result > 0)
            {
                break;
            }
            else
            {
                // wait for space to be available
                txBuf->block_until_condition(file, false);
            }
        }
        else
        {
            result += frames_written;
            count -= frames_written;
            data += frames_written;
        }
    }

    if (!result && (file->flags & O_NONBLOCK))
    {
        return -EAGAIN;
    }

    return result * sizeof(struct can_frame);
}

//
// TCQN4550Can::select()
//
template <class Defs, typename Registers>
bool MCAN<Defs, Registers>::select(File *file, int mode)
{
    bool retval = false;
    switch (mode)
    {
        case FREAD:
        {
            AtomicHolder h(this);
            if (rxPending_)
            {
                rxBuf->select_insert();
            }
            else
            {
                retval = true;
            }
            break;
        }
        case FWRITE:
        {
            AtomicHolder h (this);
            if (txPending_)
            {
                txBuf->select_insert();
            }
            else
            {
                retval = true;
            }
            break;
        }
        default:
            return Can::select(file, mode);
    }

    return retval;
}

//
// MCAN::ioctl()
//
template <class Defs, typename Registers>
int MCAN<Defs, Registers>::ioctl(
    File *file, unsigned long int key, unsigned long data)
{
    if (key == SIOCGCANSTATE)
    {
        *((can_state_t*)data) = state_;
        return 0;
    }
    return -EINVAL;
}

//
// entry()
//
template <class Defs, typename Registers>
__attribute__((optimize("-O3"))) void *MCAN<Defs, Registers>::entry()
{
    for ( ; /* forever */ ; )
    {
#if MCAN_DEBUG
        int result = sem_.timedwait(SEC_TO_NSEC(1));

        if (result != 0)
        {
            OSMutexLock locker(&lock_);
            enable_ = Defs::register_read(INTERRUPT_ENABLE);
            spiStatus_ = Defs::register_read(STATUS);
            status_ = Defs::register_read(INTERRUPT_STATUS);

            MRAMSPIMessage msg;
            msg.cmd = READ;
            msg.addrH = 0x10;
            msg.addrL = 0x00;
            msg.length = 64;

            spi_ioc_transfer xfer[2];
            xfer[0].tx_buf = (unsigned long)&msg;
            xfer[0].rx_buf = 0;
            xfer[0].len = sizeof(MRAMSPIMessage);
            xfer[1].tx_buf = 0;
            xfer[1].rx_buf = (unsigned long)regs_;
            xfer[1].len = sizeof(regs_);

            spi_->transfer_with_cs_assert_polled(xfer, 2);
            continue;
        }
#else
        sem_.wait();
#endif

        // lock SPI bus access
        OSMutexLock locker(&lock_);

        /// @todo The following sequence could be made more efficient by
        ///       making a single 8-byte read transfer with the shadowed
        ///       version of the IR register.

        // read status flags
        MCANInterrupt mcan_interrupt;
        mcan_interrupt.data = Defs::register_read(Registers::IR);

        // clear status flags for enabled interrupts
        Defs::register_write(
            Registers::IR, mcan_interrupt.data & mcanInterruptEnable_.data);

        Defs::clear_global_interrupt_flags();

        // error handling
        if (mcan_interrupt.bo || mcan_interrupt.ep || mcan_interrupt.rf0l)
        {
            // read protocol status
            Psr psr;
            psr.data = Defs::register_read(Registers::PSR);

            if (mcan_interrupt.rf0l)
            {
                // receive overrun
                ++overrunCount;
            }
            if (mcan_interrupt.ep)
            {
                if (psr.ep)
                {
                    // error passive state
#if MCAN_DEBUG
                    testPin_->clr();
#endif
                    ++softErrorCount;
                    state_ = CAN_STATE_BUS_PASSIVE;
                }
                else
                {
#if MCAN_DEBUG
                    testPin_->set();
#endif
                    state_ = CAN_STATE_ACTIVE;
                }
            }
            if (mcan_interrupt.bo)
            {
                if (psr.bo)
                {
                    // bus off
#if MCAN_DEBUG
                    testPin_->write(!testPin_->read());
#endif
                    ++busOffCount;
                    state_ = CAN_STATE_BUS_OFF;
                    // attempt recovery
                    Cccr cccr;
                    do
                    {
                        cccr.data = 0;
                        Defs::register_write(Registers::CCCR, cccr.data);
                        cccr.data = Defs::register_read(Registers::CCCR);
                    } while (cccr.init == 1);

                    // cancel TX FIFO buffers
                    Defs::register_write(
                        Registers::TXBCR, Defs::TX_FIFO_BUFFERS_MASK);

                    txBuf->signal_condition();
#if MCAN_DEBUG
                    testPin_->write(!testPin_->read());
#endif
                }
                else
                {
#if MCAN_DEBUG
                    testPin_->set();
#endif
                    state_ = CAN_STATE_ACTIVE;
                }
            }
        }

        // transmission complete
        if (mcan_interrupt.tc && txCompleteMask_)
        {
            {
                AtomicHolder h(this);
                // clear pending flag
                txPending_ = false;

                // signal anyone waiting
                txBuf->signal_condition();
            }

            // disable TX buffer tranmission complete interrupt
            txCompleteMask_ = 0;
            Defs::register_write(Registers::TXBTIE, txCompleteMask_);
        }

        // received timeout
        if (mcan_interrupt.too && mcanInterruptEnable_.too)
        {
            {
                AtomicHolder h(this);
                // clear pending flag
                rxPending_ = false;

                // signal anyone waiting
                rxBuf->signal_condition();
            }

            // disable timeout interrupt
            mcanInterruptEnable_.too = 0;
            Defs::register_write(Registers::IE, mcanInterruptEnable_.data);
        }

        interruptEnable_(interruptArg_);
    }

    return NULL;
}

// Explicitly instantiates the MCAN implementation for the TCAN4550.
/// @todo move this to TCAN4550.cxx.
template class MCAN<>;

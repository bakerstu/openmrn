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
 * @file TCAN4550Can.cxx
 * This file implements the CAN driver for the TCAN4550 CAN Controller.
 *
 * @author Stuart W. Baker
 * @date 26 February 2020
 */

#ifndef _DEFAULT_SOURCE
#define _DEFAULT_SOURCE
#endif

#include "TCAN4550Can.hxx"

#include <fcntl.h>
#include <unistd.h>

#include <atomic>

const TCAN4550Can::TCAN4550Baud TCAN4550Can::BAUD_TABLE[] =
{
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
     * TQ = BRP / freq = 10 / 40 MHz = 500 nsec
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
};


//
// init()
//
void TCAN4550Can::init(const char *spi_name, uint32_t freq, uint32_t baud,
                       uint16_t rx_timeout_bits)
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

    // lock SPI bus access
    OSMutexLock locker(&lock_);

    {
        // read/clear TCAN status flags
        uint32_t status = register_read(INTERRUPT_STATUS);
        register_write(INTERRUPT_STATUS, status);
    }
    {
        // transition to "Normal" mode with sleep and watchdog disabled
        Mode mode;
        mode.sweDis = 1;   // disable sleep
        mode.wdEnable = 0; // disable watchdog
        mode.modeSel = 2;  // normal mode
        mode.clkRef = (freq == 40000000);

        register_write(MODE, mode.data);
    }
    {
        // enter configuration mode
        Cccr cccr; // default is initialization mode
        register_write(CCCR, cccr.data);
        do
        {
            cccr.data = register_read(CCCR);
        } while (cccr.init == 0);

        cccr.cce = 1; // configuration change enable
        register_write(CCCR, cccr.data);
        do
        {
            cccr.data = register_read(CCCR);
        } while (cccr.cce == 0);
    }

    // diasable all TCAN interrupts
    register_write(INTERRUPT_ENABLE, 0);

    // clear MRAM, a bit brute force, but gets the job done
    for (uint16_t offset = 0x0000; offset < MRAM_SIZE_WORDS; ++offset)
    {
        register_write((Registers)(MRAM + offset), 0);
    }

    {
        // setup RX FIFO 0
        Rxfxc rxf0c;
        rxf0c.fsa = RX_FIFO_0_MRAM_ADDR; // FIFO start address
        rxf0c.fs = RX_FIFO_SIZE;         // FIFO size
        register_write(RXF0C, rxf0c.data);
    }
    {
        // setup TX configuration
        Txbc txbc;
        txbc.tbsa = TX_BUFFERS_MRAM_ADDR;      // buffers start address
        txbc.ndtb = TX_DEDICATED_BUFFER_COUNT; // dedicated transmit buffers
        txbc.tfqs = TX_FIFO_SIZE;              // FIFO/queue size
        register_write(TXBC, txbc.data);
    }
    {
        // setup TX buffer element size
        Txesc txesc;
        txesc.tbds = 0; // 8 byte data field size
        register_write(TXESC, txesc.data);
    }
    {
        // setup TX event FIFO
        Txefc txefc;
        txefc.efsa = TX_EVENT_FIFO_MRAM_ADDR; // event FIFO start address
        txefc.efs = TX_EVENT_FIFO_SIZE;       // event FIFO size
        txefc.efwm = TX_EVENT_FIFO_SIZE / 2;  // event FIFO watermark
        register_write(TXEFC, txefc.data);
    }
    {
        // setup timestamp counter
        // make sure that the timeout is reasonable
        HASSERT(rx_timeout_bits >= 10);

        Tscc tscc;    // default TCP = (1 - 1) = 0
        tscc.tss = 1; // value incremented according to TCP
        register_write(TSCC, tscc.data);

        Tocc tocc;
        tocc.etoc = 1;                    // enable timeout counter
        tocc.tos = 2;                     // timeout controlled by RX FIFO 0
        tocc.top = (rx_timeout_bits - 1); // timeout counts in bit periods
        register_write(TOCC, tocc.data);
    }

    // Setup timing
    for ( size_t i = 0; i < ARRAYSIZE(BAUD_TABLE); ++i)
    {
        if (BAUD_TABLE[i].freq == freq && BAUD_TABLE[i].baud == baud)
        {
            register_write(NBTP, BAUD_TABLE[i].nbtp.data);

            Cccr cccr;
            do
            {
                cccr.data = 0;
                register_write(CCCR, cccr.data);
                cccr.data = register_read(CCCR);
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
void TCAN4550Can::enable()
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
        register_write(IR, mcan_interrupt.data);
    }
    {
        // enable MCAN interrupts
        mcanInterruptEnable_.data = 0; // start with all interrupts disabled
        mcanInterruptEnable_.rf0l = 1; // RX FIFO 0 message lost
        mcanInterruptEnable_.tc = 1;   // transmission complete
        mcanInterruptEnable_.too = 1;  // timeout occured (RX timeout)
        mcanInterruptEnable_.ep = 1;   // error passive
        mcanInterruptEnable_.bo = 1;   // bus-off status
        register_write(IE, mcanInterruptEnable_.data);
    }
    {
        // enable interrupt line 0
        Ile ile;
        ile.eint0 = 1;
        register_write(ILE, ile.data);
    }
    {
        // enable normal operation mode
        Cccr cccr;
        cccr.init = 0; // normal operation mode, enables CAN bus access
        register_write(CCCR, cccr.data);
    }

    state_ = CAN_STATE_ACTIVE;

    interruptEnable_();
}

//
// disable()
//
void TCAN4550Can::disable()
{
    // There is a mutex lock of lock_ above us, so the following sequence is
    // thread safe.
    interruptDisable_();

    state_ = CAN_STATE_STOPPED;

    {
        // enable initalization mode
        Cccr cccr;
        cccr.init = 1; // initialization mode, disables CAN bus access
        register_write(CCCR, cccr.data);
    }
    {
        // disable interrupt line 0
        Ile ile;
        register_write(ILE, ile.data);
    }
    {
        // disable MCAN interrupts
        mcanInterruptEnable_.data = 0;
        register_write(IE, mcanInterruptEnable_.data);
    }

    flush_buffers();
}

//
// TCAN4550Can::flush_buffers()
//
void TCAN4550Can::flush_buffers()
{
    // lock SPI bus access
    OSMutexLock locker(&lock_);

    // cancel TX FIFO buffers
    register_write(TXBCR, TX_FIFO_BUFFERS_MASK);

    // get the rx status (FIFO fill level)
    Rxfxs rxf0s;
    rxf0s.data = register_read(RXF0S);

    /// @todo this could be made more efficeint, but does it matter?
    while (rxf0s.ffl)
    {
        // acknowledge the next FIFO index
        Rxfxa rxf0a;
        rxf0a.fai = rxf0s.fgi;
        register_write(RXF0A, rxf0a.data);

        // increment index and count
        if (++rxf0s.fgi >= RX_FIFO_SIZE)
        {
            rxf0s.fgi = 0;
        }
        --rxf0s.ffl;
    }
}

//
// TCAN4550Can::read()
//
ssize_t TCAN4550Can::read(File *file, void *buf, size_t count)
{
    HASSERT((count % sizeof(struct can_frame)) == 0);

    struct can_frame *data = (struct can_frame*)buf;
    ssize_t result = 0;

    count /= sizeof(struct can_frame);

    while (count)
    {
        size_t frames_read = 0;
        MRAMRXBuffer *mram_rx_buffer = reinterpret_cast<MRAMRXBuffer*>(data);

        {
            // lock SPI bus access
            OSMutexLock locker(&lock_);
            Rxfxs rxf0s;
            rxf0s.data = register_read(RXF0S);
            if (rxf0s.ffl)
            {
                static_assert(sizeof(struct can_frame) == sizeof(MRAMRXBuffer), "RX buffer size does not match assumptions.");

                // clip to the continous buffer memory available
                frames_read = std::min(RX_FIFO_SIZE - rxf0s.fgi, rxf0s.ffl);

                // clip to the number of asked for frames
                frames_read = std::min(frames_read, count);

                // read from MRAM
                rxbuf_read(
                    RX_FIFO_0_MRAM_ADDR + (rxf0s.fgi * sizeof(MRAMRXBuffer)),
                    mram_rx_buffer, frames_read);

                // acknowledge the last FIFO index read
                Rxfxa rxf0a;
                rxf0a.fai = rxf0s.fgi + (frames_read - 1);
                register_write(RXF0A, rxf0a.data);
            }
            if (frames_read == rxf0s.ffl)
            {
                // all of the data was pulled out, need to re-enable RX
                // timeout interrupt

                // set pending flag
                rxPending_ = true;

                // enable receive timeout interrupt
                mcanInterruptEnable_.too = 1;
                register_write(IE, mcanInterruptEnable_.data);
            }
        }
        // shuffle data for structure translation
        for (size_t i = 0; i < frames_read; ++i)
        {
            struct can_frame tmp;

            tmp.can_id = mram_rx_buffer[i].id;
            if (!mram_rx_buffer[i].xtd)
            {
                // standard frame
                tmp.can_id >>= 18;
            }
            tmp.can_dlc = mram_rx_buffer[i].dlc;
            tmp.can_rtr = mram_rx_buffer[i].rtr;
            tmp.can_eff = mram_rx_buffer[i].xtd;
            tmp.can_err = mram_rx_buffer[i].esi;

            std::atomic_thread_fence(std::memory_order_seq_cst);

            data[i].raw[0] = tmp.raw[0];
            data[i].raw[1] = tmp.raw[1];
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
// TCAN4550Can::write()
//
ssize_t TCAN4550Can::write(File *file, const void *buf, size_t count)
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
                register_write(TXBCR, TX_FIFO_BUFFERS_MASK);

                /// @todo It is possible that the tramsmit FIFO writes which
                ///       follow will be stuck in the FIFO until we pass
                ///       through this code again (could be another time
                ///       time through the loop or a future call to
                ///       TCAN4550Can::write()). This could be a long time,
                ///       resulting in stale data going out on the bus once the
                ///       error state is removed. A possible future enhancement
                ///       would be to use the MCAN timeout counter to flush the
                ///       FIFO again when it expires (suggested 3 second
                ///       timeout).
            }

            Txfqs txfqs;
            txfqs.data = register_read(TXFQS);
            if (txfqs.tffl)
            {
                // clip to the continous buffer memory available
                frames_written = std::min(
                    TX_FIFO_SIZE - (txfqs.tfqpi - TX_DEDICATED_BUFFER_COUNT),
                    txfqs.tffl);

                // clip to the number of provided frames
                frames_written = std::min(frames_written, count);

                uint32_t txbar = 0;
                uint32_t put_index = txfqs.tfqpi;
                MRAMTXBuffer *tx_buffers = txBufferMultiWrite_.txBuffers;
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
                    tx_buffers[i].data64 = data[i].data64;
                    txbar |= 0x1 << put_index;
                }

                // write to MRAM
                txbuf_write(
                    TX_BUFFERS_MRAM_ADDR + (txfqs.tfqpi * sizeof(MRAMTXBuffer)),
                    &txBufferMultiWrite_, frames_written);

                // add transmission requests
                register_write(TXBAR, txbar);
            }
            if (frames_written == txfqs.tffl)
            {
                // No space left in the TX FIFO. Setup a software emulation for
                // a transmit watermark half way through the FIFO

                // set pending flag
                txPending_ = true;

                uint32_t watermark_index = txfqs.tfgi + (TX_FIFO_SIZE / 2);
                if (watermark_index >=
                    (TX_FIFO_SIZE + TX_DEDICATED_BUFFER_COUNT))
                {
                    watermark_index -= TX_FIFO_SIZE;
                }
                txCompleteMask_ |= 0x1 << watermark_index;

                // enable TX FIFO buffer interrupt
                register_write(TXBTIE, txCompleteMask_);
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
bool TCAN4550Can::select(File* file, int mode)
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
// TCAN4550Can::ioctl()
//
int TCAN4550Can::ioctl(File *file, unsigned long int key, unsigned long data)
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
__attribute__((optimize("-O3")))
void *TCAN4550Can::entry()
{
    for ( ; /* forever */ ; )
    {
#if TCAN4550_DEBUG
        int result = sem_.timedwait(SEC_TO_NSEC(1));

        if (result != 0)
        {
            OSMutexLock locker(&lock_);
            enable_ = register_read(INTERRUPT_ENABLE);
            spiStatus_ = register_read(STATUS);
            status_ = register_read(INTERRUPT_STATUS);

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
        mcan_interrupt.data = register_read(IR);

        // clear status flags for enabled interrupts
        register_write(IR, mcan_interrupt.data & mcanInterruptEnable_.data);

        // read TCAN status flags
        uint32_t status = register_read(INTERRUPT_STATUS);

        // clear TCAN status flags
        register_write(INTERRUPT_STATUS, status);
#if TCAN4550_DEBUG
        status_ = status;
#endif
        // error handling
        if (mcan_interrupt.bo || mcan_interrupt.ep || mcan_interrupt.rf0l)
        {
            // read protocol status
            Psr psr;
            psr.data = register_read(PSR);

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
#if TCAN4550_DEBUG
                    testPin_->clr();
#endif
                    ++softErrorCount;
                    state_ = CAN_STATE_BUS_PASSIVE;
                }
                else
                {
#if TCAN4550_DEBUG
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
#if TCAN4550_DEBUG
                    testPin_->write(!testPin_->read());
#endif
                    ++busOffCount;
                    state_ = CAN_STATE_BUS_OFF;
                    // attempt recovery
                    Cccr cccr;
                    do
                    {
                        cccr.data = 0;
                        register_write(CCCR, cccr.data);
                        cccr.data = register_read(CCCR);
                    } while (cccr.init == 1);

                    // cancel TX FIFO buffers
                    register_write(TXBCR, TX_FIFO_BUFFERS_MASK);

                    txBuf->signal_condition();
#if TCAN4550_DEBUG
                    testPin_->write(!testPin_->read());
#endif
                }
                else
                {
#if TCAN4550_DEBUG
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
            register_write(TXBTIE, txCompleteMask_);
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
            register_write(IE, mcanInterruptEnable_.data);
        }

        interruptEnable_();
    }

    return NULL;
}

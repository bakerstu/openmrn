/** \copyright
 * Copyright (c) 2014, Balazs Racz
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
 * \file TivaRailcom.hxx
 *
 * Device driver for TivaWare to read one or more UART inputs for railcom data.
 *
 * usage:
 *
 * struct MyRailcomPins {
 *   // add all definitions from the example RailcomHw here
 * };
 * // Some definitions need to go outside of the class.
 * // no need for attribute weak if this is in a .cxx file.
 * const uint32_t MyRailcomPins::UART_BASE[] __attribute__((weak)) = {...};
 * ...
 *
 * TivaRailcomDriver<MyrailcomPins> railcom_driver("/dev/railcom");
 *
 * // assuming you put OS_INTERRRUPT = INT_UART3 into the structure.
 * void uart3_interrupt_handler() {
 *    railcom_driver.os_interrrupt();
 * }
 *
 * Then open /dev/railcom and read dcc::Feedback structures from it. Each read
 * much be exactly sizeof(dcc::Feedback) length. If there are no more packets to
 * read, you'll get return=0, errno==EAGAIN. Add an ioctl CAN_READ_ACTIVE to
 * get a notification when there is something to read.
 *
 * @author Balazs Racz
 * @date 6 Jan 2015
 */

#ifndef _FREERTOS_DRIVERS_TI_TIVARAILCOM_HXX_
#define _FREERTOS_DRIVERS_TI_TIVARAILCOM_HXX_

#if (!defined(TIVADCC_TIVA)) && (!defined(TIVADCC_CC3200))
#error must define either TIVADCC_TIVA or TIVADCC_CC3200
#endif

#include "TivaDCC.hxx"  // for FixedQueue

#include "RailcomDriver.hxx"
#include "dcc/RailCom.hxx"

/*
struct RailcomHw
{
    static const uint32_t CHANNEL_COUNT = 4;
    static const uint32_t UART_PERIPH[CHANNEL_COUNT];
    static const uint32_t UART_BASE[CHANNEL_COUNT];
    // Make sure there are enough entries here for all the channels times a few
    // DCC packets.
    static const uint32_t Q_SIZE = 16;

    static const auto OS_INTERRUPT = INT_UART2;

    GPIO_HWPIN(CH1, GpioHwPin, C, 4, U4RX);
    GPIO_HWPIN(CH2, GpioHwPin, C, 6, U3RX);
    GPIO_HWPIN(CH3, GpioHwPin, G, 4, U2RX);
    GPIO_HWPIN(CH4, GpioHwPin, E, 0, U7RX);

    static void hw_init() {
         CH1_Pin::hw_init();
         CH2_Pin::hw_init();
         CH3_Pin::hw_init();
         CH4_Pin::hw_init();
    }

    static void set_input() {
        CH1_Pin::set_input();
        CH2_Pin::set_input();
        CH3_Pin::set_input();
        CH4_Pin::set_input();
    }

    static void set_hw() {
        CH1_Pin::set_hw();
        CH2_Pin::set_hw();
        CH3_Pin::set_hw();
        CH4_Pin::set_hw();
    }

    /// @returns a bitmask telling which pins are active. Bit 0 will be set if
    /// channel 0 is active (drawing current).
    static uint8_t sample() {
        uint8_t ret = 0;
        if (!CH1_Pin::get()) ret |= 1;
        if (!CH2_Pin::get()) ret |= 2;
        if (!CH3_Pin::get()) ret |= 4;
        if (!CH4_Pin::get()) ret |= 8;
        return ret;
    }
}; 

// The weak attribute is needed if the definition is put into a header file.
const uint32_t RailcomHw::UART_BASE[] __attribute__((weak)) = {UART4_BASE, UART3_BASE, UART2_BASE, UART7_BASE};
const uint32_t RailcomHw::UART_PERIPH[]
__attribute__((weak)) = {SYSCTL_PERIPH_UART4, SYSCTL_PERIPH_UART3, SYSCTL_PERIPH_UART2, SYSCTL_PERIPH_UART7};

*/

/// Base class for railcom drivers. Ideally this base class should be
/// non-specific to the hardwsre or the  TivaWare driver library.
///
/// @todo(balazs.racz) factor this class out into
/// freertos_drivers/common/Railcom.hxx.
template <class HW>
class RailcomDriverBase : public RailcomDriver, private Node
{
public:
    /// Constructor. @param name is the device node (e.g. "/dev/railcom0").
    RailcomDriverBase(const char *name)
        : Node(name)
        , readableNotifiable_(nullptr)
    {
        HW::hw_init();
        MAP_IntPrioritySet(HW::OS_INTERRUPT, configKERNEL_INTERRUPT_PRIORITY);
        MAP_IntEnable(HW::OS_INTERRUPT);
    }

    ~RailcomDriverBase()
    {
        MAP_IntDisable(HW::OS_INTERRUPT);
    }

private:
    ssize_t write(File *, const void *, size_t) OVERRIDE
    {
        // This device is read-only.
        return -ENOSYS;
    }

    ssize_t read(File *file, void *buf, size_t count) OVERRIDE
    {
        if (count != sizeof(dcc::Feedback))
        {
            return -EINVAL;
        }
        OSMutexLock l(&lock_);
        if (feedbackQueue_.empty())
        {
            return -EAGAIN;
        }
        memcpy(buf, &feedbackQueue_.front(), count);
        feedbackQueue_.increment_front();
        return count;
    }

    void flush_buffers() OVERRIDE
    {
        while (!feedbackQueue_.empty())
        {
            feedbackQueue_.increment_front();
        }
    }

    // Asynchronous interface
    int ioctl(File *file, unsigned long int key, unsigned long data) override
    {
        if (IOC_TYPE(key) == CAN_IOC_MAGIC &&
            IOC_SIZE(key) == NOTIFIABLE_TYPE && key == CAN_IOC_READ_ACTIVE)
        {
            Notifiable *n = reinterpret_cast<Notifiable *>(data);
            HASSERT(n);
            // If there is no data for reading, we put the incoming notification
            // into the holder. Otherwise we notify it immediately.
            if (feedbackQueue_.empty())
            {
                portENTER_CRITICAL();
                if (feedbackQueue_.empty())
                {
                    // We are in a critical section now. If we got into this
                    // branch, then the buffer was full at the beginning of the
                    // critical section. If the hardware interrupt kicks in
                    // now, and sets the os_interrupt to pending, the os
                    // interrupt will not happen until we leave the critical
                    // section, and thus the swap will be in effect by then.
                    std::swap(n, readableNotifiable_);
                }
                portEXIT_CRITICAL();
            }
            if (n)
            {
                n->notify();
            }
            return 0;
        }
        errno = EINVAL;
        return -1;
    }

public:
    /// Implementation of the interrupt handler running at the kernel interrupt
    /// priority. Call this function from the hardware interrupt handler.
    void os_interrupt_handler() __attribute__((always_inline))
    {
        if (!feedbackQueue_.empty())
        {
            Notifiable *n = readableNotifiable_;
            readableNotifiable_ = nullptr;
            if (n)
            {
                n->notify_from_isr();
                os_isr_exit_yield_test(true);
            }
        }
    }

private:
    void set_feedback_key(uint32_t key) OVERRIDE
    {
        feedbackKey_ = key;
    }

protected:
    /** Takes a new empty packet at the front of the queue, fills in feedback
     * key and channel information.
     * @param channel is which channel to set the packet for.
     * @return a packet pointer to write into. If there is no space, returns
     * nullptr.*/
    dcc::Feedback *alloc_new_packet(uint8_t channel)
    {
        if (!feedbackQueue_.has_noncommit_space())
        {
            return nullptr;
        }
        dcc::Feedback *entry = &feedbackQueue_.back();
        feedbackQueue_.noncommit_back();
        entry->reset(feedbackKey_);
        entry->channel = channel;
        return entry;
    }

    /// Adds a sample for a preamble bit. @param sample is the next sample to
    /// return to the application layer (usually a bitmask setting which
    /// channels are active).
    void add_sample(int sample) {
        if (sample < 0) return; // skipped sampling
        if (feedbackQueue_.full()) {
            extern uint32_t feedback_sample_overflow_count;
            ++feedback_sample_overflow_count;
            return;
        }
        feedbackQueue_.back().reset(feedbackKey_);
        feedbackQueue_.back().channel = HW::get_feedback_channel();
        feedbackQueue_.back().add_ch1_data(sample);
        uint32_t tick_timer = HW::get_timer_tick();
        memcpy(feedbackQueue_.back().ch2Data, &tick_timer, 4);
        feedbackQueue_.increment_back();
        MAP_IntPendSet(HW::OS_INTERRUPT);
    }

    /** Notify this when we have data in our buffers. */
    Notifiable *readableNotifiable_;
    /** The packets we have read so far. */
    FixedQueue<dcc::Feedback, HW::Q_SIZE> feedbackQueue_;
    /// Stores the key for the next packets to read.
    uint32_t feedbackKey_;
    /** Stores pointers to packets we are filling right now, one for each
     * channel. */
    dcc::Feedback *returnedPackets_[HW::CHANNEL_COUNT];
};

/// Railcom driver for TI Tiva-class microcontrollers using the TivaWare
/// peripheral library.
///
/// This railcom driver supports parallel polling of multiple UART channels for
/// the railcom data. 
template <class HW> class TivaRailcomDriver : public RailcomDriverBase<HW>
{
public:
    /// Constructor. @param path is the device node path (e.g. "/dev/railcom0").
    TivaRailcomDriver(const char *path) : RailcomDriverBase<HW>(path)
    {
    }

private:
    /// True when we are currently within a cutout.
    bool inCutout_ = false;

    using RailcomDriverBase<HW>::returnedPackets_;
    // File node interface
    void enable() OVERRIDE
    {
        for (unsigned i = 0; i < ARRAYSIZE(HW::UART_BASE); ++i)
        {
#ifdef TIVADCC_TIVA
            MAP_SysCtlPeripheralEnable(HW::UART_PERIPH[i]);
#elif defined(TIVADCC_CC3200)
            MAP_PRCMPeripheralClkEnable(HW::UART_PERIPH[i], PRCM_RUN_MODE_CLK);
#endif            
            MAP_UARTConfigSetExpClk(HW::UART_BASE[i], cm3_cpu_clock_hz, 250000,
                                    UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                                    UART_CONFIG_PAR_NONE);
            MAP_UARTFIFOEnable(HW::UART_BASE[i]);
            // Disables the receiver.
            HWREGBITW(HW::UART_BASE[i] + UART_O_CTL, UART_CTL_RXE) = 0;
            MAP_UARTEnable(HW::UART_BASE[i]);
        }
    }
    void disable() OVERRIDE
    {
        for (unsigned i = 0; i < ARRAYSIZE(HW::UART_BASE); ++i)
        {
            MAP_UARTDisable(HW::UART_BASE[i]);
        }
    }

    // RailcomDriver interface
    void feedback_sample() OVERRIDE {
        HW::enable_measurement(true);
        this->add_sample(HW::sample());
        HW::disable_measurement();
    }

    void start_cutout() OVERRIDE
    {
        HW::enable_measurement(false);
        const bool need_ch1_cutout = HW::need_ch1_cutout() || (this->feedbackKey_ < 11000);
        Debug::RailcomRxActivate::set(true);
        for (unsigned i = 0; i < ARRAYSIZE(HW::UART_BASE); ++i)
        {
            if (need_ch1_cutout)
            {
                HWREG(HW::UART_BASE[i] + UART_O_CTL) |= UART_CTL_RXE;
            }
            // HWREGBITW(HW::UART_BASE[i] + UART_O_CTL, UART_CTL_RXE) =
            //    UART_CTL_RXE;
            // flush fifo
            while (MAP_UARTCharGetNonBlocking(HW::UART_BASE[i]) >= 0)
                ;
            returnedPackets_[i] = 0;
        }
        Debug::RailcomDriverCutout::set(true);
    }

    void middle_cutout() OVERRIDE
    {
        Debug::RailcomDriverCutout::set(false);
        for (unsigned i = 0; i < ARRAYSIZE(HW::UART_BASE); ++i)
        {
            while (MAP_UARTCharsAvail(HW::UART_BASE[i]))
            {
                Debug::RailcomDataReceived::toggle();
                Debug::RailcomAnyData::set(true);
                if (!returnedPackets_[i])
                {
                    returnedPackets_[i] = this->alloc_new_packet(i);
                }
                if (!returnedPackets_[i])
                {
                    break;
                }
                long data = MAP_UARTCharGetNonBlocking(HW::UART_BASE[i]);
                if (data < 0 || data > 0xff) {
                    // We reset the receiver circuitry because we got an error
                    // in channel 1. Typical cause of this error is if there
                    // are multiple locomotives on the block (e.g. if this is
                    // the global detector) and they talk over each other
                    // during ch1 broadcast. There is a good likelihood that
                    // the asynchronous receiver is out of sync with the
                    // transmitter, but right now we should be in the
                    // between-byte space.
                    HWREG(HW::UART_BASE[i] + UART_O_CTL) &= ~UART_CTL_RXE;
                    Debug::RailcomError::toggle();
                    returnedPackets_[i]->add_ch1_data(0xF8 | ((data >> 8) & 0x7));
                    continue;
                }
                returnedPackets_[i]->add_ch1_data(data);
            }
            HWREG(HW::UART_BASE[i] + UART_O_CTL) |= UART_CTL_RXE;
        }
        HW::middle_cutout_hook();
        Debug::RailcomDriverCutout::set(true);
    }

    void end_cutout() OVERRIDE
    {
        HW::disable_measurement();
        bool have_packets = false;
        for (unsigned i = 0; i < ARRAYSIZE(HW::UART_BASE); ++i)
        {
            while (MAP_UARTCharsAvail(HW::UART_BASE[i]))
            {
                Debug::RailcomDataReceived::toggle();
                Debug::RailcomAnyData::set(true);
                Debug::RailcomCh2Data::set(true);
                if (!returnedPackets_[i])
                {
                    returnedPackets_[i] = this->alloc_new_packet(i);
                }
                if (!returnedPackets_[i])
                {
                    break;
                }
                long data = MAP_UARTCharGetNonBlocking(HW::UART_BASE[i]);
                if (data < 0 || data > 0xff) {
                    Debug::RailcomError::toggle();
                    returnedPackets_[i]->add_ch2_data(0xF8 | ((data >> 8) & 0x7));
                    continue;
                }
                if (data == 0xE0) {
                    Debug::RailcomE0::toggle();
                }
                returnedPackets_[i]->add_ch2_data(data);
            }
            HWREG(HW::UART_BASE[i] + UART_O_CTL) &= ~UART_CTL_RXE;
            Debug::RailcomRxActivate::set(false);
            //HWREGBITW(HW::UART_BASE[i] + UART_O_CTL, UART_CTL_RXE) = 0;
            if (returnedPackets_[i]) {
                have_packets = true;
                this->feedbackQueue_.commit_back();
                Debug::RailcomPackets::toggle();
                returnedPackets_[i] = nullptr;
                MAP_IntPendSet(HW::OS_INTERRUPT);
            }
        }
        if (!have_packets)
        {
            // Ensures that at least one feedback packet is sent back even when
            // it is with no railcom payload.
            auto *p = this->alloc_new_packet(0);
            if (p)
            {
                this->feedbackQueue_.commit_back();
                Debug::RailcomPackets::toggle();
                MAP_IntPendSet(HW::OS_INTERRUPT);
            }
        }
        Debug::RailcomCh2Data::set(false);
        Debug::RailcomDriverCutout::set(false);
    }

    void no_cutout() OVERRIDE
    {
        for (unsigned i = 0; i < ARRAYSIZE(HW::UART_BASE); ++i)
        {
            if (!returnedPackets_[i])
            {
                returnedPackets_[i] = this->alloc_new_packet(i);
            }
            if (returnedPackets_[i])
            {
                this->feedbackQueue_.commit_back();
                Debug::RailcomPackets::toggle();
                returnedPackets_[i] = nullptr;
                MAP_IntPendSet(HW::OS_INTERRUPT);
            }
        }
    }
};

#endif // _FREERTOS_DRIVERS_TI_TIVARAILCOM_HXX_

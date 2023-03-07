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

#include "freertos_drivers/common/RailcomImpl.hxx"
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

/// Railcom driver for TI Tiva-class microcontrollers using the TivaWare
/// peripheral library.
///
/// This railcom driver supports parallel polling of multiple UART channels for
/// the railcom data. 
template <class HW> class TivaRailcomDriver : public RailcomDriverBase<HW>
{
public:
    /// Constructor. @param path is the device node path (e.g. "/dev/railcom0").
    TivaRailcomDriver(const char *path)
        : RailcomDriverBase<HW>(path)
    {
        MAP_IntPrioritySet(HW::OS_INTERRUPT, configKERNEL_INTERRUPT_PRIORITY);
        MAP_IntEnable(HW::OS_INTERRUPT);
    }

    ~TivaRailcomDriver()
    {
        MAP_IntDisable(HW::OS_INTERRUPT);
    }

private:
    /// True when we are currently within a cutout.
    bool inCutout_ = false;

    using RailcomDriverBase<HW>::returnedPackets_;

    /// Sets a given software interrupt pending.
    /// @param int_nr interrupt number (will be HW::OS_INTERRUPT)
    void int_set_pending(unsigned int_nr) override
    {
        MAP_IntPendSet(int_nr);
    }

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

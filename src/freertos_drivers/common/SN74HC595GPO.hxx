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
 * @file SN74HC595GPO.hxx
 * This file implements the GPO driver for the 74HC595 shift register. The SN
 * prefix is for the Texas Instruments variant, however compatible devices
 * from other suppliers may have a different prefix or no prefix.
 *
 * @author Stuart W. Baker
 * @date 23 February 2020
 */

#ifndef _FREERTOS_DRIVERS_COMMON_SN74HC595GPO_HXX_
#define _FREERTOS_DRIVERS_COMMON_SN74HC595GPO_HXX_

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <fcntl.h>
#include <unistd.h>

#include "SPI.hxx"

#include "os/Gpio.hxx"
#include "os/OS.hxx"

template <uint8_t N = 1> class SN74HC595GPO;

/// Driver for the 74HC595 shift register. Note: This driver makes use of
/// std::atomic and requires an implementation of std::atomic on any platform
/// that makes use of it. For example, at the time of creating this device
/// driver, ARMv7-M (Cortex-M3/M4/M4F) does have support for std::atomic,
/// while ARMv6-M (Cortex-M0) does not.
/// @tparam N number of chips on the bus, starting at 1
template <uint8_t N = 1> class SN74HC595 : public OSThread
{
public:
    /// Constructor.
    /// @param request_refresh_operation This is an application specific
    ///        callback for triggering a sequence of events that will result
    ///        in @ref refresh() being called. The exact implementation is
    ///        application specific, and mostly is determined by whether or not
    ///        these outputs must be toggled from an interrupt.
    SN74HC595(void (*request_refresh_operation)(void))
        : requestRefreshOperation_(request_refresh_operation)
        , spiFd_(-1)
        , sem_()
        , ioPending_(false)
    {
        HASSERT(N > 0);
        for (unsigned i = 0; i < N; ++i)
        {
            gpoData_[i] = 0;
        }
    }

    /// Initialize the SN74HC595 settings. Typically called in hw_postinit(),
    /// not hw_preinit() or hw_init().
    /// @param spi_name spi interface that the SN74HC595 is on
    /// @param priority helper thread priority
    void init(const char *spi_name, int priority = get_priority_max())
    {
        spiFd_ = ::open(spi_name, O_WRONLY);
        HASSERT(spiFd_ >= 0);

        // configure SPI bus settings
        uint8_t spi_mode = SPI_MODE_0;
        uint8_t spi_bpw = 8;
        uint32_t spi_max_speed_hz = SPI_MAX_SPEED_HZ;
        ::ioctl(spiFd_, SPI_IOC_WR_MODE, &spi_mode);
        ::ioctl(spiFd_, SPI_IOC_WR_BITS_PER_WORD, &spi_bpw);
        ::ioctl(spiFd_, SPI_IOC_WR_MAX_SPEED_HZ, &spi_max_speed_hz);

        requestRefreshOperation_();

        // start the thread at the highest priority in the system
        start("SN74HC595", priority, 384 + N);
    }

    /// Triggers the helper thread to wakeup and refresh the outputs.
    /// @param from_isr true if called from an ISR.
    void refresh(bool from_isr = false)
    {
        if (!ioPending_)
        {
            ioPending_ = true;
            if (from_isr)
            {
                int woken;
                sem_.post_from_isr(&woken);
            }
            else
            {
                sem_.post();
            }
        }
    }

private:
    /// maximum SPI clock speed in Hz
    static constexpr uint32_t SPI_MAX_SPEED_HZ = 4000000;

    /// User entry point for the created thread.
    /// @return exit status
    void *entry() override
    {
        for ( ; /* forever */ ; )
        {
            sem_.wait();
            if (ioPending_)
            {
                ioPending_ = false;
                uint8_t data[N];
                for (unsigned i = 0; i < N; ++i)
                {
                    data[i] = gpoData_[i];
                }
                ::write(spiFd_, data, N);
            }
        }
    }

    /// Request that the GPIO cache be refreshed.
    void (*requestRefreshOperation_)(void);

    int spiFd_; ///< SPI bus that accesses the SN74HC595
    OSSem sem_; ///< semaphore for posting events
    uint8_t ioPending_ : 1; ///< true if an update is pending

    /// local copy of the expansion output data
    std::atomic<uint8_t> gpoData_[N];

    /// Allow access to SN74HC595 from SN74HC595GPO
    friend class SN74HC595GPO<N>;

    DISALLOW_COPY_AND_ASSIGN(SN74HC595);
};

/// General Purpose Output (GPO) instance on the 74HC595.
/// @tparam N number of chips on the bus, starting at 1
template <uint8_t N> class SN74HC595GPO : public Gpio
{
public:
    /// Constructor.
    /// @param instance parrent 74HC595 instance that "owns" the interface.
    /// @param chip_index index on the bus for the chip, starting at 0.
    /// @param bit bit index (0 through 7) of the output.
    constexpr SN74HC595GPO(SN74HC595<N> *instance, uint8_t chip_index, uint8_t bit)
        : Gpio()
        , instance_(instance)
        , chipIndex_(chip_index)
        , bit_(bit)
    {
#ifndef __PIC32MX__
        HASSERT(bit < 8);
        HASSERT(chip_index < sizeof(instance->gpoData_));
        HASSERT(N == sizeof(instance->gpoData_));
#endif
    }

    /// Writes a GPO pin (set or clear to a specific state).
    /// @param new_state the desired output state.  See @ref Value.
    void write(Value new_state) const override
    {
        new_state ? set() : clr();
    }

    /// Retrieves the current @ref Value of a GPO output sate (requested).
    /// @return @ref SET if currently high, @ref CLR if currently low
    Value read() const override
    {
        return instance_->gpoData_[chipIndex_] & (0x1 << bit_) ?
                   Gpio::SET : Gpio::CLR;
    }

    /// Sets the GPO pin to high.
    void set() const override
    {
        if (!(instance_->gpoData_[chipIndex_] & (0x1 << bit_)))
        {
            instance_->gpoData_[chipIndex_] |= 0x1 << bit_;
            instance_->requestRefreshOperation_();
        }
    }

    /// Clears the GPO pin to low.
    void clr() const override
    {
        if ((instance_->gpoData_[chipIndex_] & (0x1 << bit_)))
        {
            instance_->gpoData_[chipIndex_] &= ~(0x1 << bit_);
            instance_->requestRefreshOperation_();
        }
    }

    /// Sets the GPO direction (does nothing).
    /// @param dir @ref DINPUT or @ref DOUTPUT
    void set_direction(Gpio::Direction dir) const override
    {
        HASSERT(dir == Gpio::Direction::DOUTPUT);
    }

    /// Gets the GPO direction.
    /// @return always returns @ref DOUTPUT
    Direction direction() const override
    {
        return Gpio::Direction::DOUTPUT;
    }

private:
    /// reference to chip instance
    SN74HC595<N> * const instance_;

    /// index on the bus for the chip
    const uint8_t chipIndex_;

    /// bit number representative of the bit
    const uint8_t bit_;

    DISALLOW_COPY_AND_ASSIGN(SN74HC595GPO);
};

#endif // _FREERTOS_DRIVERS_COMMON_SN74HC595GPO_HXX_

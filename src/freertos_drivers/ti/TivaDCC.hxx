/** \copyright
 * Copyright (c) 2014, Stuart W Baker
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
 * \file TivaDCC.hxx
 * Device class prototype for DCC driver on TivaWare.
 *
 * @author Stuart W. Baker
 * @date 6 May 2014
 */

#ifndef _FREERTOS_DRIVERS_TI_TIVADCC_HXX_
#define _FREERTOS_DRIVERS_TI_TIVADCC_HXX_

#ifndef gcc
#define gcc
#endif

#if (!defined(TIVADCC_TIVA)) && (!defined(TIVADCC_CC3200))
#error must define either TIVADCC_TIVA or TIVADCC_CC3200
#endif

#include <algorithm>
#include <cstdint>

#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "freertos/can_ioctl.h"
#include "inc/hw_memmap.h"
#include "inc/hw_timer.h"
#include "inc/hw_types.h"
#include "inc/hw_uart.h"

#ifdef TIVADCC_TIVA
#include "driverlib/sysctl.h"
#include "TivaGPIO.hxx"
#else
#include "driverlib/prcm.h"
#include "driverlib/utils.h"
#endif

#include "freertos_drivers/common/FixedQueue.hxx"
#include "Devtab.hxx"
#include "RailcomDriver.hxx"
#include "dcc/DccOutput.hxx"
#include "dcc/Packet.hxx"
#include "dcc/RailCom.hxx"
#include "executor/Notifiable.hxx"

/// If non-zero, enables the jitter feature to spread the EMC spectrum of DCC
/// signal
extern "C" uint8_t spreadSpectrum;

/** A device driver for sending DCC packets.  If the packet queue is empty,
 *  then the device driver automatically sends out idle DCC packets.  The
 *  device driver uses two instances of the 16/32-bit timer pairs.  The user
 *  is responsible for providing interrupt entry point for the interval timer
 *  and calling the inline method @ref interrupt_handler on behalf of this
 *  device driver.
 *
 *  Write calls work by sending the packet in the format of dcc::Packet.  The
 *  payload should include the X-OR linkage byte.  Only one DCC packet may be
 *  written per call to the write method.  If there is no space currently
 *  available in the write queue, the write method will return -1 with errno
 *  set to ENOSPC.
 *
 *  Handling of write throttling:
 *
 *  The driver uses a second interrupt, used as a software interrupt at the
 *  FreeRTOS priority level, to communicate back information on when the device
 *  is writable. It is okay to use the interrupt for the second timer for this
 *  purpose. This interrupt will be enabled in the IOCTL after the
 *  writenotifiable is set, disabled in the interrupt itself when the
 *  writenotifiable is cleared. It is set pending at startup and when the dcc
 *  interrupt frees up a packet from the queue, and it is cleared pending when
 *  a write realizes there is no space for a buffer.
 *
 *  The user must ensure that the 'interrupt' has a very high priority, whereas
 *  'os_interrupt' has a priority that's at or lower than the FreeRTOS kernel.
 *
 *
 *  Handling of feedback data:
 *
 *  The driver may generate return data for the application layer in the form
 *  of dcc::Feedback messages.  These will be attributed to the incoming
 *  packets by an opaque key that the application sets. For each packet that
 *  has a non-zero feedback a dcc::Feedback message will be sent to the
 *  application layer, which will be readable using the read method on the fd.
 *
 *  The application can request notification of readable and writable status
 *  using the regular IOCTL method.
 *
 *
 *  EMC spectrum spreading
 *
 *  There is an optional feature that helps with passing EMC certification for
 *  systems that are built on this driver. The observation is that if the
 *  output signal has may repeats of a certain period, then in the measured
 *  spectrum there will be a big spike in energy that might exceed the
 *  thresholds for compliance. However, by slightly varying the timing of the
 *  output signal, the energy will be spread across a wider spectrum, thus the
 *  peak of emission will be smaller.
 *
 *  This feature is enabled by `extern uint8_t spreadSpectrum;`. This can come
 *  from a constant or configuration dependent variable. If enabled, then the
 *  timing of DCC zero bits are stretched to be a random value between 100.5
 *  and 105 usec each half; the timing of DCC one bits will be stretched from
 *  56.5 to 60 usec per half. The symmetry within each bit is still perfectly
 *  matched. Marklin-Motorola packets get up to 2 usec of stretching on each
 *  phase.
 *
 *  The actual stretching is generated using a uniform random number generator
 *  within said limits to ensure we spread uniformly across the available
 *  timings. Up to four bits are output with the same timing, then a new random
 *  timing is generated.
 */
template<class HW>
class TivaDCC : public Node
{
public:
    /** Constructor.
     * @param name name of this device instance in the file system
     * @param railcom is the associated railcom driver, which will get the
     * callbacks from the timing derived by the internal signal generator.
     */
  TivaDCC(const char *name, RailcomDriver *railcom);

    /** Destructor.
     */
    ~TivaDCC()
    {
    }

    /** Handle an interrupt.
     */
    inline void interrupt_handler() __attribute__((always_inline));

    /** Handles a software interrupt to FreeRTOS. This should be called on the
     * interrupt number that is submitted as os_interrupt to the
     * constructor. */
    inline void os_interrupt_handler() __attribute__((always_inline));

    /** Structure for supporting bit timing. */
    struct Timing {
        /// In clock cycles: period of the interval timer
        uint32_t interval_period;
        /// In clock cycles: period of the PWM timer
        uint32_t period;
        /// When to transition output A; must be within the period
        uint32_t transition_a;
        /// When to transition output B; must be within the period
        uint32_t transition_b;
        /// How many ticks (minimum) we can add to the period and transition for
        /// spectrum spreading.
        uint16_t spread_min = 0;
        /// How many ticks (maximum) we can add to the period and transition for
        /// spectrum spreading.
        uint16_t spread_max = 0;
    };

    /* WARNING: these functions (hw_init, enable_output, disable_output) MUST
     * be static, because they will be called from hw_preinit, which happens
     * before the C++ constructors have run. This means that at the time of
     * calling these functions the state of the object would be undefined /
     * uninintialized. The only safe solution is to make them static. */
    /// Initializes the DCC output hardware.
    static void hw_preinit()
    {
#ifdef TIVADCC_TIVA
        MAP_SysCtlPeripheralEnable(HW::CCP_PERIPH);
        MAP_SysCtlPeripheralEnable(HW::INTERVAL_PERIPH);
        MAP_SysCtlPeripheralEnable(HW::RAILCOM_UART_PERIPH);
        HW::PIN_H::hw_init();
        HW::PIN_L::hw_init();
        HW::RAILCOM_TRIGGER_Pin::hw_init();
        HW::RAILCOM_UARTPIN::hw_init();
#else
        MAP_PRCMPeripheralClkEnable(HW::CCP_PERIPH, PRCM_RUN_MODE_CLK);
        MAP_PRCMPeripheralClkEnable(HW::INTERVAL_PERIPH, PRCM_RUN_MODE_CLK);
        MAP_PRCMPeripheralClkEnable(HW::RAILCOM_UART_PERIPH, PRCM_RUN_MODE_CLK);
#endif
        HW::Output1::hw_preinit();
        HW::Output2::hw_preinit();
        HW::Output3::hw_preinit();
        MAP_UARTConfigSetExpClk(
            HW::RAILCOM_UART_BASE, configCPU_CLOCK_HZ, 250000,
            UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE);
        MAP_UARTFIFOEnable(HW::RAILCOM_UART_BASE);
        // Disables the uart receive until the railcom cutout is here.
        HWREG(HW::RAILCOM_UART_BASE + UART_O_CTL) &= ~UART_CTL_RXE;
    }

private:
    /** Read from a file or device.
     * @param file file reference for this device
     * @param buf location to place read data
     * @param count number of bytes to read
     * @return number of bytes read upon success, -1 upon failure with errno containing the cause
     */
    ssize_t read(File *file, void *buf, size_t count) OVERRIDE;

    /** Write to a file or device.
     * @param file file reference for this device
     * @param buf location to find write data
     * @param count number of bytes to write
     * @return number of bytes written upon success, -1 upon failure with errno containing the cause
     */
    ssize_t write(File *file, const void *buf, size_t count) OVERRIDE;

    /** Request an ioctl transaction
     * @param file file reference for this device
     * @param key ioctl key
     * @param data key data
     */
    int ioctl(File *file, unsigned long int key, unsigned long data) OVERRIDE;

    void enable() OVERRIDE {} /**< function to enable device */
    void disable() OVERRIDE {} /**< function to disable device */

    /** Discards all pending buffers.  Called after disable().
     */
    void flush_buffers() override {};

    /** maximum packet size we can support */
    static const size_t MAX_PKT_SIZE = 6;


    /** idle packet */
    static dcc::Packet IDLE_PKT;

    /// Bit timings that we store and precalculate.
    typedef enum
    {
        /// Zero bit for DCC (this is the longer)
        DCC_ZERO,
        /// One bit for DCC (this is the shorter)
        DCC_ONE,
        /// One bit for DCC that we generate at the end of packet
        /// transition. This has a stretched negative part so that the next
        /// packet resync avoids a glitch in the output.
        DCC_EOP_ONE,
        /// One bit for DCC that we generate during the railcom cutout. Can be
        /// used to play with alignment of edges coming out of the railcom
        /// cutout.
        DCC_RC_ONE,
        /// Half zero bit which is sent directly after the railcom cutout is
        /// over. Needed to reset certain old decoders packet
        /// recognizer. Recommended by
        /// https://nmra.org/sites/default/files/standards/sandrp/pdf/tn-2-05draft2005-02-25_for_rp-9.2.3.pdf
        DCC_RC_HALF_ZERO,
        /// This is not a bit, but specifies when to wake up during the railcom
        /// cutout. The time is T_CS, usually 26 usec.
        RAILCOM_CUTOUT_PRE,
        /// This is not a bit, but specifies when to wake up during the railcom
        /// cutout. The time is the time elapsed between T_CS and the middle of
        /// the two windows.
        RAILCOM_CUTOUT_FIRST,
        /// This is not a bit, but specifies when to wake up during the railcom
        /// cutout. The time is the time elapsed between the end of the cutout
        /// and the the middle of the two windows.
        RAILCOM_CUTOUT_SECOND,
        /// This is not a bit, but specifies when to wake up during the railcom
        /// cutout. This is used for re-synchronizing the
        RAILCOM_CUTOUT_POST,
        /// Long negative DC pulse to act as a preamble for a Marklin packet.
        MM_PREAMBLE,
        /// Zero bit for MM packet, which is a short pulse in one direction,
        /// then a long pulse in the other.
        MM_ZERO,
        /// One bit for MM packet, which is a long pulse in one direction, then
        /// a short pulse in the other.
        MM_ONE,

        NUM_TIMINGS
    } BitEnum;

    int hDeadbandDelay_; /**< low->high deadband delay in clock count */
    int lDeadbandDelay_; /**< high->low deadband delay in clock count */
    int usecDelay_; /**< 1 usec of delay in clock count */

    /// Precalculated bit timings (translated to clock cycles).
    Timing timings[NUM_TIMINGS];

    /// Internal state machine states.
    enum State
    {
        // DCC preamble bits
        PREAMBLE,
        // Packet start bit
        START,
        // Data bits for DCC
        DATA_0,
        DATA_1,
        DATA_2,
        DATA_3,
        DATA_4,
        DATA_5,
        DATA_6,
        DATA_7,
        // end-of-byte bit for DCC (either middle-of-packet or end-of-packet)
        FRAME,
        // Marklin preamble "bit" (constant negative voltage)
        ST_MM_PREAMBLE,
        // MM data bits. The bit number here is an offset in the current input
        // byte, because MM packets contain 18 consecutive bits.
        MM_DATA_0,
        MM_DATA_1,
        MM_DATA_2,
        MM_DATA_3,
        MM_DATA_4,
        MM_DATA_5,
        MM_DATA_6,
        MM_DATA_7,
        // Internal state used for end-of-packet resynchronization of timers.
        RESYNC,
        // State at the end of packet where we make a decision whether to go
        // into a railcom cutout or not.
        DCC_MAYBE_RAILCOM,
        // If we did not generate a cutout, we generate 5 empty one bits here
        // in case a booster wants to insert a cutout.
        DCC_NO_CUTOUT,
        // Counts 26 usec into the appropriate preamble bit after which to turn
        // off output power.
        DCC_CUTOUT_PRE,
        // Start of railcom. Turns off output power and enables UART.
        DCC_START_RAILCOM_RECEIVE,
        // Point between railcom window 1 and railcom window 2 where we have to
        // read whatever arrived for channel1.
        DCC_MIDDLE_RAILCOM_CUTOUT,
        // Railcom end-of-channel2 window. Reads out the UART values, and
        // enables output power.
        DCC_STOP_RAILCOM_RECEIVE,
        // Same for marklin. A bit of negative voltage after the packet is over
        // but before loading the next packet. This ensures that old marklin
        // decoders confirm receiving the packet correctly before we go into a
        // potential protocol switch from the next packet loaded.
        MM_LEADOUT,

        // Turn on without going through the slow start sequence.
        POWER_IMM_TURNON,
    };
    /// Current state of internal state machine.
    State state_;

    /** Prepares a timing entry.
     *
     * @param ofs is the bit timing that we are defining.
     * @param period_usec is the total length of the bit.
     * @param transition_usec is the time of the transition inside the bit,
     * counted from the beginning of the bit (i.e. the length of the HIGH part
     * of the period). Can be zero for DC output LOW or can be == period_usec
     * for DC output HIGH.
     * @param interval_period_usec tells when the interval timer should expire
     * (next interrupt). Most of the time this should be the same as
     * period_usec.
     * @param timing_spread_usec if non-zero, allows the high and low of the
     * timing to be stretched by at most this many usec.
     */
    void fill_timing(BitEnum ofs, uint32_t period_usec,
        uint32_t transition_usec, uint32_t interval_period_usec,
        uint32_t timing_spread_usec = 0);

    /// Checks each output and enables those that need to be on.
    void check_and_enable_outputs()
    {
        if (HW::Output1::should_be_enabled())
        {
            HW::Output1::enable_output();
        }
        if (HW::Output2::should_be_enabled())
        {
            HW::Output2::enable_output();
        }
        if (HW::Output3::should_be_enabled())
        {
            HW::Output3::enable_output();
        }
    }

#ifdef TIVADCC_CC3200
    // This function is called differently in tivaware than CC3200.
    void MAP_SysCtlDelay(unsigned ticks3) {
        ROM_UtilsDelay(ticks3);
    }
#endif

    /// Standard timing value of when the railcom cutout should start, measured
    /// from the transition of the end-of-packet-one-bit. Minimum 26, max 32
    /// usec. Can be adjusted by HW.
    static constexpr unsigned RAILCOM_CUTOUT_START_USEC = 26;
    /// Standard timing value of the railcom cutout middle, measured from the
    /// transition of the end-of-packet-one-bit. Minimum 177 (end of channel
    /// one), maximum 193 (start channel 2) usec. Can be adjusted by HW.
    static constexpr unsigned RAILCOM_CUTOUT_MID_USEC = 185;
    /// Standard timing value of the railcom cutout end, measured from the
    /// transition of the end-of-packet-one-bit. Minimum 454 (end of channel
    /// one), maximum 488. Can be adjusted by HW.
    static constexpr unsigned RAILCOM_CUTOUT_END_USEC = 486;
    
    /// Packets still waiting to be sent.
    FixedQueue<dcc::Packet, HW::Q_SIZE> packetQueue_;
    Notifiable* writableNotifiable_; /**< Notify this when we have free buffers. */
    RailcomDriver* railcomDriver_; /**< Will be notified for railcom cutout events. */
    /// Seed for a pseudorandom sequence.
    unsigned seed_ = 0xb7a11bae;

    /// Parameters for a linear RNG: modulus
    static constexpr unsigned PMOD = 65213;
    /// Parameters for a linear RNG: multiplier
    static constexpr unsigned PMUL = 52253;
    /// Parameters for a linear RNG: additive
    static constexpr unsigned PADD = 42767;
    /** Default constructor.
     */
    TivaDCC();

    DISALLOW_COPY_AND_ASSIGN(TivaDCC);
};


/** Handle an interrupt.
 */
template <class HW>
__attribute__((optimize("-O3")))
inline void TivaDCC<HW>::interrupt_handler()
{
    static int preamble_count = 0;
    static BitEnum last_bit = DCC_ONE;
    static int count = 0;
    static int packet_repeat_count = 0;
    static int bit_repeat_count = 0;
    static const dcc::Packet *packet = &IDLE_PKT;
    static bool resync = true;
    BitEnum current_bit;
    bool get_next_packet = false;

    MAP_TimerIntClear(HW::INTERVAL_BASE, TIMER_TIMA_TIMEOUT);

    switch (state_)
    {
        default:
        case RESYNC:
            if (packet->packet_header.is_marklin)
            {
                current_bit = MM_PREAMBLE;
                state_ = ST_MM_PREAMBLE;
                break;
            }
            else
            {
                state_ = PREAMBLE;
            }
            // fall through
        case PREAMBLE:
        {
            current_bit = DCC_ONE;
            // preamble zero is output twice due to the resync, so we deduct
            // one from the count.
            int preamble_needed = HW::dcc_preamble_count() - 1;
            if (HW::generate_railcom_halfzero() &&
                !packet->packet_header.send_long_preamble)
            {
                if (preamble_count == 0)
                {
                    current_bit = DCC_RC_HALF_ZERO;
                }
                preamble_needed++;
            }
            if (packet->packet_header.send_long_preamble)
            {
                preamble_needed = 21;
            }
            if (++preamble_count >= preamble_needed)
            {
                state_ = START;
                preamble_count = 0;
            }
            break;
        }
        case START:
            current_bit = DCC_ZERO;
            count = 0;
            state_ = DATA_0;
            break;
        case DATA_0:
        case DATA_1:
        case DATA_2:
        case DATA_3:
        case DATA_4:
        case DATA_5:
        case DATA_6:
        case DATA_7:
        {
            uint8_t bit = (packet->payload[count] >> (DATA_7 - state_)) & 0x01;
            current_bit = static_cast<BitEnum>(DCC_ZERO + bit);
            state_ = static_cast<State>(static_cast<int>(state_) + 1);
            break;
        }
        case FRAME:
            if (++count >= packet->dlc)
            {
                current_bit = DCC_RC_ONE;  // end-of-packet bit
                state_ = DCC_MAYBE_RAILCOM;
                preamble_count = 0;
            }
            else
            {
                current_bit = DCC_ZERO;  // end-of-byte bit
                state_ = DATA_0;
            }
            break;
        case DCC_MAYBE_RAILCOM:
            if ((packet->packet_header.send_long_preamble == 0) &&
                (HW::Output1::need_railcom_cutout() ||
                    HW::Output2::need_railcom_cutout() ||
                    HW::Output3::need_railcom_cutout()))
            {
                current_bit = DCC_RC_ONE;
                // It takes about 5 usec to get here from the previous
                // transition of the output.
                // We change the time of the next IRQ.
                MAP_TimerLoadSet(HW::INTERVAL_BASE, TIMER_A,
                    timings[RAILCOM_CUTOUT_PRE].interval_period);
                state_ = DCC_CUTOUT_PRE;
            }
            else
            {
                railcomDriver_->no_cutout();
                current_bit = DCC_ONE;
                state_ = DCC_NO_CUTOUT;
            }
            break;
        case DCC_NO_CUTOUT:
            current_bit = DCC_ONE;
            ++preamble_count;
            // maybe railcom already sent one extra ONE bit after the
            // end-of-packet one bit. We need four more.
            if (preamble_count >= 4)
            {
                current_bit = DCC_EOP_ONE;
            }
            if (preamble_count >= 5)
            {
                // The last bit will be removed by the next packet's beginning
                // sync.
                get_next_packet = true;
            }
            break;
        case DCC_CUTOUT_PRE:
            current_bit = DCC_RC_ONE;
            // It takes about 3.6 usec to get here from the transition seen on
            // the output.
            // We change the time of the next IRQ.
            MAP_TimerLoadSet(HW::INTERVAL_BASE, TIMER_A,
                timings[RAILCOM_CUTOUT_FIRST].interval_period);
            state_ = DCC_START_RAILCOM_RECEIVE;
            break;
        case DCC_START_RAILCOM_RECEIVE:
        {
            bool rc1 = HW::Output1::need_railcom_cutout();
            bool rc2 = HW::Output2::need_railcom_cutout();
            bool rc3 = HW::Output3::need_railcom_cutout();
            unsigned delay = 0;
            // Phase 1
            if (rc1)
            {
                delay =
                    std::max(delay, HW::Output1::start_railcom_cutout_phase1());
                HW::Output1::isRailcomCutoutActive_ = 1;
            }
            if (rc2)
            {
                delay =
                    std::max(delay, HW::Output2::start_railcom_cutout_phase1());
                HW::Output2::isRailcomCutoutActive_ = 1;
            }
            if (rc3)
            {
                delay =
                    std::max(delay, HW::Output3::start_railcom_cutout_phase1());
                HW::Output3::isRailcomCutoutActive_ = 1;
            }
            // Delay
            if (delay)
            {
                MAP_SysCtlDelay(usecDelay_ * delay);
            }
            delay = 0;
            // Phase 2
            if (rc1)
            {
                delay =
                    std::max(delay, HW::Output1::start_railcom_cutout_phase2());
            }
            if (rc2)
            {
                delay =
                    std::max(delay, HW::Output2::start_railcom_cutout_phase2());
            }
            if (rc3)
            {
                delay =
                    std::max(delay, HW::Output3::start_railcom_cutout_phase2());
            }
            // Delay
            if (delay)
            {
                MAP_SysCtlDelay(usecDelay_ * delay);
            }
            // Enables UART RX.
            railcomDriver_->start_cutout();
            // Set up for next wakeup.
            current_bit = DCC_RC_ONE;
            MAP_TimerLoadSet(HW::INTERVAL_BASE, TIMER_A,
                timings[RAILCOM_CUTOUT_SECOND].interval_period);
            state_ = DCC_MIDDLE_RAILCOM_CUTOUT;
            break;
        }
        case DCC_MIDDLE_RAILCOM_CUTOUT:
            railcomDriver_->middle_cutout();
            current_bit = DCC_RC_ONE;
            MAP_TimerLoadSet(HW::INTERVAL_BASE, TIMER_A,
                timings[RAILCOM_CUTOUT_POST].interval_period);
            state_ = DCC_STOP_RAILCOM_RECEIVE;
            break;
        case DCC_STOP_RAILCOM_RECEIVE:
        {
            current_bit = RAILCOM_CUTOUT_POST;
            // This causes the timers to be reinitialized so no fractional bits
            // are left in their counters.
            resync = true;
            get_next_packet = true;
            railcomDriver_->end_cutout();
            unsigned delay = 0;
            if (HW::Output1::isRailcomCutoutActive_)
            {
                delay =
                    std::max(delay, HW::Output1::stop_railcom_cutout_phase1());
            }
            if (HW::Output2::isRailcomCutoutActive_)
            {
                delay =
                    std::max(delay, HW::Output2::stop_railcom_cutout_phase1());
            }
            if (HW::Output3::isRailcomCutoutActive_)
            {
                delay =
                    std::max(delay, HW::Output3::stop_railcom_cutout_phase1());
            }
            // Delay
            if (delay)
            {
                MAP_SysCtlDelay(usecDelay_ * delay);
            }
            if (HW::Output1::isRailcomCutoutActive_)
            {
                HW::Output1::stop_railcom_cutout_phase2();
            }
            HW::Output1::isRailcomCutoutActive_ = 0;
            if (HW::Output2::isRailcomCutoutActive_)
            {
                HW::Output2::stop_railcom_cutout_phase2();
            }
            HW::Output2::isRailcomCutoutActive_ = 0;
            if (HW::Output3::isRailcomCutoutActive_)
            {
                HW::Output3::stop_railcom_cutout_phase2();
            }
            HW::Output3::isRailcomCutoutActive_ = 0;
            check_and_enable_outputs();
            break;
        }
        case ST_MM_PREAMBLE:
            current_bit = MM_PREAMBLE;
            ++preamble_count;
            if (preamble_count == 7 ||
                preamble_count == 7 + 6 ||
                preamble_count == 7 + 6 + 10 ||
                preamble_count == 7 + 6 + 10 + 6)
            {
                // first byte contains two bits.
                state_ = MM_DATA_6;
            }
            break;
        case MM_LEADOUT:
            // MM packets never have a cutout.
            railcomDriver_->no_cutout();
            current_bit = MM_PREAMBLE;
            if (++preamble_count >= 2) {
                get_next_packet = true;
            }
            break;
        case MM_DATA_0:
        case MM_DATA_1:
        case MM_DATA_2:
        case MM_DATA_3:
        case MM_DATA_4:
        case MM_DATA_5:
        case MM_DATA_6:
        {
            uint8_t bit =
                (packet->payload[count] >> (MM_DATA_7 - state_)) & 0x01;
            current_bit = static_cast<BitEnum>(MM_ZERO + bit);
            state_ = static_cast<State>(static_cast<int>(state_) + 1);
            break;
        }
        case MM_DATA_7:
        {
            uint8_t bit =
                (packet->payload[count] >> (MM_DATA_7 - state_)) & 0x01;
            current_bit = static_cast<BitEnum>(MM_ZERO + bit);
            ++count;
            if (count == 3) {
                state_ = ST_MM_PREAMBLE;
            } else if (count >= packet->dlc) {
                preamble_count = 0;
                state_ = MM_LEADOUT;
            } else {
                state_ = MM_DATA_0;
            }
            break;
        }
        case POWER_IMM_TURNON:
            current_bit = DCC_ONE;
            packet_repeat_count = 0;
            get_next_packet = true;
            resync = true;
            break;
    }

    if (resync) {
        resync = false;
        TDebug::Resync::toggle();
        auto* timing = &timings[current_bit];
        // We are syncing -- cause timers to restart counting when these
        // execute.
        HWREG(HW::CCP_BASE + TIMER_O_TAMR) &=
            ~(TIMER_TAMR_TAMRSU | TIMER_TAMR_TAILD);
        HWREG(HW::CCP_BASE + TIMER_O_TBMR) &=
            ~(TIMER_TBMR_TBMRSU | TIMER_TBMR_TBILD);
        HWREG(HW::INTERVAL_BASE + TIMER_O_TAMR) &=
            ~(TIMER_TAMR_TAMRSU | TIMER_TAMR_TAILD);

        // These have to happen very fast because syncing depends on it. We do
        // direct register writes here instead of using the plib calls.
        HWREG(HW::INTERVAL_BASE + TIMER_O_TAILR) =
            timing->interval_period + hDeadbandDelay_ * 2;

        if (!HW::H_DEADBAND_DELAY_NSEC)
        {
            TDebug::Resync::toggle();
            MAP_TimerDisable(HW::CCP_BASE, TIMER_A|TIMER_B);
            // Sets final values for the cycle.
            MAP_TimerLoadSet(HW::CCP_BASE, TIMER_A|TIMER_B, timing->period);
            MAP_TimerMatchSet(HW::CCP_BASE, TIMER_A, timing->transition_a);
            MAP_TimerMatchSet(HW::CCP_BASE, TIMER_B, timing->transition_b);
            MAP_TimerEnable(HW::CCP_BASE, TIMER_A | TIMER_B);

            MAP_TimerDisable(HW::INTERVAL_BASE, TIMER_A);
            MAP_TimerLoadSet(
                HW::INTERVAL_BASE, TIMER_A, timing->interval_period);
            MAP_TimerEnable(HW::INTERVAL_BASE, TIMER_A);
            TDebug::Resync::toggle();

            // Switches back to asynch timer update.
            HWREG(HW::CCP_BASE + TIMER_O_TAMR) |=
                (TIMER_TAMR_TAMRSU | TIMER_TAMR_TAILD);
            HWREG(HW::CCP_BASE + TIMER_O_TBMR) |=
                (TIMER_TBMR_TBMRSU | TIMER_TBMR_TBILD);
            HWREG(HW::INTERVAL_BASE + TIMER_O_TAMR) |=
                (TIMER_TAMR_TAMRSU | TIMER_TAMR_TAILD);
        }
        else
        {

            HWREG(HW::CCP_BASE + TIMER_O_TBMATCHR) =
                timing->transition_b; // final
            // since timer A starts later, the deadband delay cycle we set to be
            // constant off (match == period)
            HWREG(HW::CCP_BASE + TIMER_O_TAMATCHR) = hDeadbandDelay_; // tmp
            HWREG(HW::CCP_BASE + TIMER_O_TBILR) = timing->period;     // final
            HWREG(HW::CCP_BASE + TIMER_O_TAILR) = hDeadbandDelay_;    // tmp

// timer synchronize (if it works...)
#ifdef TIVADCC_TIVA
            HWREG(TIMER0_BASE + TIMER_O_SYNC) = HW::TIMER_SYNC;
#endif

            // Switches back to asynch timer update.
            HWREG(HW::CCP_BASE + TIMER_O_TAMR) |=
                (TIMER_TAMR_TAMRSU | TIMER_TAMR_TAILD);
            HWREG(HW::CCP_BASE + TIMER_O_TBMR) |=
                (TIMER_TBMR_TBMRSU | TIMER_TBMR_TBILD);
            HWREG(HW::INTERVAL_BASE + TIMER_O_TAMR) |=
                (TIMER_TAMR_TAMRSU | TIMER_TAMR_TAILD);

            // Sets final values for the cycle.
            MAP_TimerLoadSet(HW::CCP_BASE, TIMER_A, timing->period);
            MAP_TimerMatchSet(HW::CCP_BASE, TIMER_A, timing->transition_a);
            MAP_TimerLoadSet(
                HW::INTERVAL_BASE, TIMER_A, timing->interval_period);
        }
        
        last_bit = current_bit;
        bit_repeat_count = 0;
        if (current_bit == RAILCOM_CUTOUT_POST)
        {
            // RAILCOM_CUTOUT_POST purposefully misaligns the two timers. We
            // need to resync when the next interval timer ticks to get them
            // back.
            resync = true;
        }
        else if (current_bit == DCC_RC_HALF_ZERO)
        {
            // After resync the same bit is output twice. We don't want that
            // with the half-zero, so we preload the DCC preamble bit.
            current_bit = DCC_ONE;
        }
    }
    if (bit_repeat_count >= 4)
    {
        // Forces reinstalling the timing.
        last_bit = NUM_TIMINGS;
    }
    if (last_bit != current_bit)
    {
        auto* timing = &timings[current_bit];
        // The delta in ticks we add to each side of the signal.
        uint32_t spread = 0;
        if (spreadSpectrum)
        {
            spread = timing->spread_max - timing->spread_min;
            seed_ *= PMUL;
            seed_ += PADD;
            seed_ %= PMOD;
            spread = (seed_ % spread) + timing->spread_min;
        }
        MAP_TimerLoadSet(HW::INTERVAL_BASE, TIMER_A,
            timing->interval_period + (spread << 1));
        MAP_TimerLoadSet(HW::CCP_BASE, TIMER_A, timing->period + (spread << 1));
        MAP_TimerLoadSet(HW::CCP_BASE, TIMER_B, timing->period + (spread << 1));
        MAP_TimerMatchSet(HW::CCP_BASE, TIMER_A, timing->transition_a + spread);
        MAP_TimerMatchSet(HW::CCP_BASE, TIMER_B, timing->transition_b + spread);
        last_bit = current_bit;
        bit_repeat_count = 0;
    }
    else
    {
        bit_repeat_count++;
    }

    if (get_next_packet)
    {
        check_and_enable_outputs();
        TDebug::NextPacket::toggle();
        if (packet_repeat_count) {
            --packet_repeat_count;
        }
        if (packet != &IDLE_PKT && packet_repeat_count == 0)
        {
            packetQueue_.increment_front();
            // Notifies the OS that we can write to the buffer.
            MAP_IntPendSet(HW::OS_INTERRUPT);
            resync = true;
        } else {
        }
        if (!packetQueue_.empty())
        {
            packet = &packetQueue_.front();
            railcomDriver_->set_feedback_key(packet->feedback_key);
        }
        else
        {
            packet = &IDLE_PKT;
            resync = true;
        }
        preamble_count = 0;
        count = 0;
        if (!packet_repeat_count) {
            packet_repeat_count = packet->packet_header.rept_count + 1;
        }
        // If we are in the repeat loop for a marklin packet, we do not do a
        // resync to not disturb the marklin preamble (which is DC_negative).
        if (resync || !packet->packet_header.is_marklin) {
            state_ = RESYNC;
        } else {
            state_ = ST_MM_PREAMBLE;
        }
    }
}

/// Converts a time length given in microseconds to the number of clock cycles.
/// @param usec is time given in microseconds.
/// @return time given in clock cycles.
static const uint32_t usec_to_clocks(uint32_t usec) {
    return (configCPU_CLOCK_HZ / 1000000) * usec;
}

/// Converts a time length given in nanoseconds to the number of clock cycles.
/// @param nsec is time given in nanoseconds.
/// @return time given in clock cycles.
static uint32_t nsec_to_clocks(uint32_t nsec) {
    // We have to be careful here not to underflow or overflow.
    return ((configCPU_CLOCK_HZ / 1000000) * nsec) / 1000;
}

template <class HW>
void TivaDCC<HW>::fill_timing(BitEnum ofs, uint32_t period_usec,
    uint32_t transition_usec, uint32_t interval_usec, uint32_t spread_max)
{
    auto* timing = &timings[ofs];
    timing->period = usec_to_clocks(period_usec);
    timing->interval_period = usec_to_clocks(interval_usec);
    if (transition_usec == 0) {
        // DC voltage negative.
        timing->transition_a = timing->transition_b = timing->period;
    } else if (transition_usec >= period_usec) {
        // DC voltage positive.
        // We use the PLO feature of the timer.
        timing->transition_a = timing->transition_b = timing->period + 1;
    } else {
        int32_t nominal_transition =
            timing->period - usec_to_clocks(transition_usec);
        timing->transition_a =
            nominal_transition + (hDeadbandDelay_ + lDeadbandDelay_) / 2;
        timing->transition_b =
            nominal_transition - (hDeadbandDelay_ + lDeadbandDelay_) / 2;
    }
    if (spread_max > 0)
    {
        timing->spread_min = usec_to_clocks(1) / 2;
        timing->spread_max = usec_to_clocks(spread_max);
    }
}

template<class HW>
dcc::Packet TivaDCC<HW>::IDLE_PKT = dcc::Packet::DCC_IDLE();

template <class HW>
TivaDCC<HW>::TivaDCC(const char *name, RailcomDriver *railcom_driver)
    : Node(name)
    , hDeadbandDelay_(nsec_to_clocks(HW::H_DEADBAND_DELAY_NSEC))
    , lDeadbandDelay_(nsec_to_clocks(HW::L_DEADBAND_DELAY_NSEC))
    , usecDelay_(nsec_to_clocks(1000) / 3)
    , writableNotifiable_(nullptr)
    , railcomDriver_(railcom_driver)
{
    state_ = PREAMBLE;

    fill_timing(DCC_ZERO, 100 << 1, 100, 100 << 1, 5);
    fill_timing(DCC_ONE, 56 << 1, 56, 56 << 1, 4);
    /// @todo tune this bit to line up with the bit stream starting after the
    /// railcom cutout.
    fill_timing(DCC_RC_ONE, 57 << 1, 57, 57 << 1);

    // The following #if switch controls whether or not the
    // "generate_railcom_halfzero()" will actually generate a half zero bit
    // or if it will in actuality generate a full zero bit. It was determined
    // that the half zero workaround does not work with some older decoders,
    // but the full zero workaround does. It also works with older decoders
    // that needed the half zero, so it seems to be a true super-set workaround.
    //
    // There is an issue filed to reevaluate this after more field data is
    // collected. The idea was to make the most minimal change necessary
    // until more data can be collected.
    // https://github.com/bakerstu/openmrn/issues/652
#if 0
    // A small pulse in one direction then a half zero bit in the other
    // direction.
    fill_timing(DCC_RC_HALF_ZERO, 100 + 56, 56, 100 + 56, 5);
#else
    // A full zero bit inserted following the RailCom cutout.
    fill_timing(DCC_RC_HALF_ZERO, 100 << 1, 100, 100 << 1, 5);
#endif

    // At the end of the packet the resync process will happen, which means that
    // we modify the timer registers in synchronous mode instead of double
    // buffering to remove any drift that may have happened during the packet.
    // This means that we need to kick off the interval timer a bit earlier than
    // nominal to compensate for the CPU execution time. At the same time we
    // stretch the negative side of the output waveform, because the next packet
    // might be marklin. Stretching avoids outputting a short positive glitch
    // between the negative half of the last dcc bit and the fully negative
    // marklin preamble.
    fill_timing(
        DCC_EOP_ONE, (56 << 1) + 20, 56, (56 << 1) - HW::RESYNC_DELAY_USEC);

    fill_timing(MM_ZERO, 208, 26, 208, 2);
    fill_timing(MM_ONE, 208, 182, 208, 2);
    // Motorola preamble is negative DC signal.
    fill_timing(MM_PREAMBLE, 208, 0, 208);

    unsigned h_deadband = 2 * (HW::H_DEADBAND_DELAY_NSEC / 1000);
    unsigned railcom_part = 0;
    unsigned target =
        RAILCOM_CUTOUT_START_USEC + HW::RAILCOM_CUTOUT_START_DELTA_USEC;
    fill_timing(RAILCOM_CUTOUT_PRE, 56 << 1, 56, target - railcom_part);
    railcom_part = target;

    target = RAILCOM_CUTOUT_MID_USEC + HW::RAILCOM_CUTOUT_MID_DELTA_USEC;
    fill_timing(RAILCOM_CUTOUT_FIRST, 56 << 1, 56, target - railcom_part);
    railcom_part = target;

    target = RAILCOM_CUTOUT_END_USEC + HW::RAILCOM_CUTOUT_END_DELTA_USEC;
    fill_timing(RAILCOM_CUTOUT_SECOND, 56 << 1, 56, target - railcom_part);
    railcom_part = target;

    static_assert((5 * 56 * 2 - 56 + HW::RAILCOM_CUTOUT_POST_DELTA_USEC) >
            RAILCOM_CUTOUT_END_USEC + HW::RAILCOM_CUTOUT_END_DELTA_USEC,
        "railcom cutout too long");
    target = 5 * 56 * 2 - 56 + HW::RAILCOM_CUTOUT_POST_DELTA_USEC;
    unsigned remaining_high = target - railcom_part;
    // remaining time until 5 one bits are complete. For the PWM timer we have
    // some fraction of the high part, then a full low side, then we stretch
    // the low side to avoid the packet transition glitch.
    fill_timing(RAILCOM_CUTOUT_POST, remaining_high + 56 + 20, remaining_high,
        remaining_high + 56 + h_deadband +
            HW::RAILCOM_CUTOUT_POST_NEGATIVE_DELTA_USEC);

    // We need to disable the timers before making changes to the config.
    MAP_TimerDisable(HW::CCP_BASE, TIMER_A);
    MAP_TimerDisable(HW::CCP_BASE, TIMER_B);

#ifdef TIVADCC_TIVA
    MAP_TimerClockSourceSet(HW::CCP_BASE, TIMER_CLOCK_SYSTEM);
    MAP_TimerClockSourceSet(HW::INTERVAL_BASE, TIMER_CLOCK_SYSTEM);
#endif    
    MAP_TimerConfigure(HW::CCP_BASE, TIMER_CFG_SPLIT_PAIR |
                                    TIMER_CFG_A_PWM |
                                    TIMER_CFG_B_PWM);
    MAP_TimerControlStall(HW::CCP_BASE, TIMER_BOTH, true);


    // This will cause reloading the timer values only at the next period
    // instead of immediately. The PLO bit needs to be set to allow for DC
    // voltage output.
    HWREG(HW::CCP_BASE + TIMER_O_TAMR) |=
        TIMER_TAMR_TAPLO | TIMER_TAMR_TAMRSU | TIMER_TAMR_TAILD;
    HWREG(HW::CCP_BASE + TIMER_O_TBMR) |=
        TIMER_TBMR_TBPLO | TIMER_TBMR_TBMRSU | TIMER_TBMR_TBILD;

    HWREG(HW::INTERVAL_BASE + TIMER_O_TAMR) |=
        TIMER_TAMR_TAMRSU | TIMER_TAMR_TAILD;

    MAP_TimerConfigure(HW::INTERVAL_BASE, TIMER_CFG_SPLIT_PAIR |
                                    TIMER_CFG_A_PERIODIC);
    MAP_TimerControlStall(HW::INTERVAL_BASE, TIMER_A, true);

    MAP_TimerControlLevel(HW::CCP_BASE, TIMER_A, HW::PIN_H_INVERT);
    MAP_TimerControlLevel(HW::CCP_BASE, TIMER_B, !HW::PIN_L_INVERT);

    MAP_TimerLoadSet(HW::CCP_BASE, TIMER_A, timings[DCC_ONE].period);
    MAP_TimerLoadSet(HW::CCP_BASE, TIMER_B, hDeadbandDelay_);
    MAP_TimerLoadSet(HW::INTERVAL_BASE, TIMER_A, timings[DCC_ONE].period + hDeadbandDelay_ * 2);
    MAP_TimerMatchSet(HW::CCP_BASE, TIMER_A, timings[DCC_ONE].transition_a);
    MAP_TimerMatchSet(HW::CCP_BASE, TIMER_B, timings[DCC_ONE].transition_b);

    MAP_IntDisable(HW::INTERVAL_INTERRUPT);
    MAP_IntPrioritySet(HW::INTERVAL_INTERRUPT, 0x20);
    MAP_TimerIntEnable(HW::INTERVAL_BASE, TIMER_TIMA_TIMEOUT);

    MAP_TimerEnable(HW::CCP_BASE, TIMER_A);
    MAP_TimerEnable(HW::CCP_BASE, TIMER_B);
    MAP_TimerEnable(HW::INTERVAL_BASE, TIMER_A);

#ifdef TIVADCC_TIVA
    MAP_TimerSynchronize(TIMER0_BASE, HW::TIMER_SYNC);
#endif
    
    MAP_TimerLoadSet(HW::CCP_BASE, TIMER_B, timings[DCC_ONE].period);
    MAP_TimerLoadSet(
        HW::INTERVAL_BASE, TIMER_A, timings[DCC_ONE].interval_period);
    MAP_IntEnable(HW::INTERVAL_INTERRUPT);

    // The OS interrupt does not come from the hardware timer.
    MAP_TimerIntDisable(HW::CCP_BASE, 0xFFFFFFFF);
    // The OS interrupt comes under the freertos kernel.
    MAP_IntPrioritySet(HW::OS_INTERRUPT, configKERNEL_INTERRUPT_PRIORITY);
    MAP_IntEnable(HW::OS_INTERRUPT);
}

/** Read from a file or device.
 * @param file file reference for this device
 * @param buf location to place read data
 * @param count number of bytes to read
 * @return number of bytes read upon success, -1 upon failure with errno containing the cause
 */
template<class HW>
ssize_t TivaDCC<HW>::read(File *file, void *buf, size_t count)
{
    return -EINVAL;
}

/** Write to a file or device.
 * @param file file reference for this device
 * @param buf location to find write data
 * @param count number of bytes to write
 * @return number of bytes written upon success, -1 upon failure with errno containing the cause
 */
template<class HW>
__attribute__((optimize("-O3")))
ssize_t TivaDCC<HW>::write(File *file, const void *buf, size_t count)
{
    if (count != sizeof(dcc::Packet))
    {
        return -EINVAL;
    }

    OSMutexLock l(&lock_);

    if (packetQueue_.full())
    {
        return -ENOSPC;
    }

    dcc::Packet* packet = &packetQueue_.back();
    memcpy(packet, buf, count);

    // Duplicates the marklin packet if it came single.
    if (packet->packet_header.is_marklin) {
        if (packet->dlc == 3) {
            packet->dlc = 6;
            packet->payload[3] = packet->payload[0];
            packet->payload[4] = packet->payload[1];
            packet->payload[5] = packet->payload[2];
        } else {
            HASSERT(packet->dlc == 6);
        }
    }

    packetQueue_.increment_back();
    static uint8_t flip = 0;
    if (++flip >= 4)
    {
        flip = 0;
        HW::flip_led();
    }

    return count;
}

/** Request an ioctl transaction
 * @param file file reference for this device
 * @param node node reference for this device
 * @param key ioctl key
 * @param data key data
 */
template<class HW>
int TivaDCC<HW>::ioctl(File *file, unsigned long int key, unsigned long data)
{
    if (IOC_TYPE(key) == CAN_IOC_MAGIC &&
        IOC_SIZE(key) == NOTIFIABLE_TYPE &&
        key == CAN_IOC_WRITE_ACTIVE) {
        Notifiable* n = reinterpret_cast<Notifiable*>(data);
        HASSERT(n);
        // If there is no space for writing, we put the incomng notification
        // into the holder. Otherwise we notify it immediately.
        if (packetQueue_.full())
        {
            portENTER_CRITICAL();
            if (packetQueue_.full())
            {
                // We are in a critical section now. If we got into this
                // branch, then the buffer was full at the beginning of the
                // critical section. If the hardware interrupt kicks in now,
                // and sets the os_interrupt to pending, the os interrupt will
                // not happen until we leave the critical section, and thus the
                // swap will be in effect by then.
                std::swap(n, writableNotifiable_);
            }
            portEXIT_CRITICAL();
        }
        if (n) {
            n->notify();
        }
        return 0;
    }
    errno = EINVAL;
    return -1;
}

template <class HW>
__attribute__((optimize("-O3")))
inline void TivaDCC<HW>::os_interrupt_handler()
{
    if (!packetQueue_.full()) {
        Notifiable* n = writableNotifiable_;
        writableNotifiable_ = nullptr;
        if (n) n->notify_from_isr();
    }
}


#endif  // _FREERTOS_DRIVERS_TI_TIVADCC_HXX_

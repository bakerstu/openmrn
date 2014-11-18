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

#include <algorithm>
#include <cstdint>

#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "freertos/can_ioctl.h"
#include "inc/hw_memmap.h"
#include "inc/hw_timer.h"
#include "inc/hw_types.h"
#include "inc/hw_uart.h"

#include "Devtab.hxx"
#include "dcc/Packet.hxx"
#include "dcc/RailCom.hxx"
#include "executor/Notifiable.hxx"

// This structure is safe to use from an interrupt context and a regular
// context at the same time, provided that
//
// . one context uses only the front() and the other only the back() functions.
//
// . ++ and -- are compiled into atomic operations on the processor (on the
//   count_ variable).
template<class T, uint8_t SIZE> class FixedQueue {
public:
    FixedQueue()
        : rdIndex_(0)
        , wrIndex_(0)
        , count_(0)
    {
    }

    bool empty() { return count_ == 0; }
    bool full() { return count_ >= SIZE; }
    size_t size() { return count_; }

    /// Returns the head of the FIFO (next element to read).
    T& front() {
        HASSERT(!empty());
        return storage_[rdIndex_];
    }

    /// Removes the head of the FIFO from the queue.
    void increment_front() {
        HASSERT(!empty());
        if (++rdIndex_ >= SIZE) rdIndex_ = 0;
        count_--;
    }

    /// Returns the space to write the next element to.
    T& back() {
        HASSERT(!full());
        return storage_[wrIndex_];
    }

    /// Commits the element at back() into the queue.
    void increment_back() {
        HASSERT(!full());
        if (++wrIndex_ >= SIZE) wrIndex_ = 0;
        ++count_;
    }

private:
    T storage_[SIZE];
    uint8_t rdIndex_;
    uint8_t wrIndex_;
    uint8_t count_;
};


namespace dcc {
struct Feedback {
    void reset(size_t feedback_key) {
        this->feedbackKey = feedback_key;
        ch1Size = 0;
        ch2Size = 0;
    }
    void add_ch1_data(uint8_t data) {
        if (ch1Size < sizeof(ch1Data)) {
            ch1Data[ch1Size++] = data;
        }
    }
    void add_ch2_data(uint8_t data) {
        if (ch2Size < sizeof(ch2Data)) {
            ch2Data[ch2Size++] = data;
        }
    }
    uint8_t ch1Size;
    uint8_t ch1Data[2];
    uint8_t ch2Size;
    uint8_t ch2Data[6];
    uint32_t feedbackKey;
};
}

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
 */
template<class HW>
class TivaDCC : public Node
{
public:
    /** Constructor.
     * @param name name of this device instance in the file system
     */
    TivaDCC(const char *name);

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

    struct Timing {
        uint32_t period;
        uint32_t transition_a;
        uint32_t transition_b;
    };

    static bool output_enabled;

    /* WARNING: these functions (hw_init, enable_output, disable_output) MUST
     * be static, because they will be called from hw_preinit, which happens
     * before the C++ constructors have run. This means that at the time of
     * calling these functions the state of the object would be undefined /
     * uninintialized. The only safe solution is to make them static. */
    static void hw_init() {
        MAP_SysCtlPeripheralEnable(HW::CCP_PERIPH);
        MAP_SysCtlPeripheralEnable(HW::INTERVAL_PERIPH);
        MAP_SysCtlPeripheralEnable(HW::PIN_H_GPIO_PERIPH);
        MAP_SysCtlPeripheralEnable(HW::PIN_L_GPIO_PERIPH);
        MAP_SysCtlPeripheralEnable(HW::RAILCOM_TRIGGER_PERIPH);
        MAP_SysCtlPeripheralEnable(HW::RAILCOM_UART_PERIPH);
        MAP_SysCtlPeripheralEnable(HW::RAILCOM_UARTPIN_PERIPH);
        enable_output();
        disable_output();
        MAP_GPIOPinConfigure(HW::RAILCOM_UARTPIN_CONFIG);
        MAP_GPIOPinTypeUART(HW::RAILCOM_UARTPIN_BASE, HW::RAILCOM_UARTPIN_PIN);
        MAP_UARTConfigSetExpClk(
            HW::RAILCOM_UART_BASE, configCPU_CLOCK_HZ, 250000,
            UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE);
        MAP_UARTFIFOEnable(HW::RAILCOM_UART_BASE);
        // Disables the uart receive until the railcom cutout is here.
        HWREG(HW::RAILCOM_UART_BASE + UART_O_CTL) &= ~UART_CTL_RXE;
    }

    static void enable_output() {
        output_enabled = true;
        MAP_GPIOPinConfigure(HW::PIN_H_GPIO_CONFIG);
        MAP_GPIOPinConfigure(HW::PIN_L_GPIO_CONFIG);
        MAP_GPIOPinTypeTimer(HW::PIN_H_GPIO_BASE, HW::PIN_H_GPIO_PIN);
        MAP_GPIOPinTypeTimer(HW::PIN_L_GPIO_BASE, HW::PIN_L_GPIO_PIN);
    }

    static void disable_output()
    {
        output_enabled = false;
        MAP_GPIOPinWrite(HW::PIN_H_GPIO_BASE, HW::PIN_H_GPIO_PIN,
                         HW::PIN_H_INVERT ? 0xff : 0);
        MAP_GPIOPinWrite(HW::PIN_L_GPIO_BASE, HW::PIN_L_GPIO_PIN,
                         HW::PIN_L_INVERT ? 0xff : 0);
        MAP_GPIOPinWrite(HW::RAILCOM_TRIGGER_BASE,
                         HW::RAILCOM_TRIGGER_PIN,
                         HW::RAILCOM_TRIGGER_INVERT ? 0xff : 0);
        MAP_GPIOPinTypeGPIOOutput(HW::PIN_H_GPIO_BASE, HW::PIN_H_GPIO_PIN);
        MAP_GPIOPinTypeGPIOOutput(HW::PIN_L_GPIO_BASE, HW::PIN_L_GPIO_PIN);
        MAP_GPIOPinTypeGPIOOutput(HW::RAILCOM_TRIGGER_BASE,
                                  HW::RAILCOM_TRIGGER_PIN);
        MAP_GPIOPinWrite(HW::PIN_H_GPIO_BASE, HW::PIN_H_GPIO_PIN,
                         HW::PIN_H_INVERT ? 0xff : 0);
        MAP_GPIOPinWrite(HW::PIN_L_GPIO_BASE, HW::PIN_L_GPIO_PIN,
                         HW::PIN_L_INVERT ? 0xff : 0);
        MAP_GPIOPinWrite(HW::RAILCOM_TRIGGER_BASE,
                         HW::RAILCOM_TRIGGER_PIN,
                         HW::RAILCOM_TRIGGER_INVERT ? 0xff : 0);
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
     * @param node node reference for this device
     * @param key ioctl key
     * @param data key data
     */
    int ioctl(File *file, unsigned long int key, unsigned long data) OVERRIDE;

    void enable(){} /**< function to enable device */
    void disable(){} /**< function to disable device */

    /** Discards all pending buffers.  Called after disable().
     */
    void flush_buffers(){};

    /** maximum packet size we can support */
    static const size_t MAX_PKT_SIZE = 6;


    /** idle packet */
    static dcc::Packet IDLE_PKT;

    typedef enum {
        DCC_ZERO,
        DCC_ONE,
        RAILCOM_CUTOUT_PRE,
        RAILCOM_CUTOUT_FIRST,
        RAILCOM_CUTOUT_SECOND,
        RAILCOM_CUTOUT_POST,
        MM_PREAMBLE,
        MM_ZERO,
        MM_ONE,

        NUM_TIMINGS
    } BitEnum;

    int hDeadbandDelay_; /**< low->high deadband delay in clock count */
    int lDeadbandDelay_; /**< high->low deadband delay in clock count */

    Timing timings[NUM_TIMINGS];

    /** Prepares a timing entry.
     *
     * @param ofs is the bit timing that we are defining.
     * @param period_usec is the total length of the bit.
     * @param transition_usec is the time of the transition inside the bit,
     * counted from the beginning of the bit (i.e. the length of the HIGH part
     * of the period). Can be zero for DC output LOW or can be == period_usec
     * for DC output HIGH. */
    void fill_timing(BitEnum ofs, uint32_t period_usec,
                     uint32_t transition_usec);

    FixedQueue<dcc::Packet, HW::Q_SIZE> packetQueue_;
    FixedQueue<dcc::Feedback, HW::Q_SIZE> feedbackQueue_;
    Notifiable* writableNotifiable_; /**< Notify this when we have free buffers. */
    Notifiable* readableNotifiable_; /**< Notify this when we have free buffers. */

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
    enum State
    {
        PREAMBLE,
        START,
        DATA_0,
        DATA_1,
        DATA_2,
        DATA_3,
        DATA_4,
        DATA_5,
        DATA_6,
        DATA_7,
        FRAME,
        ST_MM_PREAMBLE,
        MM_DATA_0,
        MM_DATA_1,
        MM_DATA_2,
        MM_DATA_3,
        MM_DATA_4,
        MM_DATA_5,
        MM_DATA_6,
        MM_DATA_7,
        RESYNC,
        DCC_MAYBE_RAILCOM,
        DCC_CUTOUT_PRE,
        DCC_START_RAILCOM_RECEIVE,
        DCC_MIDDLE_RAILCOM_CUTOUT,
        DCC_STOP_RAILCOM_RECEIVE,
        DCC_ENABLE_AFTER_RAILCOM,
        DCC_LEADOUT,
        MM_LEADOUT,
    };

    static State state = PREAMBLE;
    static int preamble_count = 0;
    static BitEnum last_bit = DCC_ONE;
    static int count = 0;
    static int packet_repeat_count = 0;
    static const dcc::Packet *packet = &IDLE_PKT;
    static dcc::Feedback *feedback = nullptr;
    static bool resync = true;
    static bool saved_output_enabled = true; // for railcom cutout
    BitEnum current_bit;
    bool get_next_packet = false;

    MAP_TimerIntClear(HW::INTERVAL_BASE, TIMER_TIMA_TIMEOUT);

    switch (state)
    {
        default:
        case RESYNC:
            current_bit = DCC_ONE;
            if (packet->packet_header.is_marklin)
            {
                state = ST_MM_PREAMBLE;
            }
            else
            {
                state = PREAMBLE;
            }
            break;
        case PREAMBLE:
            current_bit = DCC_ONE;
            if (++preamble_count == HW::dcc_preamble_count())
            {
                state = START;
                preamble_count = 0;
            }
            break;
        case START:
            current_bit = DCC_ZERO;
            count = 0;
            state = DATA_0;
            break;
        case DATA_0:
        case DATA_1:
        case DATA_2:
        case DATA_3:
        case DATA_4:
        case DATA_5:
        case DATA_6:
        case DATA_7:
            current_bit = static_cast<BitEnum>(DCC_ZERO + ((packet->payload[count] >> (DATA_7 - state)) & 0x01));
            state = static_cast<State>(static_cast<int>(state) + 1);
            break;
        case FRAME:
            if (++count >= packet->dlc)
            {
                current_bit = DCC_ONE;  // end-of-packet bit
                state = DCC_MAYBE_RAILCOM;
                preamble_count = 0;
            }
            else
            {
                current_bit = DCC_ZERO;  // end-of-byte bit
                state = DATA_0;
            }
            break;
        case DCC_MAYBE_RAILCOM:
            if (HW::railcom_cutout() && output_enabled && feedback != nullptr) {
                //current_bit = RAILCOM_CUTOUT_PRE;
                current_bit = DCC_ONE;
                // We change the time of the next IRQ.
                MAP_TimerLoadSet(HW::INTERVAL_BASE, TIMER_A,
                                 timings[RAILCOM_CUTOUT_PRE].period);
                state = DCC_CUTOUT_PRE;
            } else {
                current_bit = DCC_ONE;
                state = DCC_LEADOUT;
            }
            break;
        case DCC_LEADOUT:
            current_bit = DCC_ONE;
            if (++preamble_count >= 2) {
                get_next_packet = true;
            }
            break;
        case DCC_CUTOUT_PRE:
            current_bit = DCC_ONE;
            // We change the time of the next IRQ.
            MAP_TimerLoadSet(HW::INTERVAL_BASE, TIMER_A,
                             timings[RAILCOM_CUTOUT_FIRST].period);
            state = DCC_START_RAILCOM_RECEIVE;
            break;
        case DCC_START_RAILCOM_RECEIVE:
            saved_output_enabled = output_enabled;
            disable_output();
            current_bit = DCC_ONE;
            // delay 1 usec
            MAP_SysCtlDelay( lDeadbandDelay_ / 3 );
            // current_bit = RAILCOM_CUTOUT_POST;
            MAP_TimerLoadSet(HW::INTERVAL_BASE, TIMER_A,
                             timings[RAILCOM_CUTOUT_SECOND].period);
            state = DCC_MIDDLE_RAILCOM_CUTOUT;
            MAP_GPIOPinWrite(HW::RAILCOM_TRIGGER_BASE,
                             HW::RAILCOM_TRIGGER_PIN,
                             HW::RAILCOM_TRIGGER_INVERT ? 0 : 0xff);
            // Waits for transient after the trigger to pass.
            MAP_SysCtlDelay( lDeadbandDelay_ / 3 );
            // Enables UART RX.
            HWREG(HW::RAILCOM_UART_BASE + UART_O_CTL) |= UART_CTL_RXE;
            break;
        case DCC_MIDDLE_RAILCOM_CUTOUT:
            while (MAP_UARTCharsAvail(HW::RAILCOM_UART_BASE))
            {
                long data = MAP_UARTCharGetNonBlocking(HW::RAILCOM_UART_BASE);
                if (data < 0) continue;
                feedback->add_ch1_data(data);
            }
            current_bit = DCC_ONE;
            MAP_TimerLoadSet(HW::INTERVAL_BASE, TIMER_A,
                             timings[RAILCOM_CUTOUT_POST].period);
            state =  DCC_STOP_RAILCOM_RECEIVE;
            break;
        case DCC_STOP_RAILCOM_RECEIVE:
            HWREG(HW::RAILCOM_UART_BASE + UART_O_CTL) &= ~UART_CTL_RXE;
            current_bit = DCC_ONE;
            state = DCC_ENABLE_AFTER_RAILCOM;
            MAP_TimerLoadSet(HW::INTERVAL_BASE, TIMER_A,
                             timings[DCC_ONE].period);
            MAP_GPIOPinWrite(HW::RAILCOM_TRIGGER_BASE,
                             HW::RAILCOM_TRIGGER_PIN,
                             HW::RAILCOM_TRIGGER_INVERT ? 0xff : 0);
            while (MAP_UARTCharsAvail(HW::RAILCOM_UART_BASE))
            {
                long data = MAP_UARTCharGetNonBlocking(HW::RAILCOM_UART_BASE);
                if (data < 0) continue;
                feedback->add_ch2_data(data);
            }
            feedback = nullptr;
            feedbackQueue_.increment_back();
            MAP_IntPendSet(HW::OS_INTERRUPT);
            break;
        case DCC_ENABLE_AFTER_RAILCOM:
            if (saved_output_enabled) {
                enable_output();
            }
            current_bit = DCC_ONE;
            state = DCC_LEADOUT;
            ++preamble_count;
            break;
        case ST_MM_PREAMBLE:
            current_bit = MM_PREAMBLE;
            ++preamble_count;
            if (preamble_count == 7 ||
                preamble_count == 7 + 6 ||
                preamble_count == 7 + 6 + 10 ||
                preamble_count == 7 + 6 + 10 + 6)
            {
                // first byte contains two bits.
                state = MM_DATA_6;
            }
            break;
        case MM_LEADOUT:
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
            current_bit = static_cast<BitEnum>(
                MM_ZERO +
                ((packet->payload[count] >> (MM_DATA_7 - state)) & 0x01));
            state = static_cast<State>(static_cast<int>(state) + 1);
            break;
        case MM_DATA_7:
            current_bit = static_cast<BitEnum>(
                MM_ZERO +
                ((packet->payload[count] >> (MM_DATA_7 - state)) & 0x01));
            ++count;
            if (count == 3) {
                state = ST_MM_PREAMBLE;
            } else if (count >= packet->dlc) {
                state = MM_LEADOUT;
            } else {
                state = MM_DATA_0;
            }
            break;
    }

    if (resync) {
        resync = false;
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
            timing->period + hDeadbandDelay_ * 2;

        HWREG(HW::CCP_BASE + TIMER_O_TBMATCHR) = timing->transition_b;  // final
        // since timer A starts later, the deadband delay cycle we set to be
        // constant off (match == period)
        HWREG(HW::CCP_BASE + TIMER_O_TAMATCHR) = hDeadbandDelay_;  // tmp
        HWREG(HW::CCP_BASE + TIMER_O_TBILR) = timing->period;  // final
        HWREG(HW::CCP_BASE + TIMER_O_TAILR) = hDeadbandDelay_;  // tmp

        // timer synchronize (if it works...)
        HWREG(TIMER0_BASE + TIMER_O_SYNC) = HW::TIMER_SYNC;

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
        MAP_TimerLoadSet(HW::INTERVAL_BASE, TIMER_A, timing->period);

        last_bit = current_bit;
    } else if (last_bit != current_bit)
    {
        auto* timing = &timings[current_bit];
        MAP_TimerLoadSet(HW::INTERVAL_BASE, TIMER_A, timing->period);
        MAP_TimerLoadSet(HW::CCP_BASE, TIMER_A, timing->period);
        MAP_TimerLoadSet(HW::CCP_BASE, TIMER_B, timing->period);
        MAP_TimerMatchSet(HW::CCP_BASE, TIMER_A, timing->transition_a);
        MAP_TimerMatchSet(HW::CCP_BASE, TIMER_B, timing->transition_b);
        last_bit = current_bit;
    }

    if (get_next_packet)
    {
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
        if (!packetQueue_.empty() && !feedbackQueue_.full())
        {
            packet = &packetQueue_.front();
            feedback = &feedbackQueue_.back();
            feedback->reset(packet->feedback_key);
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
            state = RESYNC;
        } else {
            state = ST_MM_PREAMBLE;
        }
    }
}

static uint32_t usec_to_clocks(uint32_t usec) {
    return (configCPU_CLOCK_HZ / 1000000) * usec;
}

static uint32_t nsec_to_clocks(uint32_t nsec) {
    // We have to be careful here not to underflow or overflow.
    return ((configCPU_CLOCK_HZ / 1000000) * nsec) / 1000;
}

template<class HW>
bool TivaDCC<HW>::output_enabled = false;

template<class HW>
void TivaDCC<HW>::fill_timing(BitEnum ofs, uint32_t period_usec,
                          uint32_t transition_usec)
{
    auto* timing = &timings[ofs];
    timing->period = usec_to_clocks(period_usec);
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
}

template<class HW>
dcc::Packet TivaDCC<HW>::IDLE_PKT = dcc::Packet::DCC_IDLE();


template<class HW>
TivaDCC<HW>::TivaDCC(const char *name)
    : Node(name)
    , hDeadbandDelay_(nsec_to_clocks(HW::H_DEADBAND_DELAY_NSEC))
    , lDeadbandDelay_(nsec_to_clocks(HW::L_DEADBAND_DELAY_NSEC))
    , writableNotifiable_(nullptr)
    , readableNotifiable_(nullptr)
{
    fill_timing(DCC_ZERO, 105<<1, 105);
    fill_timing(DCC_ONE, 56<<1, 56);
    fill_timing(MM_ZERO, 208, 26);
    fill_timing(MM_ONE, 208, 182);
    // Motorola preamble is negative DC signal.
    fill_timing(MM_PREAMBLE, 208, 0);

    unsigned h_deadband = 2 * (HW::H_DEADBAND_DELAY_NSEC / 1000);
    unsigned railcom_part = 0;
    fill_timing(RAILCOM_CUTOUT_PRE, 6 - railcom_part, 0);
    railcom_part = 6;
    fill_timing(RAILCOM_CUTOUT_FIRST, 210 - railcom_part, 0);
    railcom_part = 210;
    fill_timing(RAILCOM_CUTOUT_SECOND, 471 - railcom_part, 0);
    railcom_part = 471;
    // remaining time
    fill_timing(RAILCOM_CUTOUT_POST, 5*56*2 - railcom_part + h_deadband, 0);

    // We need to disable the timers before making changes to the config.
    MAP_TimerDisable(HW::CCP_BASE, TIMER_A);
    MAP_TimerDisable(HW::CCP_BASE, TIMER_B);

    MAP_TimerClockSourceSet(HW::CCP_BASE, TIMER_CLOCK_SYSTEM);
    MAP_TimerClockSourceSet(HW::INTERVAL_BASE, TIMER_CLOCK_SYSTEM);
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
    MAP_IntPrioritySet(HW::INTERVAL_INTERRUPT, 0);
    MAP_TimerIntEnable(HW::INTERVAL_BASE, TIMER_TIMA_TIMEOUT);

    MAP_TimerEnable(HW::CCP_BASE, TIMER_A);
    MAP_TimerEnable(HW::CCP_BASE, TIMER_B);
    MAP_TimerEnable(HW::INTERVAL_BASE, TIMER_A);

    MAP_TimerSynchronize(TIMER0_BASE, TIMER_0A_SYNC | TIMER_0B_SYNC | TIMER_1A_SYNC | TIMER_1B_SYNC);

    MAP_TimerLoadSet(HW::CCP_BASE, TIMER_B, timings[DCC_ONE].period);
    MAP_TimerLoadSet(HW::INTERVAL_BASE, TIMER_A, timings[DCC_ONE].period);
    MAP_IntEnable(HW::INTERVAL_INTERRUPT);

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
    if (count != sizeof(dcc::Feedback))
    {
        return -EINVAL;
    }
    // We only need this critical section to prevent concurrent threads from
    // reading at the same time.
    portENTER_CRITICAL();
    if (feedbackQueue_.empty()) {
        portEXIT_CRITICAL();
        return -EAGAIN;
    }
    memcpy(buf, &feedbackQueue_.front(), count);
    feedbackQueue_.increment_front();
    portEXIT_CRITICAL();
    return count;
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

    portENTER_CRITICAL();
    // TODO(balazs.racz) this interrupt disable is not actually needed. Writing
    // to the back of the queue should be okay while the interrupt reads from
    // the front of it.
    MAP_TimerIntDisable(HW::INTERVAL_BASE, TIMER_TIMA_TIMEOUT);

    if (packetQueue_.full())
    {
        MAP_TimerIntEnable(HW::INTERVAL_BASE, TIMER_TIMA_TIMEOUT);
        portEXIT_CRITICAL();
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

    MAP_TimerIntEnable(HW::INTERVAL_BASE, TIMER_TIMA_TIMEOUT);
    portEXIT_CRITICAL();
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
                swap(n, writableNotifiable_);
            }
            portEXIT_CRITICAL();
        }
        if (n) {
            n->notify();
        }
        return 0;
    }
    if (IOC_TYPE(key) == CAN_IOC_MAGIC &&
        IOC_SIZE(key) == NOTIFIABLE_TYPE &&
        key == CAN_IOC_READ_ACTIVE) {
        Notifiable* n = reinterpret_cast<Notifiable*>(data);
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
                // critical section. If the hardware interrupt kicks in now,
                // and sets the os_interrupt to pending, the os interrupt will
                // not happen until we leave the critical section, and thus the
                // swap will be in effect by then.
                swap(n, readableNotifiable_);
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
    if (!feedbackQueue_.empty() > 0) {
        Notifiable* n = readableNotifiable_;
        readableNotifiable_ = nullptr;
        if (n) n->notify_from_isr();
    }
}


#endif  // _FREERTOS_DRIVERS_TI_TIVADCC_HXX_

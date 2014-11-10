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
#include "freertos/can_ioctl.h"
#include "inc/hw_memmap.h"
#include "inc/hw_timer.h"
#include "inc/hw_types.h"

#include "Devtab.hxx"
#include "dcc/Packet.hxx"
#include "executor/Notifiable.hxx"

/** A device driver for sending DCC packets.  If the packet queue is empty,
 *  then the device driver automatically sends out idle DCC packets.  The
 *  device driver uses two instances of the 16/32-bit timer pairs.  The user
 *  is responsible for providing interrupt entry point for the interval timer
 *  and calling the inline method @ref interrupt_handler on behalf of this
 *  device driver.
 *
 *  Write calls work by sending the packet in the format of dcc::Packet
 *  including the X-OR linkage byte.  Only one DCC packet may be written per
 *  call to the write method.  If there is no space currently available in the
 *  write queue, the write method will return -1 with errno set to ENOSPC.
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
        MAP_GPIOPinConfigure(HW::PIN_H_GPIO_CONFIG);
        MAP_GPIOPinConfigure(HW::PIN_L_GPIO_CONFIG);
        MAP_GPIOPinTypeTimer(HW::PIN_H_GPIO_BASE, HW::PIN_H_GPIO_PIN);
        MAP_GPIOPinTypeTimer(HW::PIN_L_GPIO_BASE, HW::PIN_L_GPIO_PIN);
        disable_output();
    }

    static void enable_output() {
        MAP_GPIOPinTypeTimer(HW::PIN_H_GPIO_BASE, HW::PIN_H_GPIO_PIN);
        MAP_GPIOPinTypeTimer(HW::PIN_L_GPIO_BASE, HW::PIN_L_GPIO_PIN);
        MAP_GPIOPinConfigure(HW::PIN_H_GPIO_CONFIG);
        MAP_GPIOPinConfigure(HW::PIN_L_GPIO_CONFIG);
    }

    static void disable_output()
    {
        MAP_GPIOPinWrite(HW::PIN_H_GPIO_BASE, HW::PIN_H_GPIO_PIN,
                         HW::PIN_H_INVERT ? 0xff : 0);
        MAP_GPIOPinWrite(HW::PIN_H_GPIO_BASE, HW::PIN_H_GPIO_PIN,
                         HW::PIN_H_INVERT ? 0xff : 0);
        MAP_GPIOPinTypeGPIOOutput(HW::PIN_H_GPIO_BASE, HW::PIN_H_GPIO_PIN);
        MAP_GPIOPinTypeGPIOOutput(HW::PIN_L_GPIO_BASE, HW::PIN_L_GPIO_PIN);
        MAP_GPIOPinWrite(HW::PIN_H_GPIO_BASE, HW::PIN_H_GPIO_PIN,
                         HW::PIN_H_INVERT ? 0xff : 0);
        MAP_GPIOPinWrite(HW::PIN_H_GPIO_BASE, HW::PIN_H_GPIO_PIN,
                         HW::PIN_H_INVERT ? 0xff : 0);
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

    /** Queue structure for holding outgoing DCC packets.
     */
    struct Q
    {
        size_t count; /**< number of items in the queue */
        size_t rdIndex; /**< current read index */
        size_t wrIndex; /**< current write index */
        dcc::Packet data[HW::Q_SIZE]; /**< queue data */
    };

    int oneBitPeriod; /**< period of one bit in clock count */
    int zeroBitPeriod; /**< period of zero bit in clock count */
    int hDeadbandDelay; /**< low->high deadband delay in clock count */
    int lDeadbandDelay; /**< high->low deadband delay in clock count */
    Notifiable* writableNotifiable; /**< Notify this when we have free buffers. */

    /** idle packet */
    static dcc::Packet IDLE_PKT;

    typedef enum {
        DCC_ZERO,
        DCC_ONE,
        MM_PREAMBLE,
        MM_ZERO,
        MM_ONE,

        NUM_TIMINGS
    } BitEnum;

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

    Q q; /**< DCC packet queue */

    /** Default constructor.
     */
    TivaDCC();

    DISALLOW_COPY_AND_ASSIGN(TivaDCC);
};

template <class HW>
__attribute__((optimize("-O3")))
inline void TivaDCC<HW>::os_interrupt_handler()
{
    HASSERT(writableNotifiable);
    Notifiable* n = writableNotifiable;
    writableNotifiable = nullptr;
    MAP_IntDisable(HW::OS_INTERRUPT);
    n->notify_from_isr();
}

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
        DCC_LEADOUT,
        MM_LEADOUT,
    };

    static State state = PREAMBLE;
    static int preamble_count = 0;
    static BitEnum last_bit = DCC_ONE;
    static int count = 0;
    static int packet_repeat_count = 0;
    static const dcc::Packet *packet = &IDLE_PKT;
    static bool resync = true;
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
                state = DCC_LEADOUT;
                preamble_count = 0;
            }
            else
            {
                current_bit = DCC_ZERO;  // end-of-byte bit
                state = DATA_0;
            }
            break;
        case DCC_LEADOUT:
            current_bit = DCC_ONE;
            if (++preamble_count >= 2) {
                get_next_packet = true;
            }
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
        HWREG(HW::CCP_BASE + TIMER_O_TAILR) = timing->period;
        HWREG(HW::CCP_BASE + TIMER_O_TBILR) = hDeadbandDelay;
        HWREG(HW::INTERVAL_BASE + TIMER_O_TAILR) = timing->period + hDeadbandDelay * 2;

        // This is already final.
        MAP_TimerMatchSet(HW::CCP_BASE, TIMER_A, timing->transition_a);
        // since timer B starts later, the deadband delay cycle we set to be constant off.
        MAP_TimerMatchSet(HW::CCP_BASE, TIMER_B, hDeadbandDelay);
        // TODO: this should be parametrized by the timer numbers that we are
        // using.
        MAP_TimerSynchronize(TIMER0_BASE, HW::TIMER_SYNC);

        // Switches back to asynch timer update.
        HWREG(HW::CCP_BASE + TIMER_O_TAMR) |=
            (TIMER_TAMR_TAMRSU | TIMER_TAMR_TAILD);
        HWREG(HW::CCP_BASE + TIMER_O_TBMR) |=
            (TIMER_TBMR_TBMRSU | TIMER_TBMR_TBILD);
        HWREG(HW::INTERVAL_BASE + TIMER_O_TAMR) |=
            (TIMER_TAMR_TAMRSU | TIMER_TAMR_TAILD);

        // Sets final values for the cycle.
        MAP_TimerLoadSet(HW::CCP_BASE, TIMER_B, timing->period);
        MAP_TimerMatchSet(HW::CCP_BASE, TIMER_B, timing->transition_b);
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
            --q.count;
            ++q.rdIndex;
            if (q.rdIndex == HW::Q_SIZE)
            {
                q.rdIndex = 0;
            }
            // Notifies the OS that we can write to the buffer.
            MAP_IntPendSet(HW::OS_INTERRUPT);
            resync = true;
        } else {
        }
        if (q.count)
        {
            packet = &q.data[q.rdIndex];
        }
        else
        {
            packet = &IDLE_PKT;
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
            nominal_transition + (hDeadbandDelay + lDeadbandDelay) / 2;
        timing->transition_b =
            nominal_transition - (hDeadbandDelay + lDeadbandDelay) / 2;
    }
}

template<class HW>
dcc::Packet TivaDCC<HW>::IDLE_PKT = dcc::Packet::DCC_IDLE();


template<class HW>
TivaDCC<HW>::TivaDCC(const char *name)
    : Node(name)
    , hDeadbandDelay(nsec_to_clocks(HW::H_DEADBAND_DELAY_NSEC))
    , lDeadbandDelay(nsec_to_clocks(HW::L_DEADBAND_DELAY_NSEC))
    , writableNotifiable(nullptr)
{
    q.count = 0;
    q.rdIndex = 0;
    q.wrIndex = 0;

    fill_timing(DCC_ZERO, 105<<1, 105);
    fill_timing(DCC_ONE, 56<<1, 56);
    fill_timing(MM_ZERO, 208, 26);
    fill_timing(MM_ONE, 208, 182);
    // Motorola preamble is negative DC signal.
    fill_timing(MM_PREAMBLE, 208, 0);

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
    MAP_TimerLoadSet(HW::CCP_BASE, TIMER_B, hDeadbandDelay);
    MAP_TimerLoadSet(HW::INTERVAL_BASE, TIMER_A, timings[DCC_ONE].period + hDeadbandDelay * 2);
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

    // We don't have a write notifiable pointer at the moment.
    MAP_IntDisable(HW::OS_INTERRUPT);
    MAP_IntPrioritySet(HW::OS_INTERRUPT, configKERNEL_INTERRUPT_PRIORITY);
    // but we have free space in the queue at boot time.
    MAP_IntPendSet(HW::OS_INTERRUPT);
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
    errno = EINVAL;
    return -1;
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
    portENTER_CRITICAL();
    MAP_TimerIntDisable(HW::INTERVAL_BASE, TIMER_TIMA_TIMEOUT);

    if (q.count == HW::Q_SIZE)
    {
        MAP_IntPendClear(HW::OS_INTERRUPT);
        MAP_TimerIntEnable(HW::INTERVAL_BASE, TIMER_TIMA_TIMEOUT);
        portEXIT_CRITICAL();
        return -ENOSPC;
    }
    if (count > sizeof(dcc::Packet) || count < 2U ||
        count != (((const dcc::Packet *)buf)->dlc + 2U))
    {
        MAP_TimerIntEnable(HW::INTERVAL_BASE, TIMER_TIMA_TIMEOUT);
        portEXIT_CRITICAL();
        return -EINVAL;
    }

    memcpy(&q.data[q.wrIndex], buf, count);

    dcc::Packet* packet = &q.data[q.wrIndex];
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

    if (++q.wrIndex == HW::Q_SIZE)
    {
        q.wrIndex = 0;
        HW::flip_led();
    }

    ++q.count;
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
        if (q.count == HW::Q_SIZE)
        {
            portENTER_CRITICAL();
            swap(n, writableNotifiable);
            MAP_IntEnable(HW::OS_INTERRUPT);
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


#endif  // _FREERTOS_DRIVERS_TI_TIVADCC_HXX_

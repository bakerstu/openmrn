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

#include <cstdint>

#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"

#include "Devtab.hxx"
#include "executor/Notifiable.hxx"
#include "dcc/Packet.hxx"

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
class TivaDCC : public Node
{
public:
    /** Constructor.
     * @param name name of this device instance in the file system
     * @param ccp_base base address of a capture compare pwm timer pair
     * @param interval_base base address of an interval timer
     * @param interrupt interrupt number of interval timer
     * @param os_interrupt an otherwise unused interrupt number (cound be that of the capture compare pwm timer)
     * @param preamble_count number of preamble bits to send exclusive of
     *        end of packet '1' bit
     * @param h_deadband_delay_nsec is the time (in nanoseconds) to wait
     * between turning off the low driver and turning on the high driver.
     * @param l_deadband_delay_nsec is the time (in nanoseconds) to wait
     * between turning off the high driver and turning on the low driver.
     * @param railcom_cutout true to produce the RailCom cuttout, else false
     */
    TivaDCC(const char *name,
            unsigned long ccp_base,
            unsigned long interval_base,
            uint32_t interrupt,
            uint32_t os_interrupt,
            uint8_t* led_ptr,
            int preamble_count,
            int h_deadband_delay_nsec,
            int l_deadband_delay_nsec,
            bool railcom_cutout = false);

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

    /** number of outgoing messages we can queue */
    static const size_t Q_SIZE = 4;

    /** maximum packet size we can support */
    static const size_t MAX_PKT_SIZE = 6;

    /** Queue structure for holding outgoing DCC packets.
     */
    struct Q
    {
        size_t count; /**< number of items in the queue */
        size_t rdIndex; /**< current read index */
        size_t wrIndex; /**< current write index */
        dcc::Packet data[Q_SIZE]; /**< queue data */
    };

    unsigned long ccpBase; /**< capture compare pwm base address */
    unsigned long intervalBase; /**< interval timer base address */
    uint32_t interrupt; /**< interrupt number of interval timer */
    int preambleCount; /**< number of preamble bits to send */
    int oneBitPeriod; /**< period of one bit */
    int zeroBitPeriod; /**< period of zero bit */
    int hDeadbandDelay; /**< low->high deadband delay */
    int lDeadbandDelay; /**< high->low deadband delay */
    bool railcomCutout; /**< true if we should produce the RailCom cuttout */
    uint32_t osInterrupt; /**< interrupt used to notify FreeRTOS. */
    Notifiable* writableNotifiable; /**< Notify this when we have free buffers. */
    uint8_t* ledPtr; /**< Will be toggled between 0 and 0xff as packets sent.*/

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

__attribute__((optimize("-O3")))
inline void TivaDCC::os_interrupt_handler()
{
    HASSERT(writableNotifiable);
    Notifiable* n = writableNotifiable;
    writableNotifiable = nullptr;
    MAP_IntDisable(osInterrupt);
    n->notify_from_isr();
}

/** Handle an interrupt.
 */
__attribute__((optimize("-O3")))
inline void TivaDCC::interrupt_handler()
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
    };

    static State state = PREAMBLE;
    static int preamble_count = 0;
    static BitEnum last_bit = DCC_ONE;
    static int count = 0;
    static const dcc::Packet *packet = &IDLE_PKT;
    BitEnum current_bit;
    bool get_next_packet = false;

    MAP_TimerIntClear(intervalBase, TIMER_TIMA_TIMEOUT);

    switch (state)
    {
        default:
        case PREAMBLE:
            current_bit = DCC_ONE;
            if (++preamble_count == preambleCount)
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
                get_next_packet = true;
            }
            else
            {
                current_bit = DCC_ZERO;  // end-of-byte bit
                state = DATA_0;
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
                count = 0;
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
            if (++count >= packet->dlc) {
                count = 0;
                // see if we need retransmission
                if (preamble_count >= 7 + 6 + 10 + 6) {
                    get_next_packet = true;
                } else {
                    state = ST_MM_PREAMBLE;
                }
            } else {
                // TODO(bracz) reenable this
                //state = MM_DATA_0;
            }
            break;
    }

    if (last_bit != current_bit)
    {
        auto* timing = &timings[current_bit];
        MAP_TimerLoadSet(intervalBase, TIMER_A, timing->period);
        MAP_TimerLoadSet(ccpBase, TIMER_A, timing->period);
        MAP_TimerLoadSet(ccpBase, TIMER_B, timing->period);
        MAP_TimerMatchSet(ccpBase, TIMER_A, timing->transition_a);
        MAP_TimerMatchSet(ccpBase, TIMER_B, timing->transition_b);
        last_bit = current_bit;
    }

    if (get_next_packet)
    {
        if (packet != &IDLE_PKT)
        {
            --q.count;
            ++q.rdIndex;
            if (q.rdIndex == Q_SIZE)
            {
                q.rdIndex = 0;
            }
            // Notifies the OS that we can write to the buffer.
            MAP_IntPendSet(osInterrupt);
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
        if (packet->packet_header.is_marklin)
        {
            state = ST_MM_PREAMBLE;
        }
        else
        {
            state = PREAMBLE;
        }
    }
}

#endif  // _FREERTOS_DRIVERS_TI_TIVADCC_HXX_

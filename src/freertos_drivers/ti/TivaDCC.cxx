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
 * \file TivaDCC.cxx
 * This file implements a DCC packet device driver layer specific to Tivaware.
 *
 * @author Stuart W. Baker
 * @date 25 July 2014
 */

#include "TivaDCC.hxx"

#include <algorithm>

#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "freertos/can_ioctl.h"

#include "inc/hw_memmap.h"
#include "inc/hw_timer.h"
#include "inc/hw_types.h"


dcc::Packet TivaDCC::IDLE_PKT = dcc::Packet::DCC_IDLE();

static uint32_t usec_to_clocks(uint32_t usec) {
    return (configCPU_CLOCK_HZ / 1000000) * usec;
}

static uint32_t nsec_to_clocks(uint32_t nsec) {
    // We have to be careful here not to underflow or overflow.
    return ((configCPU_CLOCK_HZ / 1000000) * nsec) / 1000;
}

TivaDCC::TivaDCC(const char *name,
                 unsigned long ccp_base,
                 unsigned long interval_base,
                 uint32_t interrupt,
                 uint32_t os_interrupt,
                 uint8_t* led_ptr,
                 int preamble_count,
                 int h_deadband_delay_nsec,
                 int l_deadband_delay_nsec,
                 bool railcom_cutout)
    : Node(name)
    , ccpBase(ccp_base)
    , intervalBase(interval_base)
    , preambleCount(preamble_count)
    , hDeadbandDelay(nsec_to_clocks(h_deadband_delay_nsec))
    , lDeadbandDelay(nsec_to_clocks(l_deadband_delay_nsec))
    , railcomCutout(railcom_cutout)
    , osInterrupt(os_interrupt)
    , writableNotifiable(nullptr)
    , ledPtr(led_ptr)
{
    q.count = 0;
    q.rdIndex = 0;
    q.wrIndex = 0;

    fill_timing(DCC_ZERO, 100<<1, 100);
    fill_timing(DCC_ONE, 56<<1, 56);
    fill_timing(MM_ZERO, 208, 26);
    fill_timing(MM_ONE, 208, 182);
    // Motorola preamble is negative DC signal.
    fill_timing(MM_PREAMBLE, 208, 0);

    // We need to disable the timers before making changes to the config.
    MAP_TimerDisable(ccpBase, TIMER_A);
    MAP_TimerDisable(ccpBase, TIMER_B);

    MAP_TimerClockSourceSet(ccpBase, TIMER_CLOCK_SYSTEM);
    MAP_TimerClockSourceSet(intervalBase, TIMER_CLOCK_SYSTEM);
    MAP_TimerConfigure(ccpBase, TIMER_CFG_SPLIT_PAIR |
                                    TIMER_CFG_A_PWM |
                                    TIMER_CFG_B_PWM);
    MAP_TimerControlStall(ccpBase, TIMER_BOTH, true);


    // This will cause reloading the timer values only at the next period
    // instead of immediately. The PLO bit needs to be set to allow for DC
    // voltage output.
    HWREG(ccpBase + TIMER_O_TAMR) |=
        TIMER_TAMR_TAPLO | TIMER_TAMR_TAMRSU | TIMER_TAMR_TAILD;
    HWREG(ccpBase + TIMER_O_TBMR) |=
        TIMER_TBMR_TBPLO | TIMER_TBMR_TBMRSU | TIMER_TBMR_TBILD;

    HWREG(intervalBase + TIMER_O_TAMR) |=
        TIMER_TAMR_TAMRSU | TIMER_TAMR_TAILD;

    MAP_TimerConfigure(intervalBase, TIMER_CFG_SPLIT_PAIR |
                                    TIMER_CFG_A_PERIODIC);
    MAP_TimerControlStall(intervalBase, TIMER_A, true);

    MAP_TimerControlLevel(ccpBase, TIMER_A, /*true*/ false);
    MAP_TimerControlLevel(ccpBase, TIMER_B, false);

    MAP_TimerLoadSet(ccpBase, TIMER_A, timings[DCC_ONE].period);
    MAP_TimerLoadSet(ccpBase, TIMER_B, hDeadbandDelay);
    MAP_TimerLoadSet(intervalBase, TIMER_A, timings[DCC_ONE].period + hDeadbandDelay * 2);
    MAP_TimerMatchSet(ccpBase, TIMER_A, timings[DCC_ONE].transition_a);
    MAP_TimerMatchSet(ccpBase, TIMER_B, timings[DCC_ONE].transition_b);

    MAP_IntDisable(interrupt);
    MAP_IntPrioritySet(interrupt, 0);
    MAP_TimerIntEnable(intervalBase, TIMER_TIMA_TIMEOUT);

    MAP_TimerEnable(ccpBase, TIMER_A);
    MAP_TimerEnable(ccpBase, TIMER_B);
    MAP_TimerEnable(intervalBase, TIMER_A);

    MAP_TimerSynchronize(TIMER0_BASE, TIMER_0A_SYNC | TIMER_0B_SYNC | TIMER_1A_SYNC | TIMER_1B_SYNC);

    MAP_TimerLoadSet(ccpBase, TIMER_B, timings[DCC_ONE].period);
    MAP_TimerLoadSet(intervalBase, TIMER_A, timings[DCC_ONE].period);
    MAP_IntEnable(interrupt);

    // We don't have a write notifiable at the moment.
    MAP_IntDisable(osInterrupt);
    MAP_IntPrioritySet(osInterrupt, configKERNEL_INTERRUPT_PRIORITY);
    // but we have free space in the queue at boot time.
    MAP_IntPendSet(osInterrupt);
}

void TivaDCC::fill_timing(BitEnum ofs, uint32_t period_usec,
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
            nominal_transition - (hDeadbandDelay + lDeadbandDelay) / 2;
        timing->transition_b =
            nominal_transition + (hDeadbandDelay + lDeadbandDelay) / 2;
    }
}

/** Read from a file or device.
 * @param file file reference for this device
 * @param buf location to place read data
 * @param count number of bytes to read
 * @return number of bytes read upon success, -1 upon failure with errno containing the cause
 */
ssize_t TivaDCC::read(File *file, void *buf, size_t count)
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
__attribute__((optimize("-O3")))
ssize_t TivaDCC::write(File *file, const void *buf, size_t count)
{
    portENTER_CRITICAL();
    MAP_TimerIntDisable(intervalBase, TIMER_TIMA_TIMEOUT);

    if (q.count == Q_SIZE)
    {
        MAP_IntPendClear(osInterrupt);
        MAP_TimerIntEnable(intervalBase, TIMER_TIMA_TIMEOUT);
        portEXIT_CRITICAL();
        return -ENOSPC;
    }
    if (count > sizeof(dcc::Packet) || count < 2U ||
        count != (((const dcc::Packet *)buf)->dlc + 2U))
    {
        MAP_TimerIntEnable(intervalBase, TIMER_TIMA_TIMEOUT);
        portEXIT_CRITICAL();
        return -EINVAL;
    }

    memcpy(&q.data[q.wrIndex], buf, count);

    if (++q.wrIndex == Q_SIZE)
    {
        q.wrIndex = 0;
        if (ledPtr)
        {
            static uint8_t flip = 0xff;
            flip = ~flip;
            *ledPtr = flip;
        }
    }

    ++q.count;
    MAP_TimerIntEnable(intervalBase, TIMER_TIMA_TIMEOUT);
    portEXIT_CRITICAL();
    return count;
}

/** Request an ioctl transaction
 * @param file file reference for this device
 * @param node node reference for this device
 * @param key ioctl key
 * @param data key data
 */
int TivaDCC::ioctl(File *file, unsigned long int key, unsigned long data)
{
    if (IOC_TYPE(key) == CAN_IOC_MAGIC &&
        IOC_SIZE(key) == NOTIFIABLE_TYPE &&
        key == CAN_IOC_WRITE_ACTIVE) {
        Notifiable* n = reinterpret_cast<Notifiable*>(data);
        HASSERT(n);
        if (q.count == Q_SIZE)
        {
            portENTER_CRITICAL();
            swap(n, writableNotifiable);
            MAP_IntEnable(osInterrupt);
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

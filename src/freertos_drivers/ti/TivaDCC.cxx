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

#include "TivaDev.hxx"

#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"

const uint8_t TivaDCC::IDLE_PKT[3] = {0x02, 0xFF, 0x00};

/** Constructor.
 * @param name name of this device instance in the file system
 * @param ccp_base base address of a capture compare pwm timer pair
 * @param interval_base base address of an interval timer 
 * @param interrupt interrupt number of interval timer
 * @param preamble_count number of preamble bits to send exclusive of 
 *        end of packet '1' bit
 * @param one_bit_period number of system clock cycles for a one bit
 * @param zero_bit_period number of system clock cycles for a one bit
 * @param startup_delay offset for introducing some deadband at startup
 * @param deadband_adjust ajustment factor for adding deadband
 * @param railcom_cuttout true to produce the RailCom cuttout, else false
 */
TivaDCC::TivaDCC(const char *name,
                 unsigned long ccp_base,
                 unsigned long interval_base,
                 uint32_t interrupt,
                 int preamble_count,
                 int one_bit_period,
                 int zero_bit_period,
                 int startup_delay,
                 int deadband_adjust,
                 bool railcom_cuttout)
    : Node(name)
    , ccpBase(ccp_base)
    , intervalBase(interval_base)
    , preambleCount(preamble_count)
    , oneBitPeriod(one_bit_period)
    , zeroBitPeriod(zero_bit_period)
    , startupDelay(startup_delay)
    , deadbandAdjust(deadband_adjust >> 1)
    , railcomCuttout(railcom_cuttout)
{
    q.count = 0;
    q.rdIndex = 0;
    q.wrIndex = 0;

    MAP_TimerClockSourceSet(ccpBase, TIMER_CLOCK_SYSTEM);
    MAP_TimerClockSourceSet(intervalBase, TIMER_CLOCK_SYSTEM);
    MAP_TimerConfigure(ccpBase, TIMER_CFG_SPLIT_PAIR |
                                    TIMER_CFG_A_PWM |
                                    TIMER_CFG_B_PWM);
    MAP_TimerConfigure(intervalBase, TIMER_CFG_SPLIT_PAIR |
                                    TIMER_CFG_A_PERIODIC);
    MAP_TimerControlLevel(ccpBase, TIMER_A, true);

    MAP_TimerLoadSet(ccpBase, TIMER_A, oneBitPeriod);
    MAP_TimerLoadSet(ccpBase, TIMER_B, oneBitPeriod);
    MAP_TimerLoadSet(intervalBase, TIMER_A, oneBitPeriod);
    MAP_TimerMatchSet(ccpBase, TIMER_A, (oneBitPeriod >> 1) - deadbandAdjust);
    MAP_TimerMatchSet(ccpBase, TIMER_B, (oneBitPeriod >> 1) + deadbandAdjust);

    MAP_IntEnable(interrupt);
    MAP_IntPrioritySet(interrupt, 0);
    MAP_TimerIntEnable(intervalBase, TIMER_TIMA_TIMEOUT);

    MAP_TimerEnable(ccpBase, TIMER_A);
    MAP_SysCtlDelay(startupDelay);
    MAP_TimerEnable(ccpBase, TIMER_B);
    MAP_TimerEnable(intervalBase, TIMER_A);
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
ssize_t TivaDCC::write(File *file, const void *buf, size_t count)
{
    MAP_TimerIntDisable(intervalBase, TIMER_TIMA_TIMEOUT);

    if (q.count == Q_SIZE)
    {
        errno = ENOSPC;
        MAP_TimerIntEnable(intervalBase, TIMER_TIMA_TIMEOUT);
        return -1;
    }
    if (count > MAX_PKT_SIZE)
    {
        errno = EINVAL;
        MAP_TimerIntEnable(intervalBase, TIMER_TIMA_TIMEOUT);
        return -1;
    }

    q.data[q.wrIndex][0] = count;
    memcpy(&q.data[q.wrIndex][1], buf, count);

    if (++q.wrIndex == Q_SIZE)
    {
        q.wrIndex = 0;
    }

    ++q.count;

    MAP_TimerIntEnable(intervalBase, TIMER_TIMA_TIMEOUT);
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
    errno = EINVAL;
    return -1;
}


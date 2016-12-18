/** \copyright
 * Copyright (c) 2015, Balazs Racz
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
 * \file CpuLoad.hxx
 * Class for maintining a CPU load indication.
 *
 * @author Balazs Racz
 * @date 30 August 2015
 */

#include "freertos_drivers/common/CpuLoad.hxx"

#include "os/os.h"
#include "task.h"

extern "C" {

/// The bits to shift to get multiples of 1.0
static constexpr uint32_t SHIFT_ONE = 24;

/// The bits to shift after a rate inclusing
static constexpr uint32_t SHIFT_RATE = 8;
/// The multiplication for the rate
static constexpr uint8_t AVG_RATE = 0xff;
/// If the last measurement was busy, we add this much weight
static constexpr uint32_t ADD_RATE = 0x1 << 24;

void CpuLoad::record_value(bool busy) {
    avg_ *= AVG_RATE;
    if (busy) {
        avg_ += ADD_RATE;
    }
    avg_ >>= SHIFT_RATE;
}

uint8_t CpuLoad::get_load() {
    return (avg_ * 100) >> SHIFT_ONE;
}

void cpuload_tick(void)
{
    if (!Singleton<CpuLoad>::exists())
        return;
    bool is_idle = xTaskGetIdleTaskHandle() == xTaskGetCurrentTaskHandle();
    if (is_idle)
    {
        Singleton<CpuLoad>::instance()->record_value(0);
    }
    else
    {
        Singleton<CpuLoad>::instance()->record_value(100);
    }
}
}

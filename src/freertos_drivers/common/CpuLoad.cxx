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


#include "CpuLoad.hxx"

#ifdef OPENMRN_FEATURE_THREAD_FREERTOS

#include "os/os.h"
#include "freertos_includes.h"

#ifdef ESP_PLATFORM
#include "sdkconfig.h"
#include <esp_idf_version.h>
#endif // ESP_PLATFORM

extern "C"
{

/// The bits to shift to get multiples of 1.0
static constexpr uint32_t SHIFT_ONE = 24;

/// The bits to shift after a rate inclusing
static constexpr uint32_t SHIFT_RATE = 8;
/// The multiplication for the rate
static constexpr uint8_t AVG_RATE = 0xff;
/// If the last measurement was busy, we add this much weight
static constexpr uint32_t ADD_RATE = 0x1 << 24;

void CpuLoad::record_value(bool busy, uintptr_t key)
{
    avg_ *= AVG_RATE;
    ++countSinceUpdate_;
    if (busy)
    {
        avg_ += ADD_RATE;
    }
    avg_ >>= SHIFT_RATE;
    // Check streak
    if (busy)
    {
        if (consecutive_ < 255)
        {
            ++consecutive_;
        }
        if (consecutive_ > maxConsecutive_)
        {
            maxConsecutive_ = consecutive_;
        }
    }
    else
    {
        consecutive_ = 0;
    }
    // Check window of 16.
    last16Bits_ <<= 1;
    last16Bits_ |= (busy ? 1 : 0);
    // sums up last 16 bits.
    unsigned v = last16Bits_;
    v = (v & 0x5555) + ((v & 0xaaaa) >> 1);
    v = (v & 0x3333) + ((v & 0xcccc) >> 2);
    v = (v & 0x0F0F) + ((v & 0xF0F0) >> 4);
    v = (v & 0x00FF) + ((v & 0xFF00) >> 8);
    if (v > peakOver16Counts_)
    {
        peakOver16Counts_ = v;
    }
    // Record per-key information
    if (key == 0)
        return;
    bool found = false;
    for (auto it = perKeyCost_.begin(); it != perKeyCost_.end(); ++it)
    {
        if (it->key == key)
        {
            found = true;
            ++it->rolling_count;
            break;
        }
    }
    if (!found && newKey_ == 0)
    {
        newKey_ = key;
    }
}

uint8_t CpuLoad::get_load() {
    return (avg_ * 100) >> SHIFT_ONE;
}

#ifdef ESP_PLATFORM
// In ESP-IDF v5.3.0 there were method renames as part of standardizing the
// APIs with the upstream FreeRTOS version, as such these methods were
// renamed and require usage of wrappers.
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5,3,0)
#define FREERTOS_GET_CURRENT_TASK_HANDLE_FOR_CORE(core) xTaskGetCurrentTaskHandleForCore(core)
#define FREERTOS_GET_IDLE_TASK_HANDLE_FOR_CORE(core) xTaskGetIdleTaskHandleForCore(core)
#else
#define FREERTOS_GET_CURRENT_TASK_HANDLE_FOR_CORE(core) xTaskGetCurrentTaskHandleForCPU(core)
#define FREERTOS_GET_IDLE_TASK_HANDLE_FOR_CORE(core) xTaskGetIdleTaskHandleForCPU(core)
#endif
#endif // ESP_PLATFORM

void cpuload_tick(unsigned irq)
{
    if (!Singleton<CpuLoad>::exists())
        return;
// On the ESP32 it is necessary to use a slightly different approach for
// recording CPU usage metrics since there may be additional CPU cores.
#ifdef ESP_PLATFORM
    if (irq != 0)
    {
        Singleton<CpuLoad>::instance()->record_value(true, (uintptr_t)irq);
    }
    else
    {
        // Record the first CPU core (PRO_CPU). NOTE: This assumes that OpenMRN
        // is running on this core.
        auto hdl = FREERTOS_GET_CURRENT_TASK_HANDLE_FOR_CORE(PRO_CPU_NUM);
        bool is_idle = FREERTOS_GET_IDLE_TASK_HANDLE_FOR_CORE(PRO_CPU_NUM) == hdl;
        Singleton<CpuLoad>::instance()->record_value(!is_idle, (uintptr_t)hdl);
    }
// NOTE: The ESP32-S2 and ESP32-C3 are single-core SoC and defines the
// FREERTOS_UNICORE flag which we can use here to disable recording of the
// APP_CPU. This can also be used on dual core SoCs with some restrictions.
#ifndef CONFIG_FREERTOS_UNICORE
    // Record the second CPU core (APP_CPU). NOTE: this is where application
    // code typically runs.
    auto hdl = FREERTOS_GET_CURRENT_TASK_HANDLE_FOR_CORE(APP_CPU_NUM);
    bool is_idle = FREERTOS_GET_IDLE_TASK_HANDLE_FOR_CORE(APP_CPU_NUM) == hdl;
    Singleton<CpuLoad>::instance()->record_value(!is_idle, (uintptr_t)hdl);
#endif // !CONFIG_FREERTOS_UNICORE
#else // NOT ESP_PLATFORM
    if (irq != 0)
    {
        Singleton<CpuLoad>::instance()->record_value(true, (uintptr_t)irq);
        return;
    }
    auto hdl = xTaskGetCurrentTaskHandle();
    bool is_idle = xTaskGetIdleTaskHandle() == hdl;
    Singleton<CpuLoad>::instance()->record_value(!is_idle, (uintptr_t)hdl);
#endif // ESP_PLATFORM
}
} // extern "C"

DEFINE_SINGLETON_INSTANCE(CpuLoad);

#endif // OPENMRN_FEATURE_THREAD_FREERTOS

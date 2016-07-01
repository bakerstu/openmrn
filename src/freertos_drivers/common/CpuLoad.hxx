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

#ifndef _OS_CPULOAD_HXX_
#define _OS_CPULOAD_HXX_

#include "utils/Singleton.hxx"

extern "C" {
/// Call this function repeatedly from a hardware timer to feed the CPUload
/// class.
void cpuload_tick(void);
}
 
/// Singleton class that records the CPU load under FreeRTOS.
///
/// Usage: 
///
/// . create a single (global) instance of this class.
///
/// . repeatedly call cpuload_tick() from a hardware timer. A rate of 100 Hz is
/// usually fine.
///
/// . retrieve CPU load when desired from the object of this class or via
///   Singleton<CpuLoad>::instance().
class CpuLoad : public Singleton<CpuLoad> {
public:
    CpuLoad() {}

    /// @returns the CPU load as an integer between 0 and 100. The load is
    /// averaged over the past short amount of time.
    uint8_t get_load() {
        return avg_;
    }

private:
    friend void cpuload_tick(void);
    /// Adds a value to the rolling average. @param value currently observed
    /// value to add to the average.
    inline void record_value(float value);

    /// Internal state for the rolling average (EWMA).
    float avg_{0.0};
};

#endif // _OS_CPULOAD_HXX_

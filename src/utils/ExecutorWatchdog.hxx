/** \copyright
 * Copyright (c) 2021, Balazs Racz
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
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
 * \file ExecutorWatchdog.hxx
 *
 * Watches an executor and prints a message if it was blocked.
 *
 * @author Balazs Racz
 * @date 5 April 2021
 */

#ifndef _UTILS_EXECUTORWATCHDOG_HXX_
#define _UTILS_EXECUTORWATCHDOG_HXX_

#include "executor/StateFlow.hxx"
#include "os/os.h"
#include "utils/logging.h"

/// This stateflow checks an executor every 50 msec. If the latency of a wakeup
/// is more than 50 msec, then prints a warning of how long the executor was
/// blocked.
class ExecutorWatchdog : public StateFlowBase
{
public:
    /// Constructor.
    /// @param service defines which executor to watch.
    ExecutorWatchdog(Service *service)
        : StateFlowBase(service)
    {
        start_flow(STATE(take_stamp));
    }

private:
    Action take_stamp()
    {
        lastTimeMsec_ = NSEC_TO_MSEC(os_get_time_monotonic());
        return sleep_and_call(&timer_, MSEC_TO_NSEC(50), STATE(woken));
    }

    Action woken()
    {
        uint32_t new_time_msec = NSEC_TO_MSEC(os_get_time_monotonic());
        auto diff = new_time_msec - lastTimeMsec_;
        if (diff > 100)
        {
            LOG(WARNING, "[WARN] Executor was blocked for %d msec",
                (int)(diff - 50));
        }
        if (++count_ > (5000 / 50))
        {
            count_ = 0;
            LOG(INFO, "Watchdog alive.");
        }
        return call_immediately(STATE(take_stamp));
    }

    StateFlowTimer timer_ {this};
    /// Timestamp when we last went to sleep.
    uint32_t lastTimeMsec_ {0};
    /// Counter that controls printing a heartbeat message that we are fine.
    int count_ {0};
};

#endif // _UTILS_EXECUTORWATCHDOG_HXX_

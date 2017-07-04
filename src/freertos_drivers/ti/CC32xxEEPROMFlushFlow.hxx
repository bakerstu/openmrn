/** \copyright
 * Copyright (c) 2016, Balazs Racz
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
 * \file CC32xxEEPROMFlushFlow.hxx
 * Helper state flow for flushing the CC32xx EEPROM emulation automatically.
 *
 * @author Balazs Racz
 * @date 24 June 2017
 */

#ifndef FREERTOS_DRIVERS_TI_CC32XXEEPROMFLUSHFLOW_HXX_
#define FREERTOS_DRIVERS_TI_CC32XXEEPROMFLUSHFLOW_HXX_

#include "executor/StateFlow.hxx"

extern "C" {
void eeprom_flush();
}

class EepromTimerFlow : public StateFlowBase, protected Atomic
{
public:
    EepromTimerFlow(Service* s)
        : StateFlowBase(s)
        , isWaiting_(0)
        , isDirty_(0)
    {
        start_flow(STATE(test_and_sleep));
    }

    void wakeup() {
        AtomicHolder h(this);
        isDirty_ = 1;
        if (isWaiting_) {
            isWaiting_ = 0;
            notify();
        }
    }

private:
    Action test_and_sleep()
    {
        bool need_sleep = false;
        {
            AtomicHolder h(this);
            if (isDirty_)
            {
                isDirty_ = 0;
                need_sleep = true;
            }
            else
            {
                isWaiting_ = 1;
            }
        }
        if (need_sleep)
        {
            return sleep_and_call(
                &timer_, MSEC_TO_NSEC(1000), STATE(test_and_flush));
        }
        else
        {
            return wait();
        }
    }

    Action test_and_flush()
    {
        bool need_sleep = false;
        {
            AtomicHolder h(this);
            if (isDirty_)
            {
                // we received another write during the sleep. Go back to sleep.
                isDirty_ = 0;
                need_sleep = true;
            }
        }
        if (need_sleep)
        {
            return sleep_and_call(
                &timer_, MSEC_TO_NSEC(1000), STATE(test_and_flush));
        }
        eeprom_flush();
        return call_immediately(STATE(test_and_sleep));
    }

    StateFlowTimer timer_{this};
    // 1 if the flow is sleeping and needs to be notified to wake up.
    unsigned isWaiting_ : 1;
    // 1 if we received a notification from the eeprom handler.
    unsigned isDirty_ : 1;
};

extern EepromTimerFlow eepromTimerFlow_;

#ifndef SKIP_UPDATED_CALLBACK
extern "C" {
void eeprom_updated_notification() {
    eepromTimerFlow_.wakeup();
}
}
#endif

#endif // FREERTOS_DRIVERS_TI_CC32XXEEPROMFLUSHFLOW_HXX_

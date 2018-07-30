/** @copyright
 * Copyright (c) 2018 Stuart W Baker
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
 * @file TivaPWM.hxx
 * This file implements a PWM driver for the Tiva timers.
 *
 * @author Balazs Racz
 * @date 12 March 2018
 */


#ifndef _FREEERTOS_DRIVERS_TI_TIVAPWM_HXX_
#define _FREEERTOS_DRIVERS_TI_TIVAPWM_HXX_

#include "PWM.hxx"

#include "driverlib/timer.h"


class TivaPWM : public PWM {
public:
    TivaPWM(unsigned timer_base, unsigned timer, unsigned period, unsigned duty)
        : base_(timer_base), timer_(timer) {
        set_period(period);
        set_duty(duty);
        MAP_TimerControlLevel(base_, timer_, true);
        MAP_TimerEnable(base_, timer_);
    }
    
    void set_period(uint32_t counts) override {
        MAP_TimerLoadSet(base_, timer_, counts);
    }
    virtual uint32_t get_period() override {
        return MAP_TimerLoadGet(base_, timer_);
    };
    void set_duty(uint32_t counts) override {
        MAP_TimerMatchSet(base_, timer_, counts);
    }
    virtual uint32_t get_duty() override {
        return MAP_TimerMatchGet(base_, timer_);
    }
    virtual uint32_t get_period_max() override {
        // @TODO figure out what is the max period
        return 0;
    }
    virtual uint32_t get_period_min() override {
        return 2;
    }
    
private:
    unsigned base_;
    unsigned timer_;
};

#endif // _FREEERTOS_DRIVERS_TI_TIVAPWM_HXX_

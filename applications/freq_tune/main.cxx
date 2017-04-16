/** \copyright
 * Copyright (c) 2013, Balazs Racz
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
 * \file main.cxx
 *
 * An application that blinks an LED.
 *
 * @author Balazs Racz
 * @date 3 Aug 2013
 */

#include <stdio.h>
#include <unistd.h>

#include "os/os.h"
#include "utils/blinker.h"
#include "console/Console.hxx"

#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "inc/hw_gpio.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"

#include "hardware.hxx"

GPIO_HWPIN(FREQ_OUT, GpioHwPin, F, 3, T1CCP1, Timer);

auto TOUT_PERIPH = SYSCTL_PERIPH_WTIMER0;
auto TOUT_BASE = TIMER1_BASE;
auto TOUT_TIM = TIMER_B;

unsigned freq = 1500; // hz
float duty = 0.3;

void start_pwm()
{
    FREQ_OUT_Pin::hw_init();
    MAP_SysCtlPeripheralEnable(TOUT_PERIPH);
    MAP_TimerConfigure(TOUT_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_B_PWM);
    MAP_TimerLoadSet(TOUT_BASE, TOUT_TIM, MAP_SysCtlClockGet() / freq);
    TimerMatchSet(
        TOUT_BASE, TOUT_TIM, MAP_SysCtlClockGet() * (1.0 - duty) / freq);
    MAP_TimerEnable(TOUT_BASE, TOUT_TIM);
}

void update_pwm()
{
    TimerMatchSet(
        TOUT_BASE, TOUT_TIM, MAP_SysCtlClockGet() * (1.0 - duty) / freq);
}


auto CAP_PERIPH = SYSCTL_PERIPH_WTIMER0;
auto CAP_BASE = WTIMER0_BASE;
auto CAP_TIM = TIMER_A;
auto CAP_INT = INT_WTIMER0A;
auto CAP_EVENT = TIMER_CAPA_EVENT;

GPIO_HWPIN(FREQ_IN, GpioHwPin, C, 4, WT0CCP0, Timer);

volatile int lastTimerValue_ = 0;
volatile int lastMeasure_ = 0;

extern "C" {
extern void wide_timer0a_interrupt_handler(void) {
    MAP_TimerIntClear(CAP_BASE, CAP_EVENT);
    int raw_new_value = MAP_TimerValueGet(CAP_BASE, CAP_TIM);
    lastMeasure_ = lastTimerValue_ - raw_new_value;
    lastTimerValue_ = raw_new_value;
}
}

void start_capture() {
    FREQ_IN_Pin::hw_init();
    MAP_SysCtlPeripheralEnable(CAP_PERIPH);
    MAP_TimerConfigure(CAP_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_TIME);
    MAP_TimerLoadSet(CAP_BASE, CAP_TIM, 1<<30);
    // TODO: up edge only
    MAP_TimerControlEvent(CAP_BASE, CAP_TIM, TIMER_EVENT_POS_EDGE);

    MAP_IntEnable(CAP_INT);
    MAP_TimerIntEnable(CAP_BASE, CAP_EVENT);
    MAP_TimerEnable(CAP_BASE, CAP_TIM);
}


/** Entry point to application.
 * @param argc number of command line arguments
 * @param argv array of command line arguments
 * @return 0, should never return
 */
int appl_main(int argc, char *argv[])
{
    setblink(0);
    start_pwm();
    start_capture();
    while (1)
    {
        usleep(100000);
        float freq = 1000.0 * MAP_SysCtlClockGet() / lastMeasure_;
        int ff = freq;
        printf("hello %ld %ld %d.%03d\r\n", MAP_SysCtlClockGet(), cm3_cpu_clock_hz, ff/1000, ff%1000 );
    }
    return 0;
}

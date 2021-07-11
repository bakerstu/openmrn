/** \copyright
 * Copyright (c) 2014, Balazs Racz
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
 * \file TivaNRZ.hxx
 *
 * Device driver for TivaWare to decode DCC track signal.
 *
 * @author Balazs Racz
 * @date 29 Nov 2014
 */

#include "TivaDCC.hxx"  // for FixedQueue
#include "TivaGPIO.hxx" // for pin definitions
#include "freertos_drivers/common/DccDecoder.hxx"

typedef DummyPin PIN_RailcomCutout;

/**
  Device driver for decoding a DCC signal on a TI Tiva class microcontroller.
 
  This driver exports a filesystem device node, but that file node is not
  usable for reading or writing anything at the moment. The only feature
  supported by this device driver right now is that it is able to tell a
  RailcomDriver when the railcom cutout is supposed to start, when we're in the
  middle and when it is over. This is necessary for the correct functionality
  of the railcom driver.

  @todo: implement actual packet decoding and sending back to the application
  layer.

  Usage: Define a structure declaring your hardware information. See below for
  what you need to define in there. Instantiate the device driver and pass the
  pointer to the railcom driver to the constructor. There is no need to touch
  the device from the application layer.

  Example hardware definitions:

struct DCCDecode
{
    static const auto TIMER_BASE = WTIMER4_BASE;
    static const auto TIMER_PERIPH = SYSCTL_PERIPH_WTIMER4;
    static const auto TIMER_INTERRUPT = INT_WTIMER4A;
    static const auto TIMER = TIMER_A;
    static const auto CFG_CAP_TIME_UP =
        TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_TIME_UP | TIMER_CFG_B_ONE_SHOT;
    // Interrupt bits.
    static const auto TIMER_CAP_EVENT = TIMER_CAPA_EVENT;
    static const auto TIMER_TIM_TIMEOUT = TIMER_TIMA_TIMEOUT;

    static const auto OS_INTERRUPT = INT_WTIMER4B;
    DECL_PIN(NRZPIN, D, 4);
    static const auto NRZPIN_CONFIG = GPIO_PD4_WT4CCP0;

    static const uint32_t TIMER_MAX_VALUE = 0x8000000UL;

    static const int Q_SIZE = 16;

};
 */

template <class HW> class TivaDccTimerModule
{
public:
    /// Exports the input pin to the driver on the module interface.
    using NRZ_Pin = typename HW::NRZ_Pin;

    // These constants are exported from HW to the driver.

    /// This is the counter from which the timer starts counting down. When the
    /// timer overflows, it starts from this value.
    static constexpr uint32_t TIMER_MAX_VALUE = HW::TIMER_MAX_VALUE;
    /// After how many timer counts we should take one sample.
    static constexpr uint32_t SAMPLE_PERIOD_CLOCKS = HW::SAMPLE_PERIOD_CLOCKS;
    /// Length of the device queue.
    static constexpr unsigned Q_SIZE = HW::Q_SIZE;

    /// @return Timer clocks per usec.
    static uint32_t get_ticks_per_usec()
    {
        return configCPU_CLOCK_HZ / 1000000;
    }

    /// Called once during construction time.
    static void module_init();

    /// Called inline with Device::enable().
    static void module_enable();

    /// Called inline with Device::disable().
    static void module_disable();

    /// Calls a software interrupt.
    static void trigger_os_interrupt()
    {
        MAP_IntPendSet(HW::OS_INTERRUPT);
    }

    /// hook
    static void dcc_before_cutout_hook()
    {
        HW::dcc_before_cutout_hook();
    }

    /// hook
    static void dcc_packet_finished_hook()
    {
        HW::dcc_packet_finished_hook();
    }

    /// hook
    static void after_feedback_hook()
    {
        HW::after_feedback_hook();
    }

    /// How many usec later should the railcom cutout start happen.
    static int time_delta_railcom_pre_usec()
    {
        return HW::time_delta_railcom_pre_usec();
    }

    /// How many usec later should the railcom cutout middle happen.
    static int time_delta_railcom_mid_usec()
    {
        return HW::time_delta_railcom_mid_usec();
    }

    /// How many usec later should the railcom cutout middle happen.
    static int time_delta_railcom_end_usec()
    {
        return HW::time_delta_railcom_end_usec();
    }
    
    /// Called from the capture interrupt handler. Checks interrupt status,
    /// clears interrupt.
    /// @return true if the interrupt was generated by a capture event.
    static inline bool int_get_and_clear_capture_event();

    /// Called from the interrupt handler if int_get_and_clear_capture_event
    /// said yes.
    /// @return the value of the downcounting capture at the time when the edge
    /// was captured.
    static inline uint32_t get_capture_counter();

    /// Called from the timeout interrupt handler. Checks interrupt status,
    /// clears timer expired interrupt.
    /// @return true if the interrupt was generated by the timer_delay_usec
    /// expiring event.
    static inline bool int_get_and_clear_delay_event();

    /// Delays a give number of usec using the capture timer feature. Needed
    /// for the timing of the railcom cutout.
    /// @param usec how much to delay.
    static void set_cap_timer_delay_usec(int usec)
    {
        Debug::DccPacketDelay::toggle();
        uint32_t new_match_v = usec * 80;
        MAP_TimerMatchSet(HW::TIMER_BASE, HW::RCOM_TIMER, 0xfffe - new_match_v);
        MAP_TimerPrescaleMatchSet(HW::TIMER_BASE, HW::RCOM_TIMER, 0);
    }

    /// Sets the timer to capture mode. Needed for the digitization of DCC
    /// signal bits.
    static void set_cap_timer_capture()
    {
        MAP_TimerDisable(HW::TIMER_BASE, HW::TIMER);
        MAP_TimerIntDisable(
            HW::TIMER_BASE, HW::TIMER_CAP_EVENT);
        MAP_TimerIntClear(
            HW::TIMER_BASE, HW::TIMER_CAP_EVENT);

        MAP_TimerConfigure(
            HW::TIMER_BASE, HW::CFG_TIM_CAPTURE | HW::CFG_RCOM_TIMER);
        MAP_TimerControlEvent(
            HW::TIMER_BASE, HW::TIMER, TIMER_EVENT_BOTH_EDGES);
        MAP_TimerLoadSet(HW::TIMER_BASE, HW::TIMER, HW::TIMER_MAX_VALUE);
        MAP_TimerPrescaleSet(HW::TIMER_BASE, HW::TIMER, HW::PS_MAX);

        MAP_TimerIntEnable(HW::TIMER_BASE, HW::TIMER_CAP_EVENT);
        MAP_TimerEnable(HW::TIMER_BASE, HW::TIMER);
    }

    /// Sets the timer to oneshot (timer) mode. Called once, then
    /// set_cap_timer_delay_usec() will be called multiple times, expecting
    /// each to deliver an rcom_interrupt().
    static void set_cap_timer_time()
    {
        MAP_TimerDisable(HW::TIMER_BASE, HW::RCOM_TIMER);

        MAP_TimerIntDisable(
            HW::TIMER_BASE, HW::TIMER_RCOM_MATCH);
        MAP_TimerIntClear(
            HW::TIMER_BASE, HW::TIMER_RCOM_MATCH);
        HW::clr_tim_mrsu();
        MAP_TimerLoadSet(HW::TIMER_BASE, HW::RCOM_TIMER, 0xfffe);
        MAP_TimerPrescaleSet(HW::TIMER_BASE, HW::RCOM_TIMER, 0);

        MAP_TimerIntEnable(HW::TIMER_BASE, HW::TIMER_RCOM_MATCH);
        MAP_TimerEnable(HW::TIMER_BASE, HW::RCOM_TIMER);
    }

    /// Called once inline in an interrupt. Signals that the delay timer is not
    /// needed anymore.
    static void stop_cap_timer_time()
    {
        MAP_TimerDisable(HW::TIMER_BASE, HW::RCOM_TIMER);
    }

    DISALLOW_COPY_AND_ASSIGN(TivaDccTimerModule);
};

template <class HW> void TivaDccTimerModule<HW>::module_init()
{
    MAP_SysCtlPeripheralEnable(HW::TIMER_PERIPH);
    MAP_GPIOPadConfigSet(HW::NRZ_Pin::GPIO_BASE, HW::NRZ_Pin::GPIO_PIN,
                         GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
}

template <class HW> void TivaDccTimerModule<HW>::module_enable()
{
    MAP_TimerClockSourceSet(HW::TIMER_BASE, TIMER_CLOCK_SYSTEM);
    MAP_TimerControlStall(HW::TIMER_BASE, HW::TIMER, true);

    set_cap_timer_capture();

    MAP_TimerIntEnable(HW::TIMER_BASE, HW::TIMER_CAP_EVENT);

    MAP_IntPrioritySet(HW::TIMER_INTERRUPT, 0x20);
    MAP_IntPrioritySet(HW::RCOM_INTERRUPT, 0x20);
    MAP_IntPrioritySet(HW::OS_INTERRUPT, configKERNEL_INTERRUPT_PRIORITY);
    MAP_IntEnable(HW::OS_INTERRUPT);
    MAP_IntEnable(HW::TIMER_INTERRUPT);
    MAP_IntEnable(HW::RCOM_INTERRUPT);
}

template <class HW> void TivaDccTimerModule<HW>::module_disable()
{
    MAP_IntDisable(HW::TIMER_INTERRUPT);
    MAP_IntDisable(HW::RCOM_INTERRUPT);
    MAP_IntDisable(HW::OS_INTERRUPT);
    MAP_TimerDisable(HW::TIMER_BASE, HW::TIMER);
    MAP_TimerDisable(HW::TIMER_BASE, HW::RCOM_TIMER);
}

template <class HW>
bool TivaDccTimerModule<HW>::int_get_and_clear_capture_event()
{
    // get masked interrupt status
    auto status = MAP_TimerIntStatus(HW::TIMER_BASE, true);
    if (status & HW::TIMER_CAP_EVENT)
    {
        //Debug::DccDecodeInterrupts::toggle();
        HW::cap_event_hook();
        MAP_TimerIntClear(HW::TIMER_BASE, HW::TIMER_CAP_EVENT);
        return true;
    }
    return false;
}

template <class HW> uint32_t TivaDccTimerModule<HW>::get_capture_counter()
{
    return MAP_TimerValueGet(HW::TIMER_BASE, HW::TIMER);
}

template <class HW> bool TivaDccTimerModule<HW>::int_get_and_clear_delay_event()
{
    Debug::DccDecodeInterrupts::set(true);
    auto status = MAP_TimerIntStatus(HW::TIMER_BASE, true);
    if (status & HW::TIMER_RCOM_MATCH)
    {
        MAP_TimerIntClear(HW::TIMER_BASE, HW::TIMER_RCOM_MATCH);
        return true;
    }
    return false;
}

template <class HW> using TivaDccDecoder = DccDecoder<TivaDccTimerModule<HW>>;

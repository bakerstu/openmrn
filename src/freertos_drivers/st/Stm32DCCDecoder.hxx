/** \copyright
 * Copyright (c) 2020, Balazs Racz
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
 * \file Stm32DCCDecoder.hxx
 *
 * Device driver module for STM32 to decode a DCC track signal.
 *
 * @author Balazs Racz
 * @date 9 Apr 2020
 */

#include "FreeRTOSConfig.h"
#include "Stm32Gpio.hxx" // for pin definitions
#include "freertos_drivers/common/DccDecoder.hxx"

#include "stm32f_hal_conf.hxx"

typedef DummyPin PIN_RailcomCutout;

/**
  Helper module for decoding a DCC signal on an STM32 class microcontroller.

  Usage: Define a structure `HW` declaring certain constants around your timer
  usage.

  Example hardware definitions:

```
struct DccDecoderHW
{
    /// The Pin (declared as GPIO input) where the DCC signal comes in.
    using NRZ_Pin = ::DCC_IN_Pin;
    /// Alternate Mode selector for the DCC_IN pin to put it into timer mode.
    static constexpr auto CAPTURE_AF_MODE = GPIO_AF1_TIM3;

    /// Takes an occupancy feedback sample every 3 milliseconds. The clock of
    /// the timer ticks every usec. This is only useful if there is a railcom
    /// driver that can also measure current used in forward mode.
    static constexpr uint32_t SAMPLE_PERIOD_TICKS = 3000;

    /// Length of the ring buffer for packets waiting for userspace to read.
    static constexpr unsigned Q_SIZE = 6;

    /// Defines the timer resource matching the Capture pin. It can be either a
    /// 16-bit or a 32-bit timer.
    static const auto CAPTURE_TIMER = TIM3_BASE;
    /// Which channel of the timer we should be capturing on. Defined by the
    /// Capture pin.
    static constexpr uint32_t CAPTURE_CHANNEL = TIM_CHANNEL_1;
    /// Interrupt flag for the given capture channel.
    static constexpr auto CAPTURE_IF = TIM_FLAG_CC1;
    /// Digital filter to apply to the captured stream for edge detection. The
    /// 0b1000 value needs 6 consecutive samples at f_CLK/8 to be the same
    /// value to trigger the edge detection. This is about 1 usec at 48 MHz.
    static constexpr unsigned CAPTURE_FILTER = 0b1000;
    /// Interrupt vector number for the capture timer resource.
    static constexpr auto CAPTURE_IRQn = TIM3_IRQn;

    /// Hook called in a P0 interrupt context every edge.
    static void cap_event_hook() {}
    /// Hook called in a P0 interrupt context before the DCC cutout is enabled.
    static inline void dcc_before_cutout_hook() {}
    /// Hook called in a P0 interrupt context when a full DCC packet is
    /// received. This is after the packet ending one bit, or after the cutout.
    static inline void dcc_packet_finished_hook() {}
    /// Hook called in a P0 interrupt context after we instructed the railcom
    /// driver to take a feedback sample.
    static inline void after_feedback_hook() {}

    /// Second timer resource that will be used to measure microseconds for the
    /// railcom cutout. May be the same as the Capture Timer, if there are at
    /// least two channels on that timer resource.
    static const auto USEC_TIMER = TIM3_BASE;
    /// Channel to use for the timing purpose. This channel shall NOT be
    /// connected to a pin.
    static constexpr uint32_t USEC_CHANNEL = TIM_CHANNEL_2;
    /// Interrupt flag for the USEC_CHANNEL.
    static constexpr auto USEC_IF = TIM_FLAG_CC2;
    static_assert(TIM_FLAG_CC2 == TIM_IT_CC2,
        "Flag and interrupt registers must be in parallel. The HAL driver is "
        "broken.");
    /// Interrupt vector number for the usec timer resource.
    static constexpr auto TIMER_IRQn = TIM3_IRQn;

    /// An otherwise unused interrupt vector number, which can be used as a
    /// software interrupt in a kernel-compatible way.
    static constexpr auto OS_IRQn = TSC_IRQn;
};
```
 */

template <class HW> class Stm32DccTimerModule
{
public:
    // Declaration matching the DccOutput static class pattern.
    using Output = typename HW::Output;

    /// Exports the input pin to the driver on the module interface.
    using NRZ_Pin = typename HW::NRZ_Pin;

    // These constants are exported from HW to the driver.

    /// This is the counter from which the timer starts counting down. When the
    /// timer overflows, it starts from this value.
    static constexpr uint32_t TIMER_MAX_VALUE = 0xffff;
    /// After how many timer counts we should take one sample for occupancy
    /// feedback.
    static constexpr uint32_t SAMPLE_PERIOD_CLOCKS = HW::SAMPLE_PERIOD_TICKS;
    /// Length of the device queue.
    static constexpr unsigned Q_SIZE = HW::Q_SIZE;

    /// @return Timer clocks per usec.
    static uint32_t get_ticks_per_usec()
    {
        // We set the timer prescaler to go at one tick per usec.
        return 1;
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
        NVIC_SetPendingIRQ(HW::OS_IRQn);
    }

    /// hook
    static void before_cutout_hook()
    {
        HW::before_cutout_hook();
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

    /// Delays a given number of usec using the usec timer feature. Needed for
    /// the timing of the railcom cutout. An rcom_interrupt shall be raised
    /// after this many usec.
    /// @param usec how much to delay, with the measurement started from the
    /// set_cap_timer_time() call (or the previous edge captured before that by
    /// the capture timer).
    static void set_cap_timer_delay_usec(int usec)
    {
        Debug::DccPacketDelay::toggle();
        // This code handles underflow of the timer correctly. We cannot wait
        // longer than one full cycle though (65 msec -- typical RailCom waits
        // are 20-500 usec).
        uint32_t new_match_v = usecTimerStart_ + usec;
        new_match_v &= 0xffff;
        __HAL_TIM_SET_COMPARE(
            usec_timer_handle(), HW::USEC_CHANNEL, new_match_v);
        __HAL_TIM_CLEAR_IT(usec_timer_handle(), HW::USEC_IF);
        __HAL_TIM_ENABLE_IT(usec_timer_handle(), HW::USEC_IF);
    }

    /// Sets the timer to capture mode. Needed for the digitization of DCC
    /// signal bits.
    static void set_cap_timer_capture()
    {
        __HAL_TIM_CLEAR_IT(capture_timer_handle(), HW::CAPTURE_IF);
        /// @todo consider clearing the overflow flag as well.
        __HAL_TIM_ENABLE_IT(capture_timer_handle(), HW::CAPTURE_IF);
    }

    /// Sets the timer to oneshot (timer) mode. Called once, then
    /// set_cap_timer_delay_usec() will be called multiple times, expecting
    /// each to deliver an rcom_interrupt().
    static void set_cap_timer_time()
    {
        if (shared_timers()) {
            usecTimerStart_ =  get_raw_capture_counter();
        } else {
            usecTimerStart_ =  __HAL_TIM_GET_COUNTER(usec_timer_handle());
        }
        /// @TODO __HAL_TIM_DISABLE_IT(capture_timer_handle(), HW::CAPTURE_IF);
        // channel setup already happened in module_enable.
    }

    /// Called once inline in an interrupt. Signals that the delay timer is not
    /// needed anymore.
    static void stop_cap_timer_time()
    {
        __HAL_TIM_DISABLE_IT(usec_timer_handle(), HW::USEC_IF);
        //TIM_CCxChannelCmd(usec_timer(), HW::USEC_CHANNEL, TIM_CCx_DISABLE);
    }

    /// Called during the railcom cutout when the railcom usec timer expires.
    /// The implementation is expected to modify cutout_state if the default
    /// sequence of handlers are not matching the desired
    /// functionality. Additional wakeups can be scheduled with
    /// set_cap_timer_delay_usec(...). An empty implementation is acceptable.
    static inline void rcom_cutout_hook(DccDecoderDefs::CutoutState *cutout_state);

private:
    static TIM_HandleTypeDef captureTimerHandle_;
    static TIM_HandleTypeDef usecTimerHandle_;
    /// Holds the base value from where the usec timer should be counting down
    /// when the delay_usec call is invoked. This is set from the last capture
    /// edge when the mode is switched to the usec timer.
    static uint32_t usecTimerStart_;

    /// Initializes a timer resource (shared for all channels).
    /// @param handle pointer to the HAL timer handle.
    /// @param instance pointer to the timer registers, such as TIM3.
    static void init_timer(TIM_HandleTypeDef *handle, TIM_TypeDef *instance)
    {
        memset(handle, 0, sizeof(*handle));
        handle->Instance = instance;
        handle->Init.Period = TIMER_MAX_VALUE;
        // 1 usec per tick
        handle->Init.Prescaler = configCPU_CLOCK_HZ / 1000000;
        handle->Init.ClockDivision = 0;
        handle->Init.CounterMode = TIM_COUNTERMODE_UP;
        handle->Init.RepetitionCounter = 0;
        handle->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

        HASSERT(HAL_TIM_IC_DeInit(handle) == HAL_OK);
        HASSERT(HAL_TIM_IC_Init(handle) == HAL_OK);
    }

    /// Helper function to read the raw capture register value.
    /// @return the value of the CCx register matching the capture channel.
    static inline uint32_t get_raw_capture_counter();
    
    /// @return the hardware pointer to the capture timer.
    static TIM_TypeDef *capture_timer()
    {
        return reinterpret_cast<TIM_TypeDef *>(HW::CAPTURE_TIMER);
    }
    /// @return the HAL handle to the capture timer.
    static TIM_HandleTypeDef *capture_timer_handle()
    {
        return &captureTimerHandle_;
    }

    /// @return the hardware pointer to the usec timer.
    static TIM_TypeDef *usec_timer()
    {
        return reinterpret_cast<TIM_TypeDef *>(HW::USEC_TIMER);
    }
    /// @return the HAL handle to the usec timer (which may alias to the
    /// capture timer handle).
    static TIM_HandleTypeDef *usec_timer_handle()
    {
        if (shared_timers())
        {
            return capture_timer_handle();
        }
        return &usecTimerHandle_;
    }

    /// @return true if the same timer resource (on different channels) is used
    /// for both the capture and the usec timer.
    static bool shared_timers()
    {
        return HW::CAPTURE_TIMER == HW::USEC_TIMER;
    }

    /// Private constructor. This class cannot be instantiated.
    Stm32DccTimerModule();

    DISALLOW_COPY_AND_ASSIGN(Stm32DccTimerModule);
};

template <class HW>
TIM_HandleTypeDef Stm32DccTimerModule<HW>::captureTimerHandle_;
template <class HW> TIM_HandleTypeDef Stm32DccTimerModule<HW>::usecTimerHandle_;
template <class HW> uint32_t Stm32DccTimerModule<HW>::usecTimerStart_;

template <class HW> void Stm32DccTimerModule<HW>::module_init()
{
    memset(&captureTimerHandle_, 0, sizeof(captureTimerHandle_));
    memset(&usecTimerHandle_, 0, sizeof(usecTimerHandle_));
    // GPIO input.
    NRZ_Pin::hw_init();
}

template <class HW> void Stm32DccTimerModule<HW>::module_enable()
{
    init_timer(capture_timer_handle(), capture_timer());
    // Switches pin to GPIO input.
    NRZ_Pin::hw_init();
    // This must precede switching the pin to alternate mode to avoid the MCU
    // driving the pin as an output in case the timer channel registers are
    // uninitialized or in PWM mode.
    set_cap_timer_capture();

    GPIO_InitTypeDef gpio_init;
    memset(&gpio_init, 0, sizeof(gpio_init));
    gpio_init.Mode = GPIO_MODE_AF_PP;
    gpio_init.Pull = GPIO_PULLUP;
    gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio_init.Alternate = HW::CAPTURE_AF_MODE;
    gpio_init.Pin = HW::NRZ_Pin::pin();
    HAL_GPIO_Init(HW::NRZ_Pin::port(), &gpio_init);

    if (!shared_timers())
    {
        init_timer(usec_timer_handle(), usec_timer());
    }

    // Set up capture channel.
    {
        TIM_IC_InitTypeDef channel_init;
        memset(&channel_init, 0, sizeof(channel_init));
        channel_init.ICPolarity = TIM_ICPOLARITY_BOTHEDGE;
        channel_init.ICSelection = TIM_ICSELECTION_DIRECTTI;
        channel_init.ICPrescaler = TIM_ICPSC_DIV1;
        channel_init.ICFilter = HW::CAPTURE_FILTER;
        HASSERT(HAL_TIM_IC_ConfigChannel(capture_timer_handle(), &channel_init,
                    HW::CAPTURE_CHANNEL) == HAL_OK);
    }

    HASSERT(HAL_TIM_IC_Start_IT(capture_timer_handle(), HW::CAPTURE_CHANNEL) ==
        HAL_OK);
    // Disable interrupt until the set_cap_timer_capture() is called.
    __HAL_TIM_DISABLE_IT(capture_timer_handle(), HW::CAPTURE_IF);

    // Set up timing channel
    {
        TIM_OC_InitTypeDef channel_init;
        memset(&channel_init, 0, sizeof(channel_init));
        channel_init.OCMode = TIM_OCMODE_TIMING; // frozen -- no output
        channel_init.Pulse = (__HAL_TIM_GET_COUNTER(usec_timer_handle()) + 1) &
            0xffff; // will be reloaded in the delay_usec function.
        // the rest are irrelevant.
        channel_init.OCPolarity = TIM_OCPOLARITY_HIGH;
        channel_init.OCNPolarity = TIM_OCNPOLARITY_HIGH;
        channel_init.OCFastMode = TIM_OCFAST_DISABLE;
        channel_init.OCIdleState = TIM_OCIDLESTATE_RESET;
        channel_init.OCNIdleState = TIM_OCNIDLESTATE_RESET;
        HASSERT(HAL_TIM_OC_ConfigChannel(usec_timer_handle(), &channel_init,
                    HW::USEC_CHANNEL) == HAL_OK);
    }

    HASSERT(
        HAL_TIM_OC_Start_IT(usec_timer_handle(), HW::USEC_CHANNEL) == HAL_OK);
    // Disable interrupt until the delay_usec() is called.
    __HAL_TIM_DISABLE_IT(usec_timer_handle(), HW::USEC_IF);

#if defined(GCC_ARMCM0)
    HAL_NVIC_SetPriority(HW::CAPTURE_IRQn, 1, 0);
    HAL_NVIC_SetPriority(HW::TIMER_IRQn, 1, 0);
    HAL_NVIC_SetPriority(HW::OS_IRQn, 3, 0);
#elif defined(GCC_ARMCM3)
    SetInterruptPriority(HW::CAPTURE_IRQn, 0x20);
    SetInterruptPriority(HW::TIMER_IRQn, 0x20);
    SetInterruptPriority(HW::OS_IRQn, configKERNEL_INTERRUPT_PRIORITY);
#else
#error not defined how to set interrupt priority
#endif

    NVIC_EnableIRQ(HW::CAPTURE_IRQn);
    NVIC_EnableIRQ(HW::TIMER_IRQn);
    NVIC_EnableIRQ(HW::OS_IRQn);
}

template <class HW> void Stm32DccTimerModule<HW>::module_disable()
{
    // Switches pin to GPIO input.
    NRZ_Pin::hw_init();

    capture_timer_handle()->Instance = capture_timer();
    usec_timer_handle()->Instance = usec_timer();
    NVIC_DisableIRQ(HW::CAPTURE_IRQn);
    NVIC_DisableIRQ(HW::TIMER_IRQn);
    NVIC_DisableIRQ(HW::OS_IRQn);
    HASSERT(HAL_TIM_IC_Stop_IT(capture_timer_handle(), HW::CAPTURE_CHANNEL) ==
        HAL_OK);

    HASSERT(
        HAL_TIM_OC_Stop_IT(usec_timer_handle(), HW::USEC_CHANNEL) == HAL_OK);
    if (!shared_timers())
    {
        HASSERT(HAL_TIM_IC_DeInit(usec_timer_handle()) == HAL_OK);
    }
    HASSERT(HAL_TIM_IC_DeInit(capture_timer_handle()) == HAL_OK);
}

template <class HW>
bool Stm32DccTimerModule<HW>::int_get_and_clear_capture_event()
{
    if (__HAL_TIM_GET_FLAG(capture_timer_handle(), HW::CAPTURE_IF))
    {
        //Debug::DccDecodeInterrupts::toggle();
        HW::cap_event_hook();
        __HAL_TIM_CLEAR_FLAG(capture_timer_handle(), HW::CAPTURE_IF);
        return true;
    }
    return false;
}

template <class HW> uint32_t Stm32DccTimerModule<HW>::get_capture_counter()
{
    // Simulates down-counting.
    return TIMER_MAX_VALUE - get_raw_capture_counter();
}

template <class HW> uint32_t Stm32DccTimerModule<HW>::get_raw_capture_counter()
{
    // This if structure will be optimized away.
    if (HW::CAPTURE_CHANNEL == TIM_CHANNEL_1)
    {
        return capture_timer()->CCR1;
    }
    else if (HW::CAPTURE_CHANNEL == TIM_CHANNEL_2)
    {
        return capture_timer()->CCR2;
    }
    else if (HW::CAPTURE_CHANNEL == TIM_CHANNEL_3)
    {
        return capture_timer()->CCR3;
    }
    else if (HW::CAPTURE_CHANNEL == TIM_CHANNEL_4)
    {
        return capture_timer()->CCR4;
    }
    else
    {
        DIE("Unknown capture channel");
        return 0;
    }
}

template <class HW>
bool Stm32DccTimerModule<HW>::int_get_and_clear_delay_event()
{
    if (__HAL_TIM_GET_FLAG(usec_timer_handle(), HW::USEC_IF))
    {
        Debug::DccDecodeInterrupts::set(true);
        __HAL_TIM_CLEAR_IT(usec_timer_handle(), HW::USEC_IF);
        // we also disable the interrupt until it is reenabled by loading a new
        // usec delay target.
        __HAL_TIM_DISABLE_IT(usec_timer_handle(), HW::USEC_IF);
        return true;
    }
    return false;
}

template <class HW> using Stm32DccDecoder = DccDecoder<Stm32DccTimerModule<HW>>;

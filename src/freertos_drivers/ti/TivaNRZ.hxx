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
 * Device driver for TivaWare to decode NRZ codes. (DCC, Marklin-Motorola, M4
 * are examples of NRZ codes used in model railroading.)  The driver will time
 * rising and falling edges of the input signal and return the timing data to
 * userspace.
 *
 * @author Balazs Racz
 * @date 29 Nov 2014
 */

#include "TivaDCC.hxx" // for FixedQueue

/*
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

/// Protocol-independent driver for decoding NRZ codes using a TI Tiva class
/// microcontroller. The driver will capture and accurately time each half wave
/// of the incoming signal. The timing information will be available to the
/// user-space by executing read() commands on the fd of the device
/// driver. Then arbitrary user-space code can be run to decode the correct
/// data stream without needing timing-accurate code in user space.
///
/// Supports asynchronous reads using the Notifiable method.
template <class HW> class TivaNRZ : public Node
{
public:
    /// Constructor. @param name is the device filesystem name (e.g.
    /// "/dev/nrz0")
    TivaNRZ(const char *name);

    ~TivaNRZ()
    {
    }

    /** Handles a raw interrupt. */
    inline void interrupt_handler() __attribute__((always_inline));

    /** Handles a software interrupt to FreeRTOS. */
    inline void os_interrupt_handler() __attribute__((always_inline));

private:
    /** Read from a file or device.
     * @param file file reference for this device
     * @param buf location to place read data
     * @param count number of bytes to read
     * @return number of bytes read upon success, -1 upon failure with errno
     * containing the cause
     */
    ssize_t read(File *file, void *buf, size_t count) OVERRIDE;

    /** Write to a file or device.
     * @param file file reference for this device
     * @param buf location to find write data
     * @param count number of bytes to write
     * @return number of bytes written upon success, -1 upon failure with errno
     * containing the cause
     */
    ssize_t write(File *file, const void *buf, size_t count) OVERRIDE
    {
        return -EINVAL;
    }

    /** Request an ioctl transaction
     * @param file file reference for this device
     * @param key ioctl key
     * @param data key data
     */
    int ioctl(File *file, unsigned long int key, unsigned long data) OVERRIDE;

    void enable();  /**< function to enable device */
    void disable(); /**< function to disable device */

    /** Discards all pending buffers.  Called after disable().
     */
    void flush_buffers()
    {
        while (!inputData_.empty())
        {
            inputData_.increment_front();
        }
    };

    /// Data to send measurements back to the application level.
    FixedQueue<uint32_t, HW::Q_SIZE> inputData_;
    /// Free running timer's last value.
    uint32_t lastTimerValue_;
    /// How many times the free running timer overflowed.
    uint32_t reloadCount_;
    /// unused. @todo delete
    unsigned lastLevel_;
    /// Flags when we lost data due to buffer overrun.
    bool overflowed_ = false;
    /// Notifiable for asynchronous support.
    Notifiable *readableNotifiable_ = nullptr;

    DISALLOW_COPY_AND_ASSIGN(TivaNRZ);
};

template <class HW>
TivaNRZ<HW>::TivaNRZ(const char *name)
    : Node(name)
{
    MAP_SysCtlPeripheralEnable(HW::TIMER_PERIPH);
    MAP_SysCtlPeripheralEnable(HW::NRZPIN_PERIPH);
    MAP_GPIOPinConfigure(HW::NRZPIN_CONFIG);
    MAP_GPIOPinTypeTimer(HW::NRZPIN_BASE, HW::NRZPIN_PIN);

    disable();
}

template <class HW> void TivaNRZ<HW>::enable()
{
    disable();
    MAP_TimerClockSourceSet(HW::TIMER_BASE, TIMER_CLOCK_SYSTEM);
    MAP_TimerConfigure(HW::TIMER_BASE, HW::CFG_CAP_TIME_UP);
    MAP_TimerControlStall(HW::TIMER_BASE, HW::TIMER, true);
    MAP_TimerControlEvent(HW::TIMER_BASE, HW::TIMER, TIMER_EVENT_BOTH_EDGES);
    MAP_TimerLoadSet(HW::TIMER_BASE, HW::TIMER, HW::TIMER_MAX_VALUE);
    MAP_TimerPrescaleSet(HW::TIMER_BASE, HW::TIMER, HW::PS_MAX);

    reloadCount_ = 0;
    lastTimerValue_ = 0;

    MAP_TimerIntEnable(HW::TIMER_BASE, HW::TIMER_CAP_EVENT);
    MAP_TimerIntEnable(HW::TIMER_BASE, HW::TIMER_TIM_TIMEOUT);

    MAP_IntPrioritySet(HW::TIMER_INTERRUPT, 0);
    MAP_IntPrioritySet(HW::OS_INTERRUPT, configKERNEL_INTERRUPT_PRIORITY);
    MAP_IntEnable(HW::OS_INTERRUPT);
    MAP_IntEnable(HW::TIMER_INTERRUPT);

    MAP_TimerEnable(HW::TIMER_BASE, HW::TIMER);
}

template <class HW> void TivaNRZ<HW>::disable()
{
    MAP_IntDisable(HW::TIMER_INTERRUPT);
    MAP_IntDisable(HW::OS_INTERRUPT);
    MAP_TimerDisable(HW::TIMER_BASE, HW::TIMER);
}

template <class HW>
__attribute__((optimize("-O3"))) void TivaNRZ<HW>::interrupt_handler()
{
    // get masked interrupt status
    auto status = MAP_TimerIntStatus(HW::TIMER_BASE, true);
    if (status & HW::TIMER_TIM_TIMEOUT)
    {
        // The timer got reloaded.
        reloadCount_++;
        MAP_TimerIntClear(HW::TIMER_BASE, HW::TIMER_TIM_TIMEOUT);
    }
    // TODO(balazs.racz): Technically it is possible that the timer reload
    // happens between the event match and the interrupt entry. In this case we
    // will incorrectly add a full cycle to the event length.
    if (status & HW::TIMER_CAP_EVENT)
    {
        MAP_TimerIntClear(HW::TIMER_BASE, HW::TIMER_CAP_EVENT);
        MAP_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_0, 0);
        static uint32_t raw_new_value;
        raw_new_value = MAP_TimerValueGet(HW::TIMER_BASE, HW::TIMER);
        static uint32_t new_value;
        new_value = raw_new_value;
        while (reloadCount_ > 0)
        {
            new_value += HW::TIMER_MAX_VALUE;
            reloadCount_--;
        }
        //HASSERT(new_value > lastTimerValue_);
        new_value -= lastTimerValue_;
        if (!inputData_.full())
        {
            inputData_.back() = overflowed_ ? 3 : new_value;
            overflowed_ = false;
            inputData_.increment_back();
        } else {
            overflowed_ = true;
        }
        lastTimerValue_ = raw_new_value;
        MAP_IntPendSet(HW::OS_INTERRUPT);
    }
}

template <class HW>
__attribute__((optimize("-O3"))) void TivaNRZ<HW>::os_interrupt_handler()
{
    if (!inputData_.empty())
    {
        Notifiable *n = readableNotifiable_;
        readableNotifiable_ = nullptr;
        if (n) {
            n->notify_from_isr();
            os_isr_exit_yield_test(true);
        }
    }
}

template <class HW>
int TivaNRZ<HW>::ioctl(File *file, unsigned long int key, unsigned long data)
{
    if (IOC_TYPE(key) == CAN_IOC_MAGIC && IOC_SIZE(key) == NOTIFIABLE_TYPE &&
        key == CAN_IOC_READ_ACTIVE)
    {
        Notifiable *n = reinterpret_cast<Notifiable *>(data);
        HASSERT(n);
        // If there is no data for reading, we put the incoming notification
        // into the holder. Otherwise we notify it immediately.
        if (inputData_.empty())
        {
            portENTER_CRITICAL();
            if (inputData_.empty())
            {
                // We are in a critical section now. If we got into this
                // branch, then the buffer was full at the beginning of the
                // critical section. If the hardware interrupt kicks in now,
                // and sets the os_interrupt to pending, the os interrupt will
                // not happen until we leave the critical section, and thus the
                // swap will be in effect by then.
                std::swap(n, readableNotifiable_);
            }
            portEXIT_CRITICAL();
        }
        if (n)
        {
            n->notify();
        }
        return 0;
    }
    errno = EINVAL;
    return -1;
}

/** Read from a file or device.
 * @param file file reference for this device
 * @param buf location to place read data
 * @param count number of bytes to read
 * @return number of bytes read upon success, -1 upon failure with errno containing the cause
 */
template<class HW>
ssize_t TivaNRZ<HW>::read(File *file, void *buf, size_t count)
{
    if (count != 4)
    {
        return -EINVAL;
    }
    // We only need this critical section to prevent concurrent threads from
    // reading at the same time.
    portENTER_CRITICAL();
    if (inputData_.empty()) {
        portEXIT_CRITICAL();
        return -EAGAIN;
    }
    uint32_t v = reinterpret_cast<uint32_t>(buf);
    HASSERT((v & 3) == 0); // alignment check.
    uint32_t* pv = static_cast<uint32_t*>(buf);
    *pv = inputData_.front();
    inputData_.increment_front();
    portEXIT_CRITICAL();
    return count;
}

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
#include "RailcomDriver.hxx" // for debug pins
#include "dcc/Receiver.hxx"

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
template <class HW> class TivaDccDecoder : public Node
{
public:
    /// Constructor.
    ///
    /// @param name name of device node, e.g. "/dev/dccdecode0"
    /// @param railcom_driver is the associated railcom driver, which will get
    /// the callbacks from the timing of the acquired signal.
    TivaDccDecoder(const char *name, RailcomDriver *railcom_driver);

    ~TivaDccDecoder()
    {
        inputData_->destroy();
    }

    /** Handles a raw interrupt. */
    inline void interrupt_handler() __attribute__((always_inline));

    /** Handles interrupt from the second timer used for railcom timing. */
    inline void rcom_interrupt_handler() __attribute__((always_inline));

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
    ssize_t read(File *file, void *buf, size_t count) OVERRIDE {
        HASSERT((count % inputData_->member_size()) == 0);
        unsigned max = count / inputData_->member_size();
        while (true)
        {
            portENTER_CRITICAL();
            unsigned copied = inputData_->get((input_data_type *)buf, max);
            if (!nextPacketData_ && inputData_->space())
            {
                inputData_->data_write_pointer(&nextPacketData_);
            }
            portEXIT_CRITICAL();
            if (copied > 0)
            {
                return copied * inputData_->member_size();
            }
            if (file->flags & O_NONBLOCK)
            {
                return -EAGAIN;
            }
            inputData_->block_until_condition(file, true);
        }
    }

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

    /** Device select method. Default impementation returns true.
     * @param file reference to the file
     * @param mode FREAD for read active, FWRITE for write active, 0 for
     *        exceptions
     * @return true if active, false if inactive
     */
    bool select(File *file, int mode)
    {
        bool retval = false;
        portENTER_CRITICAL();
        switch (mode)
        {
            case FREAD:
                if (inputData_->pending() > 0)
                {
                    retval = true;
                }
                else
                {
                    inputData_->select_insert();
                }
                break;
            default:
                break;
        }
        portEXIT_CRITICAL();
        return retval;
    }

    void enable() override;  /**< function to enable device */
    void disable() override; /**< function to disable device */

    /** Discards all pending buffers.  Called after disable().
     */
    void flush_buffers() override
    {
        inputData_->flush();
    };

    /*
    void set_sample_timer_period()
    {
        MAP_TimerDisable(HW::TIMER_BASE, HW::SAMPLE_TIMER);
        MAP_TimerIntDisable(HW::TIMER_BASE, HW::SAMPLE_TIMER_TIMEOUT);
        MAP_TimerLoadSet(HW::TIMER_BASE, HW::SAMPLE_TIMER,
            HW::SAMPLE_PERIOD_CLOCKS & 0xffffU);
        MAP_TimerPrescaleSet(
            HW::TIMER_BASE, HW::SAMPLE_TIMER, HW::SAMPLE_PERIOD_CLOCKS >> 16);
        MAP_TimerEnable(HW::TIMER_BASE, HW::SAMPLE_TIMER);
        }*/

    /// Delays a give number of usec using the capture timer feature. Needed
    /// for the timing ofthe railcom cutout.
    /// @param usec how much to delay.
    void set_cap_timer_delay_usec(int usec)
    {
        Debug::DccPacketDelay::toggle();
        uint32_t new_match_v = usec * 80;
        MAP_TimerMatchSet(HW::TIMER_BASE, HW::RCOM_TIMER, 0xfffe - new_match_v);
        MAP_TimerPrescaleMatchSet(HW::TIMER_BASE, HW::RCOM_TIMER, 0);
    }

    /// Sets the timer to capture mode. Needed for the digitization of DCC
    /// signal bits.
    void set_cap_timer_capture()
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

        lastTimerValue_ = HW::TIMER_MAX_VALUE;
        nextSample_ = lastTimerValue_ - HW::SAMPLE_PERIOD_CLOCKS;

        MAP_TimerIntEnable(HW::TIMER_BASE, HW::TIMER_CAP_EVENT);
        MAP_TimerEnable(HW::TIMER_BASE, HW::TIMER);
    }

    /// Sets the timer to oneshot (timer) mode.
    void set_cap_timer_time()
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

    typedef DCCPacket input_data_type;
    DeviceBuffer<DCCPacket>* inputData_{
        DeviceBuffer<DCCPacket>::create(HW::Q_SIZE)};
    DCCPacket* nextPacketData_{nullptr};
    bool nextPacketFilled_{false};
    /// Holds the value ofthe free running timer at the time we captured the
    /// previous edge.
    uint32_t lastTimerValue_;
    /// Holds the timer value when we should be taking an occupnacy sample the
    /// next time.
    uint32_t nextSample_;
    /// true if the next edge we shall sample.
    bool sampleActive_ = false;
    /// Seems unused? @todo delete.
    unsigned lastLevel_;
    /// if true then sampling will be suspended until the timer overflows.
    bool waitSampleForOverflow_ = false;
    /// True if the current internal state is the cutout state.
    bool inCutout_ = false;
    /// True for the last bit time before the cutout, to prevent sampling fro
    /// colliding with cutout.
    bool prepCutout_ = false;
    /// Which window of the cutout we are in.
    uint32_t cutoutState_;

    /// notified for cutout events.
    RailcomDriver *railcomDriver_;

    /// DCC packet decoder state machine and internal state.
    dcc::DccDecoder decoder_;

    /// How many usec the railcom has before the cutout
    static const auto RAILCOM_CUTOUT_PRE = 26;
    /// How many usec the railcom has to the middle of window
    static const auto RAILCOM_CUTOUT_MID = 173;
    /// How many usec the railcom has to the end of the window
    static const auto RAILCOM_CUTOUT_END = 470;

    DISALLOW_COPY_AND_ASSIGN(TivaDccDecoder);
};

template <class HW>
TivaDccDecoder<HW>::TivaDccDecoder(const char *name,
                                   RailcomDriver *railcom_driver)
    : Node(name)
    , railcomDriver_(railcom_driver)
{
    MAP_SysCtlPeripheralEnable(HW::TIMER_PERIPH);
    HW::NRZ_Pin::hw_init();

    disable();
}

template <class HW> void TivaDccDecoder<HW>::enable()
{
    disable();
    MAP_TimerClockSourceSet(HW::TIMER_BASE, TIMER_CLOCK_SYSTEM);
    MAP_TimerControlStall(HW::TIMER_BASE, HW::TIMER, true);

    if (!nextPacketData_)
    {
        if (inputData_->space())
        {
            inputData_->data_write_pointer(&nextPacketData_);
        }
    }

    set_cap_timer_capture();

    MAP_TimerIntEnable(HW::TIMER_BASE, HW::TIMER_CAP_EVENT);

    MAP_IntPrioritySet(HW::TIMER_INTERRUPT, 0);
    MAP_IntPrioritySet(HW::RCOM_INTERRUPT, 0);
    MAP_IntPrioritySet(HW::OS_INTERRUPT, configKERNEL_INTERRUPT_PRIORITY);
    MAP_IntEnable(HW::OS_INTERRUPT);
    MAP_IntEnable(HW::TIMER_INTERRUPT);
    MAP_IntEnable(HW::RCOM_INTERRUPT);
}

template <class HW> void TivaDccDecoder<HW>::disable()
{
    MAP_IntDisable(HW::TIMER_INTERRUPT);
    MAP_IntDisable(HW::RCOM_INTERRUPT);
    MAP_IntDisable(HW::OS_INTERRUPT);
    MAP_TimerDisable(HW::TIMER_BASE, HW::TIMER);
    MAP_TimerDisable(HW::TIMER_BASE, HW::RCOM_TIMER);
}

template <class HW>
__attribute__((optimize("-O3"))) void TivaDccDecoder<HW>::interrupt_handler()
{
    Debug::DccDecodeInterrupts::set(true);
    // get masked interrupt status
    auto status = MAP_TimerIntStatus(HW::TIMER_BASE, true);
    // TODO(balazs.racz): Technically it is possible that the timer reload
    // happens between the event match and the interrupt entry. In this case we
    // will incorrectly add a full cycle to the event length.
    if (status & HW::TIMER_CAP_EVENT)
    {
        //Debug::DccDecodeInterrupts::toggle();
        HW::cap_event_hook();
        MAP_TimerIntClear(HW::TIMER_BASE, HW::TIMER_CAP_EVENT);
        uint32_t raw_new_value;
        raw_new_value = MAP_TimerValueGet(HW::TIMER_BASE, HW::TIMER);
        uint32_t old_value = lastTimerValue_;
        if (raw_new_value > old_value) {
            // Timer has overflowed.
            if (nextSample_ < old_value) {
                nextSample_ += HW::TIMER_MAX_VALUE;
            }
            old_value += HW::TIMER_MAX_VALUE;
            waitSampleForOverflow_ = false;
            Debug::CapTimerOverflow::set(false);
        }
        if (raw_new_value < nextSample_ && !waitSampleForOverflow_) {
            sampleActive_ = true;
            if (nextSample_ <= HW::SAMPLE_PERIOD_CLOCKS) {
                nextSample_ += HW::TIMER_MAX_VALUE;
                waitSampleForOverflow_ = true;
                Debug::CapTimerOverflow::set(true);
            }
            nextSample_ -= HW::SAMPLE_PERIOD_CLOCKS;
        }
        uint32_t new_value = old_value - raw_new_value;
        bool cutout_just_finished = false;
        decoder_.process_data(new_value);
        if (decoder_.before_dcc_cutout())
        {
            prepCutout_ = true;
            HW::dcc_before_cutout_hook();
        }
        // If we are at the second half of the last 1 bit and the
        // value of the input pin is 1, then we cannot recognize when
        // the first half of the cutout bit disappears thus we'll
        // never get the DCC cutout signal. We will therefore start
        // the cutout by hand with a bit of delay.
        else if (decoder_.state() == dcc::DccDecoder::DCC_MAYBE_CUTOUT &&
                 true) //HW::NRZ_Pin::get())
        {
            //Debug::RailcomDriverCutout::set(true);
            set_cap_timer_time();
            set_cap_timer_delay_usec(RAILCOM_CUTOUT_PRE);
            inCutout_ = true;
            cutoutState_ = 0;
        }
        else if (decoder_.state() == dcc::DccDecoder::DCC_CUTOUT)
        {
            //railcomDriver_->start_cutout();
            //inCutout_ = true;
        }
        /// TODO(balazs.racz) recognize middle cutout.
        else if (decoder_.state() == dcc::DccDecoder::DCC_PACKET_FINISHED)
        {
            if (inCutout_) {
                //railcomDriver_->end_cutout();
                inCutout_ = false;
            }
            HW::dcc_packet_finished_hook();
            prepCutout_ = false;
            cutout_just_finished = true;
            // Record packet to send back to userspace
            if (nextPacketData_)
            {
                nextPacketData_->header_raw_data = 0;
                nextPacketData_->dlc = decoder_.packet_length();
                memcpy(nextPacketData_->payload, decoder_.packet_data(),
                    decoder_.packet_length());
                nextPacketData_ = nullptr;
                nextPacketFilled_ = true;
                
                MAP_IntPendSet(HW::OS_INTERRUPT);
            }
        }
        lastTimerValue_ = raw_new_value;
        if (sampleActive_ && HW::NRZ_Pin::get() &&
            !prepCutout_ && !cutout_just_finished)
        {
            sampleActive_ = false;
            // The first positive edge after the sample timer expired (but
            // outside of the cutout).
            railcomDriver_->feedback_sample();
            HW::after_feedback_hook();
        }
    }
}

template <class HW>
__attribute__((optimize("-O3"))) void
TivaDccDecoder<HW>::rcom_interrupt_handler()
{
    Debug::DccDecodeInterrupts::set(true);
    auto status = MAP_TimerIntStatus(HW::TIMER_BASE, true);
    if (status & HW::TIMER_RCOM_MATCH)
    {
        MAP_TimerIntClear(HW::TIMER_BASE, HW::TIMER_RCOM_MATCH);
        // Debug::RailcomDriverCutout::set(false);
        switch (cutoutState_)
        {
            case 0:
            {
                set_cap_timer_delay_usec(RAILCOM_CUTOUT_MID);
                railcomDriver_->start_cutout();
                cutoutState_ = 1;
                break;
            }
            case 1:
            {
                set_cap_timer_delay_usec(RAILCOM_CUTOUT_END);
                railcomDriver_->middle_cutout();
                cutoutState_ = 2;
                break;
            }
            default:
            {
                set_cap_timer_delay_usec(RAILCOM_CUTOUT_END);
                MAP_TimerDisable(HW::TIMER_BASE, HW::RCOM_TIMER);
                railcomDriver_->end_cutout();
                inCutout_ = false;
                break;
            }
        }
    }
    Debug::DccDecodeInterrupts::set(false);
}

template <class HW>
__attribute__((optimize("-O3"))) void TivaDccDecoder<HW>::os_interrupt_handler()
{
    if (nextPacketFilled_) {
        inputData_->advance(1);
        nextPacketFilled_ = false;
        inputData_->signal_condition_from_isr();
    }
    if (!nextPacketData_) {
        if (inputData_->space())
        {
            inputData_->data_write_pointer(&nextPacketData_);
        }
    }
}

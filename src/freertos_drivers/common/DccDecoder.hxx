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
 * \file DccDecoder.hxx
 *
 * Generic device driver for decoding DCC track signal using a timer resource
 * in capture mode.
 *
 * @author Balazs Racz
 * @date 8 April 2020
 */

#include "freertos_drivers/common/RailcomDriver.hxx"
#include "dcc/Receiver.hxx"
#include "dcc/PacketProcessor.hxx"
#include "dcc/packet.h"

/**
  Device driver for decoding a DCC signal using a Timer resource.

  This driver exports a filesystem device node, which will be readable for
  decoded DCC packets (read into struct DCCPacket). All writes fail.

  The additional feature supported by this device driver is that it is able to
  tell a RailcomDriver when the railcom cutout is supposed to start, when we're
  in the middle and when it is over. This is necessary for the correct
  functionality of the railcom driver.

  Usage:

  Define a module for accessing the timer. See the following two examples:
  {\link TivaDccTimerModule }
  {\link Stm32DccTimerModule }.

  In the HwInit.cxx, instantiate the driver with passing the module as a
  template argument. The above example modules themselves take a template
  argument with static definitions.

  Module API:
  - NRZ_Pin
  - TIMER_MAX_VALUE
  - SAMPLE_PERIOD_CLOCKS
  - Q_SIZE
  - TICKS_PER_USEC
  - module_init()
  - module_enable()
  - module_disable()
  - trigger_os_interrupt()
  - dcc_before_cutout_hook()
  - dcc_packet_finished_hook()
  - after_feedback_hook()
  - int_get_and_clear_capture_event()
  - get_capture_counter()
  - int_get_and_clear_delay_event()
  - set_cap_timer_capture()
  - set_cap_timer_time()
  - set_cap_timer_delay_usec()
  - stop_cap_timer_time()

 */
template <class Module> class DccDecoder : public Node
{
public:
    /// Constructor.
    ///
    /// @param name name of device node, e.g. "/dev/dccdecode0"
    /// @param railcom_driver is the associated railcom driver, which will get
    /// the callbacks from the timing of the acquired signal.
    DccDecoder(const char *name, RailcomDriver *railcom_driver);

    ~DccDecoder()
    {
        inputData_->destroy();
    }

    /// Installs a hook that will be called in the interrupt context for each
    /// incoming packet.
    /// @param p the hook interface to be called.
    void set_packet_processor(dcc::PacketProcessor *p)
    {
        packetProcessor_ = p;
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
            if (!decoder_.pkt() && inputData_->space())
            {
                DCCPacket* next;
                inputData_->data_write_pointer(&next);
                decoder_.set_packet(next);
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
    bool select(File *file, int mode) OVERRIDE
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

#ifdef DCC_DECODER_DEBUG
    LogRing<uint16_t, 256> debugLog_;
#endif
    
    typedef DCCPacket input_data_type;
    DeviceBuffer<DCCPacket> *inputData_ {
        DeviceBuffer<DCCPacket>::create(Module::Q_SIZE)};
    bool nextPacketFilled_{false};
    /// Holds the value of the free running timer at the time we captured the
    /// previous edge.
    uint32_t lastTimerValue_;
    /// Holds the timer value when we should be taking an occupancy sample the
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
    /// Counts unique identifiers for DCC packets to be returned.
    uint32_t packetId_ = 0;
    /// How many times did we lose a DCC packet due to no buffer available.
    //uint32_t overflowCount_ {0};

    /// notified for cutout events.
    RailcomDriver *railcomDriver_;

    /// notified for every arrived DCC / MM packet within the interrupt.
    dcc::PacketProcessor* packetProcessor_ = nullptr;

    /// DCC packet decoder state machine and internal state.
    dcc::DccDecoder decoder_ {Module::get_ticks_per_usec()};

    /// How many usec the railcom has before the cutout (measured from the
    /// packet end 1 bit complete)
    static const auto RAILCOM_CUTOUT_PRE = 26;
    /// How many usec the railcom has to the middle of window (measured from the
    /// packet end 1 bit complete)
    static const auto RAILCOM_CUTOUT_MID = 185;
    /// How many usec the railcom has to the end of the window (measured from
    /// the packet end 1 bit complete)
    static const auto RAILCOM_CUTOUT_END = 471;

    DISALLOW_COPY_AND_ASSIGN(DccDecoder);
};

template <class Module>
DccDecoder<Module>::DccDecoder(const char *name, RailcomDriver *railcom_driver)
    : Node(name)
    , railcomDriver_(railcom_driver)
{
    Module::NRZ_Pin::hw_init();
    Module::module_init();
    disable();
}

template <class Module> void DccDecoder<Module>::enable()
{
    disable();

    Module::module_enable();
    Module::set_cap_timer_capture();

    lastTimerValue_ = Module::TIMER_MAX_VALUE;
    nextSample_ = lastTimerValue_ - Module::SAMPLE_PERIOD_CLOCKS;

    if (!decoder_.pkt() && inputData_->space())
    {
        DCCPacket *next;
        inputData_->data_write_pointer(&next);
        decoder_.set_packet(next);
    }
}

template <class Module> void DccDecoder<Module>::disable()
{
    Module::module_disable();
}

template <class Module>
__attribute__((optimize("-O3"))) void DccDecoder<Module>::interrupt_handler()
{
    Debug::DccDecodeInterrupts::set(true);
    if (Module::int_get_and_clear_capture_event())
    {
        // We have a capture event at hand.
        // Debug::DccDecodeInterrupts::toggle();
        uint32_t raw_new_value = Module::get_capture_counter();
        uint32_t old_value = lastTimerValue_;
#ifdef DCC_DECODER_DEBUG
        debugLog_.add(0);
        debugLog_.add(old_value);
        debugLog_.add(raw_new_value);
#endif
        if (raw_new_value > old_value) {
            // Timer has overflowed.
            if (nextSample_ < old_value) {
                nextSample_ += Module::TIMER_MAX_VALUE;
            }
            old_value += Module::TIMER_MAX_VALUE;
            waitSampleForOverflow_ = false;
            Debug::CapTimerOverflow::set(false);
        }
        if (raw_new_value < nextSample_ && !waitSampleForOverflow_) {
            sampleActive_ = true;
            if (nextSample_ <= Module::SAMPLE_PERIOD_CLOCKS)
            {
                nextSample_ += Module::TIMER_MAX_VALUE;
                waitSampleForOverflow_ = true;
                Debug::CapTimerOverflow::set(true);
            }
            nextSample_ -= Module::SAMPLE_PERIOD_CLOCKS;
        }
        uint32_t new_value = old_value - raw_new_value;
#ifdef DCC_DECODER_DEBUG
        debugLog_.add(new_value);
#endif        
        bool cutout_just_finished = false;
        decoder_.process_data(new_value);
        if (decoder_.before_dcc_cutout())
        {
            prepCutout_ = true;
            auto* p = decoder_.pkt();
            if (p)
            {
                p->feedback_key = ++packetId_;
            }
            railcomDriver_->set_feedback_key(packetId_);
            Module::dcc_before_cutout_hook();
        }
        // If we are at the second half of the last 1 bit and the
        // value of the input pin is 1, then we cannot recognize when
        // the first half of the cutout bit disappears thus we'll
        // never get the DCC cutout signal. We will therefore start
        // the cutout by hand with a bit of delay.
        else if (decoder_.state() == dcc::DccDecoder::DCC_MAYBE_CUTOUT &&
            true) // Module::NRZ_Pin::get())
        {
            //Debug::RailcomDriverCutout::set(true);
            Module::set_cap_timer_time();
            Module::set_cap_timer_delay_usec(
                RAILCOM_CUTOUT_PRE + Module::time_delta_railcom_pre_usec());
            inCutout_ = true;
            cutoutState_ = 0;
            if (decoder_.pkt())
            {
                nextPacketFilled_ = true;
            }
            Module::trigger_os_interrupt();
        }
        else if (decoder_.state() == dcc::DccDecoder::DCC_CUTOUT)
        {
            //railcomDriver_->start_cutout();
            //inCutout_ = true;
        }
        else if (decoder_.state() == dcc::DccDecoder::DCC_PACKET_FINISHED)
        {
            Debug::DccPacketFinishedHook::set(true);
            if (inCutout_) {
                //railcomDriver_->end_cutout();
                inCutout_ = false;
            }
            Module::dcc_packet_finished_hook();
            prepCutout_ = false;
            cutout_just_finished = true;
            Debug::DccPacketFinishedHook::set(false);
        }
        lastTimerValue_ = raw_new_value;
        if (sampleActive_ && Module::NRZ_Pin::get() && !prepCutout_ &&
            !cutout_just_finished)
        {
            sampleActive_ = false;
            // The first positive edge after the sample timer expired (but
            // outside of the cutout).
            railcomDriver_->feedback_sample();
            Module::after_feedback_hook();
        }
    }
}

template <class Module>
__attribute__((optimize("-O3"))) void
DccDecoder<Module>::rcom_interrupt_handler()
{
    Debug::DccDecodeInterrupts::set(true);
    if (Module::int_get_and_clear_delay_event())
    {
        // Debug::RailcomDriverCutout::set(false);
        switch (cutoutState_)
        {
            case 0:
            {
                Module::set_cap_timer_delay_usec(
                    RAILCOM_CUTOUT_MID + Module::time_delta_railcom_mid_usec());
                railcomDriver_->start_cutout();
                cutoutState_ = 1;
                break;
            }
            case 1:
            {
                Module::set_cap_timer_delay_usec(
                    RAILCOM_CUTOUT_END + Module::time_delta_railcom_end_usec());
                railcomDriver_->middle_cutout();
                cutoutState_ = 2;
                break;
            }
            default:
            {
                Module::stop_cap_timer_time();
                Module::set_cap_timer_capture();
                railcomDriver_->end_cutout();
                inCutout_ = false;
                break;
            }
        }
    }
    Debug::DccDecodeInterrupts::set(false);
}

template <class Module>
__attribute__((optimize("-O3"))) void DccDecoder<Module>::os_interrupt_handler()
{
    unsigned woken = 0;
    if (nextPacketFilled_)
    {
        if (packetProcessor_)
        {
            packetProcessor_->packet_arrived(decoder_.pkt(), railcomDriver_);
        }
        inputData_->advance(1);
        nextPacketFilled_ = false;
        inputData_->signal_condition_from_isr();
        woken = 1;
        decoder_.set_packet(nullptr);
    }
    if (!decoder_.pkt() && inputData_->space())
    {
        DCCPacket *next;
        inputData_->data_write_pointer(&next);
        decoder_.set_packet(next);
    }
    portYIELD_FROM_ISR(woken);
}

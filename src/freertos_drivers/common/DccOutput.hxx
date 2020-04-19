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
 * \file DccOutput.hxx
 * Common definitions (base classes) for DCC output drivers.
 *
 * @author Balazs Racz
 * @date 19 Apr 2020
 */

#ifndef _FREERTOS_DRIVERS_COMMON_DCCOUTPUT_HXX_
#define _FREERTOS_DRIVERS_COMMON_DCCOUTPUT_HXX_


/// This structure represents a single output channel for a DCC command
/// station. The DCC driver uses these structures, with the business logic
/// filled in by the hardware implementor.
///
/// @param OUTPUT_NUMBER 0,1,... the number of outputs. Each output is
/// independently controlled.
template<int OUTPUT_NUMBER> struct DccOutput {
public:

    /// 1 if the application desires this output to be powered. Set by the
    /// application, read by the driver.
    static std::atomic_uint8_t isOutputControlEnabled_;

    /// 1 if the short detector determined that this output is shorted and thus
    /// should not be powered automatically. Set by the application, read by
    /// the driver. When the application sets it, it is the job of the
    /// application to disable the output.
    static std::atomic_uint8_t isOutputShorted_;

    /// 0 if we should not produce a railcom cutout; 1 for short cutout; 2 for
    /// regular cutout. Set by the application and read by the DCC driver.
    static std::atomic_uint8_t isRailcomCutoutEnabled_;
    
    /// 1 if we are in a railcom cutout currently. Set and cleared by the
    /// driver before calling the start/stop railcom cutout functions.
    static std::atomic_uint8_t isRailcomCutoutActive_;

    /// Called by the driver to decide whether to make this channel participate
    /// in the railcom cutout.
    static bool need_railcom_cutout() {
        return isOutputControlEnabled_ && !isOutputShorted_ &&
            isRailcomCutoutEnabled_ != 0;
    }

    /// Called once after the railcom cutout is done to decide whether this
    /// output should be reenabled.
    static bool should_be_enabled() {
        return isOutputControlEnabled_ && !isOutputShorted_;
    }
    
private:
    /// Private constructor. These objects cannot be initialized and must only
    /// have static members.
    DccOutput();
};

template<int N> std::atomic_uint8_t DccOutput<N>::isOutputControlEnabled_ = 0;
template<int N> std::atomic_uint8_t DccOutput<N>::isOutputShorted_ = 0;
template<int N> std::atomic_uint8_t DccOutput<N>::isRailcomCutoutEnabled_ = 2;
template<int N> std::atomic_uint8_t DccOutput<N>::isRailcomCutoutActive_ = 0;

/// Interface that the actual outputs have to implement in their
/// hardware-specific classes.
template<int N> struct DccOutputDummy : public DccOutput<N> {
public:
    /// Invoked at the beginning of a railcom cutout. @return the number of usec
    /// to wait before invoking phase2.
    static unsigned start_railcom_cutout_phase1(void) {}
    
    /// Invoked at the beginning of a railcom cutout after the delay. @return
    /// number of usec to delay before enabling railcom UART receive.
    static unsigned start_railcom_cutout_phase2(void) {}

    /// Invoked at the end of a railcom cutout. @return the number of usec to
    /// wait before invoking phase2.
    static unsigned stop_railcom_cutout_phase1(void) {}
    
    /// Invoked at the end of a railcom cutout.
    static void stop_railcom_cutout_phase2(void) {}

    /// Called once every packet by the driver, typically before the preamble,
    /// if the output is supposed to be on.
    static void enable_output(void) {}

    /// A dummy output never needs a railcom cutout.
    static bool need_railcom_cutout()
    {
        return false;
    }
};



#endif // _FREERTOS_DRIVERS_COMMON_DCCOUTPUT_HXX_

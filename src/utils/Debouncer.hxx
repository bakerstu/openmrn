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
 * \file Debouncer.hxx
 * Strategy classes for debouncing inputs.
 *
 * A Debouncer is a class that input object will use as a template. The
 * debouncer class may have arbitrary class-local state, and there will be one
 * object of it per input bit. The debouncer will be periodically updated when
 * polling the input state. For each update the debouncer has to decide whether
 * the state has changed (and the new update should be published) or the state
 * has not changed or the update shold be ignored.
 *
 * A debouncer class has to have the following public methods:
 *
 * // A structure that captures the parameters.
 * typedef ... Options;
 *
 * // Constructor from the parameters.
 * explicit Debouncer(const Options& opts);
 *
 *  // returns the last known state
 * bool current_state();
 *
 * // called once after construction
 * void initialize(bool state);
 *
 * // called when an on-network event comes to change the state
 * void override(bool new_state);
 *
 * // returns true if measurement should be published as the new state.
 * bool update_state(bool measurement);
 *
 * @author Balazs Racz
 * @date 13 Jul 2014
 */

#ifndef _UTILS_DEBOUNCER_HXX_
#define _UTILS_DEBOUNCER_HXX_

/** This debouncer will update state if for N consecutive attempts the input
 * value is the same. */
class QuiesceDebouncer
{
public:
    typedef uint8_t Options;

    QuiesceDebouncer(const Options &wait_count)
        : count_(0)
        , waitCount_(wait_count)
        , currentState_(0)
    {
    }

    void initialize(bool state)
    {
        currentState_ = state ? 1 : 0;
        count_ = 0;
    }

    void override(bool new_state)
    {
        initialize(new_state);
    }

    bool current_state()
    {
        return currentState_;
    }

    bool update_state(bool state)
    {
        unsigned new_state = state ? 1 : 0;
        if (new_state == currentState_)
        {
            count_ = 0;
            return false;
        }
        if (++count_ == waitCount_)
        {
            currentState_ = new_state;
            count_ = 0;
            return true;
        }
        return false;
    }

private:
    unsigned count_ : 8;
    unsigned waitCount_ : 8;
    unsigned currentState_ : 1;
};

/** This debouncer will count the ON values and if the number of ON values in a
 * given window exceeds a threshold, then triggers. @todo: there should really
 * be a hysteresis. */
class CountingDebouncer
{
public:
    /// Static parameters for the CountingDebouncer. Passed in at construction
    /// and kept constant after construction.
    typedef struct
    {
        /// Number of samples to keep. Must be between 1 and 32.
        uint8_t window_size;
        /// Number of samples that have to be 1 within the window to consider
        /// the debounce value 1. Must be within 1 and window_size.
        uint8_t min_count;
    } Options;

    /// Constructor. @param opts specified the debouncing options.
    CountingDebouncer(const Options &opts)
        : opts_(opts)
        , lastCount_(0)
        , currentState_(0)
    {
        if (opts_.window_size > 32)
        {
            opts_.window_size = 32;
        }
        if (opts_.window_size == 0)
        {
            opts_.window_size = 1;
        }
        if (opts_.min_count == 0)
        {
            opts_.min_count = 1;
        }
        if (opts_.min_count > opts_.window_size)
        {
            opts_.min_count = opts_.window_size;
        }
    }

    /// Initilalize the debouncer. @param state is the current input state at
    /// initialization time, which will be taken over as is (without debouncing
    /// because we don't have any information based on which to debounce).
    void initialize(bool state)
    {
        currentState_ = state ? 1 : 0;
    }

    /// Overrides the debounced state to a given value, irrespective of what is
    /// on the input pin. @param new_state is the desired state.
    void override(bool new_state)
    {
        initialize(new_state);
    }

    /// @return the current debounced state.
    bool current_state()
    {
        return currentState_;
    }

    /// Callback from the polling loop checking the input state. @param state
    /// is the currently observed input state. @return true if the debounced
    /// state has just transitioned to matching the inbound parameter, false if
    /// the debounced state has not changed (i.e. not yet matching the current
    /// observed state or already previously matching the observed state).
    bool update_state(bool state)
    {
        if (window_ & (1<<(opts_.window_size - 1))) {
            --lastCount_;
        }
        window_ <<= 1;
        if (state) {
            lastCount_++;
            window_ |= 1;
        }
        unsigned new_state = lastCount_ >= opts_.min_count ? 1 : 0;
        if (new_state != currentState_) {
            currentState_ = new_state;
            return true;
        } else {
            return false;
        }
    }

private:
    uint32_t window_{0}; ///< bit-map of last observations. bit 0 is latest.
    Options opts_; ///< options
    unsigned lastCount_ : 6; ///< how many 1 bits we have in the window
    unsigned currentState_ : 1; ///< last known state
};

/** This class acts as a debouncer that uses one momentary input button, and
 * switches the event state at every rising edge of that input button.
 Internally it uses another debouncer to smooth the input button. */
template <class Debouncer> class ToggleDebouncer
{
public:
    typedef typename Debouncer::Options Options;

    ToggleDebouncer(const Options &options)
        : impl_(options)
        , eventState_(0)
    {
    }

    void initialize(bool state)
    {
        // comes from the hardware
        impl_.initialize(state);
    }

    void override(bool state)
    {
        eventState_ = state ? 1 : 0;
    }

    bool current_state()
    {
        // We ignore the local state and just publish the network state.
        return eventState_;
    }

    bool update_state(bool state)
    {
        bool press_changed = impl_.update_state(state);
        // Look for rising edge
        if (press_changed && impl_.current_state())
        {
            // flip desired state and report update
            eventState_ ^= 1;
            return true;
        }
        return false;
    }

private:
    Debouncer impl_;
    unsigned eventState_ : 1; // what the world thinks about the event
};

#endif // _UTILS_DEBOUNCER_HXX_

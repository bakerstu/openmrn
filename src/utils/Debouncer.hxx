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

    QuiesceDebouncer(const Options& wait_count)
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

    bool current_state() {
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

#endif // _UTILS_DEBOUNCER_HXX_

/** \copyright
 * Copyright (c) 2014, Balazs Racz
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
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
 * \file Ewma.hxx
 *
 * Exponentially weighted moving average for computing average transfer speed.
 *
 * @author Balazs Racz
 * @date 2 May 2015
 */

#include <time.h>
#include <stdint.h>

class Ewma {
public:
    Ewma(float alpha = 0.8) : alpha_(alpha) {

    }

    void add_absolute(uint32_t offset) {
        add_diff(offset - lastOffset_);
        lastOffset_ = offset;
    }

    void add_diff(uint32_t bytes) {
        long long t = current_time();
        if (lastMeasurementTimeNsec_) {
            float spd = bytes * 1e9 / (t-lastMeasurementTimeNsec_);
            if (avg_) {
                avg_ = avg_ * alpha_ + spd * (1-alpha_);
            } else {
                avg_ = spd;
            }
        }
        lastMeasurementTimeNsec_ = t;
    }

    float avg() { return avg_; }

private:
    long long current_time() {
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        long long t = ts.tv_sec * 1000000000;
        t += ts.tv_nsec;
        return t;
    }

    float alpha_;
    float avg_{0.0};
    long long lastMeasurementTimeNsec_{0};
    uint32_t lastOffset_{0};
};

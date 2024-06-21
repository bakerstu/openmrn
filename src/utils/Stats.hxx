/** \copyright
 * Copyright (c) 2023, Balazs Racz
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
 * \file Stats.hxx
 *
 * Utility class for collecting statistics.
 *
 * @author Balazs Racz
 * @date 28 Dec 2023
 */

#ifndef _UTILS_STATS_HXX_
#define _UTILS_STATS_HXX_

#include <math.h>
#include <stdint.h>
#include <string>
#include <limits>

class Stats
{
public:
    using FloatType = double;
    using ValueType = int32_t;

    Stats()
    {
        clear();
    }

    // Clear the statistics (erase all data points).
    void clear()
    {
        sum_ = 0;
        count_ = 0;
        qsum_ = 0;
        max_ = std::numeric_limits<ValueType>::min();
    }

    /// Appends a data point to the statistics.
    /// @param value the data point.
    void add(ValueType value)
    {
        ++count_;
        sum_ += value;
        FloatType fval = value;
        qsum_ += fval * fval;
        if (value > max_)
        {
            max_ = value;
        }
    }

    /// @return the average
    FloatType favg()
    {
        return FloatType(sum_) / count_;
    }

    /// @return the average (rounded down to nearest integer).
    int32_t avg()
    {
        return sum_ / count_;
    }

    /// @return the sample standard deviation (uncorrected).
    FloatType stddev()
    {
        // Formula: sqrt(N*qsum - sum^2) / N
        FloatType sum(sum_);
        return sqrt(qsum_ * count_ - sum * sum) / count_;
    }

    /// Creates a half-a-line printout of this stats object for debug purposes.
    std::string debug_string();

private:
    /// Number of samples added.
    uint32_t count_;
    /// Maximum value found since the last clear.
    ValueType max_;
    /// Sum of sample values added.
    int64_t sum_;
    /// Sum of squares of sample values added.
    FloatType qsum_;
};

#endif // _UTILS_STATS_HXX_

/** @copyright
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
 * @file TimeBase.hxx
 *
 * Represents a fast clock's time base against the OS clock.
 *
 * @author Balazs Racz
 * @date 29 Jan 2023
 */

#ifndef _UTILS_TIMEBASE_HXX_
#define _UTILS_TIMEBASE_HXX_

#include <time.h>
#include "os/OS.hxx"

/// Helper class for implementing fast clocks. Represents the fast time against
/// the OS (monotonic) clock.
class TimeBase : protected Atomic
{
public:
    TimeBase()
        : started_(false)
    { }

    /// Synchronizes this time with a different clock.
    /// @param other a different clock. The current clock will take over that
    /// other clocks's state.
    void sync(const TimeBase &other)
    {
        timestamp_ = other.timestamp_;
        seconds_ = other.seconds_;
        rate_ = other.rate_;
        started_ = other.started_;
    }

    /// Get the time as a value of seconds relative to the system epoch.  At the
    /// same time get an atomic matching pair of the rate
    /// @return pair<time in seconds relative to the system epoch, rate>
    std::pair<time_t, int16_t> time_and_rate_quarters()
    {
        AtomicHolder h(this);
        return std::make_pair(time(), rate_);
    }

    /// Get the difference in time scaled to real time.
    /// @param t1 time 1 to compare
    /// @param t2 time 2 to compare
    /// @return (t1 - t2) scaled to real time.
    time_t compare_realtime(time_t t1, time_t t2)
    {
        int rate = rate_;
        if (rate == 0)
        {
            // avoid divid by zero error
            rate = 4;
        }
        return ((t1 - t2) * 4) / rate;
    }

    /// Get the time as a value of seconds relative to the system epoch.
    /// @return time in seconds relative to the system epoch
    time_t time()
    {
        AtomicHolder h(this);
        if (started_)
        {
            long long elapsed = OSTime::get_monotonic() - timestamp_;
            elapsed = ((elapsed * std::abs(rate_)) + 2) / 4;

            time_t diff = (time_t)NSEC_TO_SEC_ROUNDED(elapsed);
            return (rate_ < 0) ? seconds_ - diff : seconds_ + diff;
        }
        else
        {
            // clock is stopped, time is not progressing
            return seconds_;
        }
    }

    /// Get the time as a standard struct tm.
    /// @param result a pointer to the structure that will hold the result
    /// @return pointer to the passed in result on success, nullptr on failure
    struct tm *gmtime_r(struct tm *result)
    {
        time_t now = time();
        return ::gmtime_r(&now, result);
    }

    /// Get the date (month/day).
    /// @param month month (1 to 12)
    /// @param day day of month (1 to 31)
    /// @return 0 upon success, else -1 on failure
    int date(int *month, int *day)
    {
        struct tm tm;
        if (gmtime_r(&tm) == nullptr)
        {
            return -1;
        }
        *month = tm.tm_mon + 1;
        *day = tm.tm_mday;
        return 0;
    }

    /// Get the day of the week.
    /// @returns day of the week (0 - 6, Sunday - Saturday) upon success,
    ///          else -1 on failure
    int day_of_week()
    {
        struct tm tm;
        if (gmtime_r(&tm) == nullptr)
        {
            return -1;
        }
        return tm.tm_wday;
    }

    /// Get the day of the year.
    /// @returns day of the year (0 - 365) upon success, else -1 on failure
    int day_of_year()
    {
        struct tm tm;
        if (gmtime_r(&tm) == nullptr)
        {
            return -1;
        }
        return tm.tm_yday;
    }

    /// Get the year.
    /// @returns year (0 - 4095) upon success, else -1 on failure
    int year()
    {
        struct tm tm;
        if (gmtime_r(&tm) == nullptr)
        {
            return -1;
        }
        return tm.tm_year + 1900;
    }

    /// Report the clock rate as a 12-bit fixed point number
    /// (-512.00 to 511.75).
    /// @return clock rate
    int16_t get_rate_quarters()
    {
        return rate_;
    }

    /// Test of the clock is running. Running backwards (negative rate) also
    /// qualifies as running.
    /// @return true if running, else false
    bool is_running()
    {
        AtomicHolder h(this);
        return rate_ != 0 && started_;
    }

    /// Test of the clock is started (rate could still be 0).
    /// @return true if started, else false
    bool is_started()
    {
        return started_;
    }

    /// Convert fast clock period to a period in real nsec. Result will
    /// preserve sign.
    /// @param rate rate to use in conversion
    /// @param fast_sec period in seconds in fast clock time
    /// @param real_nsec period in nsec
    /// @return true if successfull, else false if clock is not running
    bool fast_sec_to_real_nsec_period(
        int16_t rate, time_t fast_sec, long long *real_nsec)
    {
        if (rate != 0 && rate >= -2048 && rate <= 2047)
        {
            *real_nsec =
                ((SEC_TO_NSEC(std::abs(fast_sec)) * 4) + (std::abs(rate) / 2)) /
                rate;
            if (fast_sec < 0)
            {
                *real_nsec = -(*real_nsec);
            }
            return true;
        }
        else
        {
            return false;
        }
    }

    /// Convert period in real nsec to a fast clock period. Result will
    /// preserve sign.
    /// @param rate rate to use in conversion
    /// @param real_nsec period in nsec
    /// @param fast_sec period in seconds in fast clock time
    /// @return true if successfull, else false if clock is not running
    bool real_nsec_to_fast_sec_period(
        int16_t rate, long long real_nsec, time_t *fast_sec)
    {
        if (rate != 0 && rate >= -2048 && rate <= 2047)
        {
            *fast_sec = (std::abs(NSEC_TO_SEC(real_nsec * rate)) + 2) / 4;
            if ((real_nsec < 0 && rate > 0) || (real_nsec >= 0 && rate < 0))
            {
                *fast_sec = -(*fast_sec);
            }
            return true;
        }
        else
        {
            return false;
        }
    }

    /// Convert a fast time to absolute nsec until it will occur.
    ///
    /// Caution!!! Not an atomic operation if called from a different thread or
    /// executor than the thread or executor being used by this object.
    ///
    /// @param fast_sec seconds in rate time
    /// @param real_nsec period in nsec until it will occure
    /// @return true if successfull, else false if clock is not running
    bool real_nsec_until_fast_time_abs(time_t fast_sec, long long *real_nsec)
    {
        if (fast_sec_to_real_nsec_period_abs(fast_sec - seconds_, real_nsec))
        {
            *real_nsec += timestamp_;
            long long monotonic = OSTime::get_monotonic();
            *real_nsec -= monotonic;
            return true;
        }
        else
        {
            return false;
        }
    }

    /// Convert fast clock absolute (negative or positive) period to a positive
    /// (absolute) period in real nsec.
    ///
    /// Caution!!! Not an atomic operation if called from a different thread or
    /// executor than the thread or executor being used by this object.
    ///
    /// @param sec period in seconds in fast clock time
    /// @param real_nsec period in nsec
    /// @return true if successfull, else false if clock is not running
    bool fast_sec_to_real_nsec_period_abs(time_t fast_sec, long long *real_nsec)
    {
        if (fast_sec_to_real_nsec_period(rate_, fast_sec, real_nsec))
        {
            *real_nsec = std::abs(*real_nsec);
            return true;
        }
        else
        {
            return false;
        }
    }

protected:
    /// OS time at the last time update.
    long long timestamp_ {OSTime::get_monotonic()};
    /// Clock time at the last time update.
    time_t seconds_ {0};
    int16_t rate_ {0};        ///< effective clock rate

    uint16_t started_ : 1; ///< true if clock is started
};

#endif // _UTILS_TIMEBASE_HXX_

/** @copyright
 * Copyright (c) 2018, Stuart W. Baker
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
 * @file BroadcastTimeDefs.cxx
 *
 * Static definitions for implementations of the OpenLCB Broadcast Time
 * Protocol.
 *
 * @author Stuart W. Baker
 * @date 30 July 2022
 */

#include "BroadcastTimeDefs.hxx"

#include <string>

#include "utils/format_utils.hxx"

namespace openlcb
{

extern "C"
{
// normally requires _GNU_SOURCE
char *strptime(const char *, const char *, struct tm *);
}

//
// BroadcastTimeDefs::time_to_string()
//
std::string BroadcastTimeDefs::time_to_string(int hour, int min)
{
    if (hour < 0 || hour > 23)
    {
        hour = 0;
    }
    if (min < 0 || min > 59)
    {
        min = 0;
    }

    struct tm tm;
    tm.tm_hour = hour;
    tm.tm_min = min;
    char value[6];
    if (strftime(value, 6, "%R", &tm) != 0)
    {
        return value;
    }
    else
    {
        return "Error";
    }
}

//
// BroadcastTimeDefs::rate_quarters_to_string()
//
std::string BroadcastTimeDefs::rate_quarters_to_string(int16_t rate)
{
    std::string result;
    if (rate < 0)
    {
        result.push_back('-');
        rate = -rate;
    }
    uint16_t whole = rate >> 2;
    uint16_t frac = rate & 0x3;

    result += integer_to_string(whole);

    switch (frac)
    {
        default:
        case 0:
            result.append(".00");
            break;
        case 1:
            result.append(".25");
            break;
        case 2:
            result.append(".50");
            break;
        case 3:
            result.append(".75");
            break;
    }
    return result;
}

//
// BroadcastTimeDefs::date_to_string()
//
std::string BroadcastTimeDefs::date_to_string(int year, int month, int day)
{
    struct tm tm = {};
    tm.tm_year = year - 1900;
    tm.tm_mon = month - 1;
    tm.tm_mday = day;
    char value[13];
    if (strftime(value, 13, "%b %e, %Y", &tm) != 0)
    {
        return value;
    }
    else
    {
        return "Error";
    }
}

//
// BroadcastTimeDefs::string_to_time()
//
bool BroadcastTimeDefs::string_to_time(
    const std::string &stime, int *hour, int *min)
{
    struct tm tm;
    if (strptime(stime.c_str(), "%R", &tm) == nullptr)
    {
        return false;
    }

    if (hour)
    {
        *hour = tm.tm_hour;
    }
    if (min)
    {
        *min = tm.tm_min;
    }
    return true;
}

//
// BroadcastTimeDefs::string_to_rate_quaraters()
//
int16_t BroadcastTimeDefs::string_to_rate_quarters(const std::string &srate)
{
    // Note: The obvious way to simplify this method would be to make use of
    //       strtof(). However, avoiding strtof() saves nearly 8K of code space
    //       (ARMv7m, -Os).

    size_t decimal = srate.find('.');

    char *end;
    long rate;
    bool negative = false;

    // Get the whole number portion.
    {
        std::string whole_str = srate.substr(0, decimal);
        rate = strtol(whole_str.c_str(), &end, 0);
        if (end == whole_str.c_str())
        {
            // None of the string processed.
            return 4;
        }
        negative = whole_str.find('-') != std::string::npos;
    }
    rate *= 4;

    // Get the fractional portion
    if (decimal != std::string::npos)
    {
        std::string decimal_str = srate.substr(decimal + 1);
        decimal_str.resize(2, '0');
        long rate_frac = strtol(decimal_str.c_str(), &end, 10);
        if (end != decimal_str.c_str())
        {
            // Value 12 is chosen so that we round up when not an exact multiple
            // of 0.25.
            while (rate_frac > 12)
            {
                rate += negative ? -1 : 1;
                rate_frac -= 25;
            }
        }
    }

    // Boundary checks
    if (rate < -2048)
    {
        // set to minimum valid rate
        rate = -2048;
    }
    else if (rate > 2047)
    {
        // set to maximum valid rate
        rate = 2047;
    }
    return rate;
}

//
// BroadcastTimeDefs::string_to_date()
//
bool BroadcastTimeDefs::string_to_date(
    const std::string &sdate, int *year, int *month, int *day)
{
    struct tm tm;
    memset(&tm, 0, sizeof(tm));
    if (strptime(sdate.c_str(), "%b %e, %Y", &tm) == nullptr)
    {
        return false;
    }

    // newlib does not have the proper boundary checking for strptime().
    // Therefore we use mktime() to determine if the time we have is really
    // valid or not. In newlib, mktime() can actually correct some invalid
    // struct tm values by making some educated guesses.
    //
    // While glibc does have proper boundary checking for strptime(), it
    // can still use mktime() to correct some invalid struct tm values by
    // making some educated guesses.
    time_t t = mktime(&tm);

    // newlib does not correctly set the errno value when mktime()
    // encounters an error. Instead it "only" returns -1, which is technically
    // a valid time. We are counting on the fact that we zeroed out the struct
    // tm above, and subsequently -1 cannot be an expected result.
    if (t == (time_t)-1)
    {
        return false;
    }

    if (tm.tm_year < (0 - 1900) || tm.tm_year > (4095 - 1900))
    {
        // Out of range for openlcb.
        return false;
    }
    *year = tm.tm_year + 1900;
    *month = tm.tm_mon + 1;
    *day = tm.tm_mday;
    return true;
}

bool BroadcastTimeDefs::canonicalize_rate_string(std::string *srate)
{
    int r = string_to_rate_quarters(*srate);
    string out = rate_quarters_to_string(r);
    if (out != *srate)
    {
        *srate = std::move(out);
        return true;
    }
    return false;
}

bool BroadcastTimeDefs::canonicalize_date_string(std::string *sdate)
{
    int y = 1970, m = 1, d = 1;
    string_to_date(*sdate, &y, &m, &d);
    string out = date_to_string(y, m, d);
    if (out != *sdate)
    {
        *sdate = std::move(out);
        return true;
    }
    return false;
}

bool BroadcastTimeDefs::canonicalize_time_string(std::string *stime)
{
    int h = 0, m = 0;
    string_to_time(*stime, &h, &m);
    string out = time_to_string(h, m);
    if (out != *stime)
    {
        *stime = std::move(out);
        return true;
    }
    return false;
}

} // namespace openlcb

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

/// An array that contains all Mmm abbreviations of the months of the year.
static const char *MONTHS[12] =
{
    "Jan",
    "Feb",
    "Mar",
    "Apr",
    "May",
    "Jun",
    "Jul",
    "Aug",
    "Sep",
    "Oct",
    "Nov",
    "Dec"
};

/// Trim the leading "left" space of the C string.
/// @param str C string to trim
/// @param offset number of characters to ignore before looking.
/// @return pointer to the first non space character of the string or the null
///         terminating character if the end of the string is reached
static const char *ltrim(const char *str, size_t offset = 0)
{
    while (*str != '\0')
    {
        if (offset)
        {
            --offset;
        }
        else if (!std::isspace(*str))
        {
            break;
        }
        ++str;
    }
    return str;
}

/// Find the next instance of a character in a string.
/// @param str C string to search
/// @param c character value to search for
/// @return pointer to the first instance of the character in the string or the
///         null terminating character if the end of the string is reached
static const char *c_str_find(const char *str, char c)
{
    while (*str != '\0' && *str != c)
    {
        ++str;
    }
    return str;
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

    std::string stime;
    stime.reserve(5);
    if (hour < 10)
    {
        stime.append(1, '0');
    }
    stime.append(integer_to_string(hour));
    stime.append(":");
    if (min < 10)
    {
        stime.append(1, '0');
    }
    stime.append(integer_to_string(min));

    return stime;
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
    // Note: The obvious way to simplify this method would be to make use of
    //       strftime(). However, avoiding strftime() saves over 3K of code
    //       space (ARMv7m, -Os).

    struct tm tm;
    memset(&tm, 0, sizeof(tm));
    tm.tm_year = year - 1900;
    tm.tm_mon = month - 1;
    tm.tm_mday = day;

    // We use mktime() to determine if the time we have is really valid or not.
    // In newlib, mktime() can actually correct some invalid struct tm values by
    // making some educated guesses.
    //
    // glibc can also use mktime() to correct some invalid struct tm values by
    // making some educated guesses.
    time_t t = mktime(&tm);
    tm.tm_year += 1900;

    // newlib does not correctly set the errno value when mktime()
    // encounters an error. Instead it "only" returns -1, which is technically
    // a valid time. We are counting on the fact that we zeroed out the struct
    // tm above, and subsequently -1 cannot be an expected result.
    if (t == (time_t)-1 || tm.tm_year < 0 || tm.tm_year > 4095)
    {
        return "Error";
    }
    HASSERT(tm.tm_mon < 12);
    HASSERT(tm.tm_mday >= 1 && tm.tm_mday <= 31);

    std::string sdate;
    sdate.reserve(12);
    sdate.append(MONTHS[tm.tm_mon]);
    sdate.append(integer_to_string(tm.tm_mday, 3));
    sdate.append(", ");
    sdate.append(integer_to_string(tm.tm_year));

    return sdate;
}

//
// BroadcastTimeDefs::string_to_time()
//
bool BroadcastTimeDefs::string_to_time(
    const std::string &stime, int *hour, int *min)
{
    // Note: The obvious way to simplify this method would be to make use of
    //       strptime(). However, avoiding strptime() saves nearly 2K of code
    //       space (ARMv7m, -Os).

    const char *p = stime.c_str();
    char *p_new;
    int h_value = 0;
    int m_value = 0;
    bool result = true;

    h_value = strtol(p, &p_new, 10);
    p = ltrim(p_new);
    if (p[0] == ':')
    {
        ++p;
        m_value = strtol(p, nullptr, 10);

        if (h_value < 0 || h_value > 23 || m_value < 0 || m_value > 59)
        {
            // Invalid string, out of range.
            result = false;
        }
    }
    else
    {
        // Invalid string, no colon.
        result = false;
    }

    if (hour)
    {
        *hour = result ? h_value : 0;
    }
    if (min)
    {
        *min = result ? m_value : 0;
    }

    return result;
}

//
// BroadcastTimeDefs::string_to_rate_quaraters()
//
int16_t BroadcastTimeDefs::string_to_rate_quarters(const std::string &srate)
{
    // Note: The obvious way to simplify this method would be to make use of
    //       strtof(). However, avoiding strtof() saves nearly 8K of code space
    //       (ARMv7m, -Os).

    const char *p = srate.c_str();
    char *p_new;
    int rate;
    bool negative = false;

    // Get the whole number portion.
    p = ltrim(p);
    if (*p == '-')
    {
        negative = true;
        p = ltrim(p + 1);
    }
    rate = strtol(p, &p_new, 0);
    if (p_new == p && *p != '.')
    {
        // None of the string processed, default to 1:1 rate.
        return 4;
    }
    rate *= negative ? -4 : 4;

    // Get the fractional portion
    p = ltrim(p_new);
    if (*p == '.')
    {
        ++p;
        char tmp[3] = {p[0], p[1], '\0'};
        int rate_frac = strtol(tmp, &p_new, 10);
        if (p_new != p)
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

    rate = std::max(rate, -2048);
    rate = std::min(rate, 2047);

    return rate;
}

//
// BroadcastTimeDefs::string_to_date()
//
bool BroadcastTimeDefs::string_to_date(
    const std::string &sdate, int *year, int *month, int *day)
{
    // Note: The obvious way to simplify this method would be to make use of
    //       strptime(). However, avoiding strptime() saves nearly 2K of code
    //       space (ARMv7m, -Os).

    const char *m_str = ltrim(sdate.c_str());
    const char *y_str = c_str_find(m_str, ',');
    if (*y_str != ',')
    {
        // Invalid format, no comma
        return false;
    }
    y_str = ltrim(y_str + 1);
    if (*y_str == '\0')
    {
        // Invalid format, no year.
        return false;
    }

    const char *d_str = ltrim(m_str, 3);
    if (d_str > y_str)
    {
        // Likely no space between month and day.
        d_str = m_str + 3;
    }

    struct tm tm;
    memset(&tm, 0, sizeof(tm));

    // Month.
    {
        // Get a sub-string that matches the Mmm case format.
        char Mmm_str[4] =
        {
            (char)std::toupper(m_str[0]), (char)std::tolower(m_str[1]),
            (char)std::tolower(m_str[2]), '\0'
        };

        unsigned m_idx;
        for (m_idx = 0; m_idx < ARRAYSIZE(MONTHS); ++m_idx)
        {
            if (!strcmp(Mmm_str, MONTHS[m_idx]))
            {
                break;
            }
        }
        if (m_idx >= ARRAYSIZE(MONTHS))
        {
            // Invalid month.
            return false;
        }
        tm.tm_mon = m_idx;
    }

    // Day.
    {
        int d_value = strtol(d_str, nullptr, 10);
        if (d_value < 1 || d_value > 31)
        {
            // Invalid day.
            return false;
        }
        tm.tm_mday = d_value;
    }

    // Year.
    tm.tm_year = strtol(y_str, nullptr, 10);
    tm.tm_year -= 1900;

    // We use mktime() to determine if the time we have is really valid or not.
    // In newlib, mktime() can actually correct some invalid struct tm values by
    // making some educated guesses.
    //
    // glibc can also use mktime() to correct some invalid struct tm values by
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

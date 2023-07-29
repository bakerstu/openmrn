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
 * @file BroadcastTime.cxx
 *
 * Implementation of a Broadcast Time Protocol Interface.
 *
 * @author Stuart W. Baker
 * @date 4 November 2018
 */

#ifndef _POSIX_C_SOURCE
#define _POSIX_C_SOURCE 200112L
#endif

#include "openlcb/BroadcastTime.hxx"

namespace openlcb
{

//
// BroadcastTimeClient::clear_timezone
//
void BroadcastTime::clear_timezone()
{
#ifndef ESP32
        setenv("TZ", "GMT0", 1);
        tzset();
#endif
}

extern "C"
{
// normally requires _GNU_SOURCE
char *strptime(const char *, const char *, struct tm *);
}

//
// BroadcastTimeClient::set_data_year_str
//
void BroadcastTime::set_date_year_str(const char *date_year)
{
    struct tm tm;
    memset(&tm, 0, sizeof(tm));
    if (strptime(date_year, "%b %e, %Y", &tm) == nullptr)
    {
        // Invalid result, don't set the data/year.
        return;
    }

// It is assumed that this combination of compiler predefines is sufficent to
// know that the compiler is GCC and [likely] using newlib for libc.
#if defined(__GNUC__) && defined(__FreeRTOS__)
    // newlib does not have proper boundary checking for strptime(). Therefore
    // we will use mktime() to determine if the time we have is really valid
    // or not. In newlib, mktime() can actually correct some invalid struct tm
    // values my making some educated guesses.
    //
    // Also, newlib does not correctly set the errno value when mktime()
    // encounters an error. Instead it "only" returns -1, which is technically
    // a valie time. We are counting on the fact that we zero'd out the
    // struct tm above, and subsequently -1 cannot be an expected result.
    time_t t = mktime(&tm);
    if (t == (time_t)-1)
    {
        // Not a valid (or correctable) struct tm input.
        return;
    }
#endif

    // If we have gotten here, our libc implementation should have the
    // the struct tm filled in with a valid result.
    if (tm.tm_year >= (0 - 1900) && tm.tm_year <= (4095 - 1900))
    {
        // date valid
        set_date(tm.tm_mon + 1, tm.tm_mday);
        set_year(tm.tm_year + 1900);
    }
}

} // namespace openlcb

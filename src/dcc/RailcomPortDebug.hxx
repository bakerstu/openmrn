/** \copyright
 * Copyright (c) 2015, Balazs Racz
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
 * \file RailcomPortDebug.hxx
 *
 * Flows that can attach to Railcom Hubs and perform various debugging outputs.
 *
 * @author Balazs Racz
 * @date 6 Feb 2016
 */

#ifndef _DCC_RAILCOMPORTDEBUG_HXX
#define _DCC_RAILCOMPORTDEBUG_HXX

#include "dcc/RailcomHub.hxx"

namespace dcc
{

/** Registers as a member ofthe railcom hub. Formats all incoming railcom
 * packet in a human-visible way, and puts into the logging facility. */
class RailcomPrintfFlow : public dcc::RailcomHubPortInterface
{
public:
    RailcomPrintfFlow(dcc::RailcomHubFlow *source)
        : parent_(source)
    {
        source->register_port(this);
    }

    ~RailcomPrintfFlow()
    {
        parent_->unregister_port(this);
    }

private:
    string display_railcom_data(const uint8_t *data, int len)
    {
        static char buf[200];
        int ofs = 0;
        HASSERT(len <= 6);
        for (int i = 0; i < len; ++i)
        {
            ofs += sprintf(buf + ofs, "0x%02x (0x%02x), ", data[i],
                dcc::railcom_decode[data[i]]);
        }
        uint8_t type = (dcc::railcom_decode[data[0]] >> 2);
        if (len == 2)
        {
            uint8_t payload = dcc::railcom_decode[data[0]] & 0x3;
            payload <<= 6;
            payload |= dcc::railcom_decode[data[1]];
            switch (type)
            {
                case dcc::RMOB_ADRLOW:
                    ofs += sprintf(buf + ofs, "adrlow=%d", payload);
                    break;
                case dcc::RMOB_ADRHIGH:
                    ofs += sprintf(buf + ofs, "adrhigh=%d", payload);
                    break;
                case dcc::RMOB_EXT:
                    ofs += sprintf(buf + ofs, "ext=%d", payload);
                    break;
                case dcc::RMOB_DYN:
                    ofs += sprintf(buf + ofs, "dyn=%d", payload);
                    break;
                case dcc::RMOB_SUBID:
                    ofs += sprintf(buf + ofs, "subid=%d", payload);
                    break;
                default:
                    ofs += sprintf(buf + ofs, "type-%d=%d", type, payload);
            }
        }
        return string(buf, ofs);
    }

    void send(Buffer<dcc::RailcomHubData> *d, unsigned prio) OVERRIDE
    {
        AutoReleaseBuffer<dcc::RailcomHubData> rb(d);
        dcc::Feedback &fb = *d->data();
        if (fb.feedbackKey <= 1000)
            return;
        if (fb.ch1Size && fb.channel != 0xff)
        {
            LOG(INFO, "Railcom %x CH1 data(%" PRIu32 "): %s", fb.channel,
                fb.feedbackKey,
                display_railcom_data(fb.ch1Data, fb.ch1Size).c_str());
        }
        if (fb.ch2Size && fb.channel != 0xff)
        {
            LOG(INFO, "Railcom %x CH2 data(%" PRIu32 "): %s", fb.channel,
                fb.feedbackKey,
                display_railcom_data(fb.ch2Data, fb.ch2Size).c_str());
        }
    }

    dcc::RailcomHubFlow *parent_;
};

} // namespace dcc

#endif // _DCC_RAILCOMPORTDEBUG_HXX

/** \copyright
 * Copyright (c) 2026, Balazs Racz
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
 * \file FilteringDirectHub.hxx
 *
 * DirectHub implementation that performs addressed message filtering for openlcb.
 *
 * @author Balazs Racz
 * @date 28 Jul 2026
 */

#ifndef _OPENLCB_FILTERINGDIRECTHUB_HXX_
#define _OPENLCB_FILTERINGDIRECTHUB_HXX_

#include <unordered_set>

#include "openlcb/CanFilter.hxx"
#include "utils/DirectHubImpl.hxx"
#include "utils/gc_format.h"

namespace openlcb
{

/// Helper function to extract const struct can_frame& from payload type T.
inline const struct can_frame &get_can_frame(const CanHubData &payload)
{
    return payload.frame();
}

inline const struct can_frame &get_can_frame(const struct can_frame &payload)
{
    return payload;
}

/**
 * DirectHub implementation that performs OpenLCB addressed message filtering.
 */
template <class T = CanHubData>
class FilteringDirectHubImpl : public DirectHubImpl<T>
{
public:
    FilteringDirectHubImpl(ExecutorBase *e)
        : DirectHubImpl<T>(e)
        , isFiltering_(true)
    {
    }

    FilteringDirectHubImpl(DirectHubService *s)
        : DirectHubImpl<T>(s)
        , isFiltering_(true)
    {
    }

    /// Sets whether filtering is enabled.
    /// @param is_filtering true to enable filtering, false for legacy behavior.
    void set_filtering(bool is_filtering)
    {
        isFiltering_ = is_filtering;
    }

    /// Sets a port to be promiscuous.
    /// @param port the port to set.
    /// @param is_promiscuous true to enable promiscuous mode, false to disable.
    void set_port_promiscuous(DirectHubPort<T> *port, bool is_promiscuous)
    {
        OSMutexLock l(&lock_);
        if (is_promiscuous)
        {
            promiscuousPorts_.insert(port);
        }
        else
        {
            promiscuousPorts_.erase(port);
        }
    }

    using DirectHubImpl<T>::unregister_port;

    void unregister_port(DirectHubPort<T> *port, Notifiable *done) override
    {
        filter_.remove_port(reinterpret_cast<uintptr_t>(port));
        {
            OSMutexLock l(&lock_);
            promiscuousPorts_.erase(port);
        }
        DirectHubImpl<T>::unregister_port(port, done);
    }

    void do_send() override
    {
        if (isFiltering_)
        {
            prepare_filter();
        }
        DirectHubImpl<T>::do_send();
    }

    bool should_send_to(DirectHubPort<T> *p) override
    {
        if (!isFiltering_)
        {
            return DirectHubImpl<T>::should_send_to(p);
        }

        bool is_promisc;
        {
            OSMutexLock l(&lock_);
            is_promisc = promiscuousPorts_.count(p) > 0;
        }

        return filter_.is_matching(reinterpret_cast<uintptr_t>(p), is_promisc);
    }

private:
    void prepare_filter()
    {
        if (this->msg_.payload_)
        {
            filter_.prepare_packet(
                get_can_frame(*this->msg_.payload_->data()),
                reinterpret_cast<uintptr_t>(this->msg_.source_));
        }
    }

    CanFilter filter_;
    bool isFiltering_{true};
    OSMutex lock_;
    std::unordered_set<DirectHubPort<T> *> promiscuousPorts_;
};

typedef FilteringDirectHubImpl<CanHubData> FilteringCanDirectHub;

/**
 * DirectHub implementation for GridConnect byte streams (uint8_t[]) that
 * performs OpenLCB addressed message filtering by parsing GridConnect ASCII frames.
 */
template <>
class FilteringDirectHubImpl<uint8_t[]> : public DirectHubImpl<uint8_t[]>
{
public:
    FilteringDirectHubImpl(ExecutorBase *e)
        : DirectHubImpl<uint8_t[]>(e)
        , isFiltering_(true)
    {
    }

    FilteringDirectHubImpl(DirectHubService *s)
        : DirectHubImpl<uint8_t[]>(s)
        , isFiltering_(true)
    {
    }

    /// Sets whether filtering is enabled.
    /// @param is_filtering true to enable filtering, false for legacy behavior.
    void set_filtering(bool is_filtering)
    {
        isFiltering_ = is_filtering;
    }

    /// Sets a port to be promiscuous.
    /// @param port the port to set.
    /// @param is_promiscuous true to enable promiscuous mode, false to disable.
    void set_port_promiscuous(DirectHubPort<uint8_t[]> *port, bool is_promiscuous)
    {
        OSMutexLock l(&lock_);
        if (is_promiscuous)
        {
            promiscuousPorts_.insert(port);
        }
        else
        {
            promiscuousPorts_.erase(port);
        }
    }

    using DirectHubImpl<uint8_t[]>::unregister_port;

    void unregister_port(DirectHubPort<uint8_t[]> *port, Notifiable *done) override
    {
        filter_.remove_port(reinterpret_cast<uintptr_t>(port));
        {
            OSMutexLock l(&lock_);
            promiscuousPorts_.erase(port);
        }
        DirectHubImpl<uint8_t[]>::unregister_port(port, done);
    }

    void do_send() override
    {
        if (isFiltering_)
        {
            prepare_filter();
        }
        DirectHubImpl<uint8_t[]>::do_send();
    }

    bool should_send_to(DirectHubPort<uint8_t[]> *p) override
    {
        if (!isFiltering_)
        {
            return DirectHubImpl<uint8_t[]>::should_send_to(p);
        }

        bool is_promisc;
        {
            OSMutexLock l(&lock_);
            is_promisc = promiscuousPorts_.count(p) > 0;
        }

        return filter_.is_matching(reinterpret_cast<uintptr_t>(p), is_promisc);
    }

private:
    void prepare_filter()
    {
        auto &buf = msg_.buf_;
        if (buf.size() == 0)
        {
            return;
        }

        uint8_t *p;
        unsigned available;
        buf.head()->get_read_pointer(buf.skip(), &p, &available);
        if (*p != ':')
        {
            return;
        }

        std::string assembled;
        const char *text_packet;
        if (available == buf.size())
        {
            assembled.assign((const char *)p, available);
            text_packet = assembled.c_str();
        }
        else
        {
            buf.append_to(&assembled);
            text_packet = assembled.c_str();
        }

        struct can_frame frame;
        if (gc_format_parse(text_packet, &frame) == 0)
        {
            filter_.prepare_packet(frame, reinterpret_cast<uintptr_t>(msg_.source_));
        }
    }

    CanFilter filter_;
    bool isFiltering_{true};
    OSMutex lock_;
    std::unordered_set<DirectHubPort<uint8_t[]> *> promiscuousPorts_;
};

typedef FilteringDirectHubImpl<uint8_t[]> FilteringByteDirectHub;

} // namespace openlcb

#endif // _OPENLCB_FILTERINGDIRECTHUB_HXX_

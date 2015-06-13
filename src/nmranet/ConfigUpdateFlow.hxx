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
 * \file ConfigUpdateFlow.hxx
 *
 * Implementation of the notification flow for all config update
 * listeners. This flow calls each update listener and performs the necessary
 * actions.
 *
 * @author Balazs Racz
 * @date 13 June 2015
 */

#ifndef _NMRANET_CONFIGUPDATEFLOW_HXX_
#define _NMRANET_CONFIGUPDATEFLOW_HXX_

#include "utils/ConfigUpdateListener.hxx"
#include "utils/ConfigUpdateService.hxx"
#include "executor/StateFlow.hxx"

namespace nmranet
{

class ConfigUpdateFlow : public StateFlowBase,
                         public ConfigUpdateService,
                         private Atomic
{
public:
    ConfigUpdateFlow(Service *service)
        : StateFlowBase(service)
        , nextRefresh_(listeners_.begin())
        , fd_(-1)
    {
    }

    void init(const char *path)
    {
        if (!path)
        {
            fd_ = -1;
        }
        else
        {
            fd_ = ::open(path, O_RDONLY);
            HASSERT(fd_ >= 0);
        }
        trigger_update();
        isInitialLoad_ = 1;
    }

    void TEST_set_fd(int fd)
    {
        fd_ = fd;
    }

    void trigger_update()
    {
        AtomicHolder h(this);
        nextRefresh_ = listeners_.begin();
        isInitialLoad_ = 0;
        needsReboot_ = 0;
        needsReInit_ = 0;
        if (is_state(exit().next_state()))
        {
            start_flow(STATE(call_next_listener));
        }
    }

    void register_update_listener(ConfigUpdateListener *listener) OVERRIDE
    {
        AtomicHolder h(this);
        listeners_.push_front(listener);
    }

    void unregister_update_listener(ConfigUpdateListener *listener) OVERRIDE
    {
        AtomicHolder h(this);
        auto it = listeners_.begin();
        while (it != listeners_.end() && it.operator->() != listener)
        {
            ++it;
        }
        if (it != listeners_.end())
        {
            listeners_.erase(it);
        }
        // We invalidated the iterators due to the erase.
        nextRefresh_ = listeners_.begin();
    }

private:
    Action call_next_listener()
    {
        ConfigUpdateListener *l = nullptr;
        {
            AtomicHolder h(this);
            if (nextRefresh_ == listeners_.end())
            {
                /// TODO(balazs.racz) apply the changes reported.
                return exit();
            }
            l = nextRefresh_.operator->();
        }
        if (fd_ < 0)
        {
            DIE("CONFIG_FILENAME not specified, or init() was not called, but "
                "there are configuration listeners.");
        }
        ConfigUpdateListener::UpdateAction action =
            l->apply_configuration(fd_, isInitialLoad_, n_.reset(this));
        switch (action)
        {
            case ConfigUpdateListener::UPDATED:
            {
                break;
            }
            case ConfigUpdateListener::RETRY:
            {
                // Will call ourselves again.
                return wait();
            }
            case ConfigUpdateListener::REINIT_NEEDED:
            {
                needsReInit_ = 1;
                break;
            }
            case ConfigUpdateListener::REBOOT_NEEDED:
            {
                needsReboot_ = 1;
                break;
            }
        }
        ++nextRefresh_;
        return wait();
    }

    typedef TypedQueue<ConfigUpdateListener> queue_type;
    /// All registered update listeners. Protected by Atomic *this.
    queue_type listeners_;
    /// Where are we in the refresh cycle.
    typename queue_type::iterator nextRefresh_;
    /// are we in initial load?
    unsigned isInitialLoad_ : 1;
    unsigned needsReboot_ : 1;
    unsigned needsReInit_ : 1;
    int fd_;
    BarrierNotifiable n_;
};

} // namespace nmranet

#endif // _NMRANET_CONFIGUPDATEFLOW_HXX_

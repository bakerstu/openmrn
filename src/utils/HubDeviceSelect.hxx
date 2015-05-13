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
 * \file HubDevice.hxx
 * Components for Hubs to connect to physical devices.
 *
 * @author Balazs Racz
 * @date 26 Apr 2014
 */

#ifndef _UTILS_HUBDEVICESELECT_HXX_
#define _UTILS_HUBDEVICESELECT_HXX_

#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>

#include "executor/StateFlow.hxx"
#include "freertos/can_ioctl.h"
#include "utils/Hub.hxx"

template <class HFlow>
class HubDeviceSelect : public Destructable, private Atomic, public Service
{
public:
    HubDeviceSelect(HFlow *hub, const char *path)
        : Service(hub->service()->executor())
        , fd_(::open(path, O_RDWR | O_NONBLOCK))
        , hub_(hub)
        , readFlow_(this)
        , writeFlow_(this)
    {
        HASSERT(fd_ >= 0);
        hub_->register_port(write_port());
    }

    HubDeviceSelect(HFlow *hub, int fd)
        : Service(hub->service()->executor())
        , fd_(fd)
        , hub_(hub)
        , readFlow_(this)
        , writeFlow_(this)
    {
        HASSERT(fd_ >= 0);
        ::fcntl(fd, F_SETFL, O_RDWR | O_NONBLOCK);
        hub_->register_port(write_port());
    }

    virtual ~HubDeviceSelect()
    {
        hub_->unregister_port(write_port());
        executor()->sync_run([this]()
            {
                readFlow_.shutdown();
                writeFlow_.shutdown();
            });
    }

    HFlow *hub()
    {
        return hub_;
    }

    typename HFlow::port_type *write_port()
    {
        return &writeFlow_;
    }

    int fd()
    {
        return fd_;
    }

    static void resize_target(typename HFlow::buffer_type *b);
    static void check_target_size(
        typename HFlow::buffer_type *b, int remaining);
    static bool needs_read_fully();

protected:
    class ReadFlow : public StateFlowBase
    {
    public:
        ReadFlow(HubDeviceSelect *device)
            : StateFlowBase(device)
            , b_(nullptr)
        {
            this->start_flow(STATE(allocate_buffer));
        }

        void shutdown()
        {
            this->service()->executor()->unselect(&selectHelper_);
        }

        HubDeviceSelect *device()
        {
            return static_cast<HubDeviceSelect *>(this->service());
        }

        Action allocate_buffer()
        {
            return this->allocate_and_call(device()->hub(), STATE(try_read));
        }

        Action try_read()
        {
            b_ = this->get_allocation_result(device()->hub());
            b_->data()->skipMember_ = device()->write_port();
            HubDeviceSelect::resize_target(b_);
            if (HubDeviceSelect::needs_read_fully())
            {
                return this->read_repeated(&selectHelper_, device()->fd(),
                    (void *)b_->data()->data(), b_->data()->size(),
                    STATE(read_done), 0);
            }
            else
            {
                return this->read_single(&selectHelper_, device()->fd(),
                    (void *)b_->data()->data(), b_->data()->size(),
                    STATE(read_done), 0);
            }
        }

        Action read_done()
        {
            HubDeviceSelect::check_target_size(b_, selectHelper_.remaining_);
            device()->hub()->send(b_, 0);
            b_ = nullptr;
            return this->call_immediately(STATE(allocate_buffer));
        }

    private:
        StateFlowSelectHelper selectHelper_{this};
        typename HFlow::buffer_type *b_;
    };

    typedef StateFlow<typename HFlow::buffer_type, QList<1>> WriteFlowBase;
    class WriteFlow : public WriteFlowBase
    {
    public:
        WriteFlow(HubDeviceSelect *dev)
            : WriteFlowBase(dev)
        {
        }

        void shutdown()
        {
            this->service()->executor()->unselect(&selectHelper_);
        }

        HubDeviceSelect *device()
        {
            return static_cast<HubDeviceSelect *>(this->service());
        }

        StateFlowBase::Action entry() OVERRIDE
        {
            return this->write_repeated(&selectHelper_, device()->fd(),
                this->message()->data()->data(),
                this->message()->data()->size(), STATE(write_done),
                this->priority());
        }

        StateFlowBase::Action write_done()
        {
            HASSERT(!selectHelper_.remaining_);
            return this->release_and_exit();
        }

    private:
        StateFlowBase::StateFlowSelectHelper selectHelper_{this};
    };

protected:
    /** The device file descriptor. */
    int fd_;
    HFlow *hub_;
    ReadFlow readFlow_;
    WriteFlow writeFlow_;
};

#endif // _UTILS_HUBDEVICESELECT_HXX_

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

/// Generic template for the buffer traits. HubDeviceSelect will not compile on
/// this default template because it lacks the necessary definitions. For each
/// hub type there must be a partial template specialization of this class.
template <class BType> struct SelectBufferInfo
{
};

/// Partial template specialization of buffer traits for string-typed hubs.
template <> struct SelectBufferInfo<HubFlow::buffer_type>
{
    static void resize_target(HubFlow::buffer_type *b)
    {
        b->data()->resize(64);
    }
    static void check_target_size(HubFlow::buffer_type *b, int remaining)
    {
        HASSERT(remaining >= 0);
        HASSERT(remaining <= 64);
        b->data()->resize(64 - remaining);
    }
    static bool needs_read_fully()
    {
        return false;
    }
};

/// Partial template specialization of buffer traits for struct-typed hubs.
template <class T>
struct SelectBufferInfo<Buffer<HubContainer<StructContainer<T>>>>
{
    typedef Buffer<HubContainer<StructContainer<T>>> buffer_type;

    static void resize_target(buffer_type *b)
    {
    }
    static void check_target_size(buffer_type *b, int remaining)
    {
        HASSERT(remaining == 0);
    }
    static bool needs_read_fully()
    {
        return true;
    }
};

/// Partial template specialization of buffer traits for CAN frame-typed
/// hubs. The implementation here is equivalent to
/// SelectBufferInfo<Hubcontainer<StructContainer<T>>> but the c++ template
/// inference cannot figure this out.
template <>
struct SelectBufferInfo<Buffer<CanHubData>> {
    typedef Buffer<CanHubData> buffer_type;
    
    static void resize_target(buffer_type *b)
    {
    }
    static void check_target_size(buffer_type *b, int remaining)
    {
        HASSERT(remaining == 0);
    }
    static bool needs_read_fully()
    {
        return true;
    }
};

/// HubPort that connects a select-aware device to a strongly typed Hub.
///
/// The device is given by either the path to the device or the fd to an opened
/// device instance. The device will be put to nonblocking mode and all
/// processing will be performed in the executor of the hub, by using
/// ExecutorBase::select(). No additional threads are started.
///
/// Reads and writes will be performed in the units defined by the type of the
/// hub: for string-typed hubs in 64 bytes units; for hubs of specific
/// structures (such as CAN frame, dcc Packets or dcc Feedback structures) in
/// the units ofthe size of the structure.
template <class HFlow>
class HubDeviceSelect : public Destructable, private Atomic, public Service
{
public:

#ifndef __WINNT__
    /// Creates a select-aware hub port for the device specified by `path'.
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
#endif

    /// Creates a select-aware hub port for the opened device specified by
    /// `fd'. It can be a hardware device, socket or pipe.
    HubDeviceSelect(HFlow *hub, int fd)
        : Service(hub->service()->executor())
        , fd_(fd)
        , hub_(hub)
        , readFlow_(this)
        , writeFlow_(this)
    {
        HASSERT(fd_ >= 0);
#ifdef __WINNT__
        unsigned long par = 1;
        ioctlsocket(fd_, FIONBIO, &par);
#else
        ::fcntl(fd, F_SETFL, O_RDWR | O_NONBLOCK);
#endif
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

protected:
    /// State flow implementing select-aware fd reads.
    class ReadFlow : public StateFlowBase
    {
    public:
        typedef typename HFlow::buffer_type buffer_type;

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
            SelectBufferInfo<buffer_type>::resize_target(b_);
            if (SelectBufferInfo<buffer_type>::needs_read_fully())
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
            /// @TODO check if the selectHelper_.buf == nullptr, indicating an
            /// error.
            SelectBufferInfo<buffer_type>::check_target_size(
                b_, selectHelper_.remaining_);
            device()->hub()->send(b_, 0);
            b_ = nullptr;
            return this->call_immediately(STATE(allocate_buffer));
        }

    private:
        StateFlowSelectHelper selectHelper_{this};
        buffer_type *b_;
    };

    typedef StateFlow<typename HFlow::buffer_type, QList<1>> WriteFlowBase;
    /// State flow implementing select-aware fd writes.
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

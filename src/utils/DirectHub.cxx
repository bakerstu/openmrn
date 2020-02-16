/** \copyright
 * Copyright (c) 2020, Balazs Racz
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
 * \file DirectHub.cxx
 *
 * Optimized class for ingress and egress of write-once objects, aimed to
 * support multi-recipient messages with fast internal fan-out but low compute
 * overhead.
 *
 * @author Balazs Racz
 * @date 9 Feb 2020
 */

#include "utils/DirectHub.hxx"

#include <algorithm>
#include <vector>

#include "executor/StateFlow.hxx"

/// A single service class that is shared between all interconnected DirectHub
/// instances. It is the responsibility of this Service to perform the locking
/// of the individual flows.
class DirectHubService : public Service
{
public:
    typedef Q QueueType;

    DirectHubService(ExecutorBase *e)
        : Service(e)
        , busy_(0)
    {
    }

    /// @return lock object for the busy_ flag.
    Atomic *lock()
    {
        return pendingSend_.lock();
    }

    /// Adds a caller to the waiting list of who wants to send traffic to the
    /// hub. If there is no waiting list, the caller will be executed inline.
    /// @param caller represents an entry point to the hub. It is required that
    /// caller finishes its run() by invoking on_done().
    void enqueue_caller(Executable *caller)
    {
        {
            AtomicHolder h(lock());
            if (busy_)
            {
                pendingSend_.insert_locked(caller);
                return;
            }
            busy_ = 1;
        }
        caller->run();
    }

    /// This function must be called at the end of the enqueued functions in
    /// order to properly clear the busy flag or take out the next enqueued
    /// executable.
    void on_done()
    {
        // De-queues the next entry.
        Result deq;
        {
            AtomicHolder h(lock());
            if (pendingSend_.empty())
            {
                busy_ = 0;
                return;
            }
            deq = pendingSend_.next();
        }
        // Schedules it on the executor.
        executor()->add(static_cast<Executable *>(deq.item), deq.index);
    }

    /// 1 if there is any message being processed right now.
    unsigned busy_ : 1;
    /// List of callers that are waiting for the busy_ lock.
    QueueType pendingSend_;
};

template <class T>
class DirectHubImpl : public DirectHubInterface<T>,
                      protected StateFlowBase,
                      private Atomic
{
public:
    DirectHubImpl(DirectHubService *service)
        : StateFlowBase(service)
    {
    }

    void register_port(DirectHubPort<T> *port) override
    {
        AtomicHolder h(this);
        ports_.push_back(port);
    }

    /// Synchronously unregisters a port.
    void unregister_port(DirectHubPort<T> *port) override
    {
        SyncNotifiable n;
        unregister_port(port, &n);
        n.wait_for_notification();
    }

    /// Removes a port from this hub. This port must have been registered
    /// previously.
    /// @param port the downstream port.
    void unregister_port(DirectHubPort<T> *port, Notifiable *done) override
    {
        // By enqueueing on the service we ensure that the state flow is not
        // processing any packets while the code below is running.
        service()->enqueue_caller(new CallbackExecutable([this, port, done]() {
            {
                AtomicHolder h(this);
                ports_.erase(std::remove(ports_.begin(), ports_.end(), port),
                    ports_.end());
            }
            done->notify();
            service()->on_done();
        }));
    }

    void enqueue_send(Executable *caller) override
    {
        service()->enqueue_caller(caller);
    }

    MessageAccessor<T> *mutable_message() override
    {
        return *msg_;
    }

    void do_send() override
    {
        unsigned next_port = 0;
        while (true)
        {
            DirectHubPort<T> *p;
            {

                AtomicHolder h(this);
                if (next_port >= ports_.size())
                {
                    break;
                }
                p = ports_[next_port];
                ++next_port;
            }
            if (should_send_to(p))
            {
                p->send(&msg_);
            }
        }
        msg_.clear();
        service()->on_done();
    }

    /// Filters a message going towards a specific output port.
    /// @param p the output port
    /// @return true if this message should be sent to that output port.
    bool should_send_to(DirectHubPort<T> *p)
    {
        return static_cast<HubSource *>(p) != msg_.source_;
    }

private:
    DirectHubService *service()
    {
        return static_cast<DirectHubService *>(StateFlowBase::service());
    }

    /// Stores the registered output ports. Protected by Atomic *this.
    std::vector<DirectHubPort<T> *> ports_;

    /// The message we are trying to send.
    MessageAccessor<T> msg_;
}; // class DirectHubImpl

DirectHubInterface<uint8_t[]> *create_hub(ExecutorBase *e)
{
    auto *s = new DirectHubService(e);
    auto *dh = new DirectHubImpl<uint8_t[]>(s);
    return dh;
}

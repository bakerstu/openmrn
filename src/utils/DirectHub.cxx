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
#include <fcntl.h>
#include <vector>

#include "executor/StateFlow.hxx"
#include "utils/logging.h"

static DataBufferPool g_direct_hub_data_pool(64);

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

    Service *get_service() override
    {
        return service();
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
        return &msg_;
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

/// Temporary function to instantiate the hub.
DirectHubInterface<uint8_t[]> *create_hub(ExecutorBase *e)
{
    auto *s = new DirectHubService(e);
    auto *dh = new DirectHubImpl<uint8_t[]>(s);
    return dh;
}

/// Connects a (bytes typed) hub to an FD.
class DirectHubPortSelect : public DirectHubPort<uint8_t[]>,
                            private StateFlowBase
{
private:
    class DirectHubReadFlow : public StateFlowBase {
    public:
        DirectHubReadFlow(DirectHubPortSelect* parent)
            : StateFlowBase(parent->service()),
              parent_(parent) {
            start_flow(STATE(alloc_for_read));
        }

    private:
        Action alloc_for_read() {
            //Buffer<uint8_t[]> *b;
            DataBuffer *b;
            g_direct_hub_data_pool.alloc(&b);
            LOG(WARNING, "buffer size %u base %u", (unsigned)b->size(),  (unsigned)sizeof(BufferBase));
            LOG(WARNING, "ptr %p data %p", b, b->data());
            return exit();
        }

        BufferPtr<uint8_t[]> buf_;
        DirectHubPortSelect* parent_;
    } readFlow_;

    friend class DirectHubReadFlow;
    
public:
    DirectHubPortSelect(Service *s, DirectHubInterface<uint8_t[]> *hub, int fd)
        : StateFlowBase(s)
        , readFlow_(this)
        , hub_(hub)
        , fd_(fd)
    {
        ::fcntl(fd_, F_SETFL, O_NONBLOCK);
        wait_and_call(STATE(read_queue));
        notRunning_ = 1;
        hub_->register_port(this);
    }

    ~DirectHubPortSelect()
    {
        hub_->unregister_port(this);
    }

    /// Synchronous output routine called by the hub.
    void send(MessageAccessor<uint8_t[]> *msg) override
    {
        /// @todo we should try to collect the bytes into a buffer first before
        /// enqueueing them.
        BufferType *b;
        mainBufferPool->alloc(&b);
        b->data()->buf = msg->payload_->ref_all(msg->skip_ + msg->size_);
        b->data()->skip_ = msg->skip_;
        b->data()->size_ = msg->size_;
        if (msg->done_)
        {
            b->set_done(msg->done_->new_child());
        }
        // Checks if we need to wake up the flow.
        {
            AtomicHolder h(pendingQueue_.lock());
            pendingQueue_.insert_locked(b);
            totalPendingSize_ += msg->size_;
            if (notRunning_)
            {
                notRunning_ = 0;
            }
            else
            {
                // flow already running.
                return;
            }
        }
        notify();
    }

private:
    Action read_queue()
    {
        BufferType *head = static_cast<BufferType *>(pendingQueue_.next().item);
        HASSERT(head);
        currentHead_.reset(head);
        nextToWrite_ = currentHead_->data()->buf;
        nextToSkip_ = currentHead_->data()->skip_;
        nextToSize_ = currentHead_->data()->size_;
        return do_write();
    }

    Action do_write()
    {
        uint8_t *data;
        unsigned len;
        nextToWrite_ = nextToWrite_->get_read_pointer(nextToSkip_, &data, &len);
        if (len > nextToSize_)
        {
            len = nextToSize_;
        }
        nextToSkip_ = 0;
        nextToSize_ -= len;
        return write_repeated(
            &selectHelper_, fd_, data, len, STATE(write_done));
    }

    Action write_done()
    {
        if (selectHelper_.hasError_)
        {
            /// @todo: maybe we need to stop the reader thread too.
            LOG(WARNING, "Error writing to fd %d", fd_);
            ::close(fd_);
            hub_->unregister_port(this, this);
            return wait_and_call(STATE(exit));
        }
        if (nextToSize_)
        {
            return do_write();
        }
        currentHead_.reset();
        AtomicHolder h(pendingQueue_.lock());
        if (pendingQueue_.empty())
        {
            notRunning_ = 1;
            return wait_and_call(STATE(read_queue));
        }
        else
        {
            return call_immediately(STATE(read_queue));
        }
    }

    /// Holds the necessary information we need to keep in the queue about a
    /// single output entry.
    struct OutputDataEntry
    {
        LinkedDataBuffer *buf = nullptr;
        unsigned skip_ = 0;
        unsigned size_ = 0;
        ~OutputDataEntry()
        {
            if (buf)
            {
                buf->unref_all(skip_ + size_);
            }
        }
    };

    /// Type of buffers we are enqueuing for output.
    typedef Buffer<OutputDataEntry> BufferType;
    /// Type of the queue used to keep the output buffer queue.
    typedef Q QueueType;

    /// The buffer that is taken out of the queue while flushing.
    BufferPtr<OutputDataEntry> currentHead_;
    /// Data we are currently writing to a buffer.
    LinkedDataBuffer *nextToWrite_;
    /// Skip_ parameter matching nextToWrite_;
    unsigned nextToSkip_;
    /// Size_ parameter matching nextToWrite_;
    unsigned nextToSize_;
    /// Helper object for performing asynchronous writes.
    StateFlowSelectHelper selectHelper_ {this};
    /// Time when the last buffer flush has happened. Not used yet.
    //long long lastWriteTimeNsec_ = 0;
    
    /// Contains buffers of OutputDataEntries to write.
    QueueType pendingQueue_;
    /// Total numberof bytes in the pendingQueue.
    size_t totalPendingSize_ = 0;
    /// 1 if the state flow is paused, waiting for the notification.
    unsigned notRunning_ : 1;
    /// Parent hub where output data is coming from.
    DirectHubInterface<uint8_t[]> *hub_;
    /// File descriptor for input/output.
    int fd_;
};


void create_port_for_fd(DirectHubInterface<uint8_t[]>* hub, int fd) {
    new DirectHubPortSelect(hub->get_service(), hub, fd);

}

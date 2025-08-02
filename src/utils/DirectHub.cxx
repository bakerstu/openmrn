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

// #define LOGLEVEL VERBOSE

#include "openmrn_features.h"

#if OPENMRN_FEATURE_BSD_SOCKETS

#include "utils/DirectHub.hxx"

#include <algorithm>
#include <vector>

#include <fcntl.h>

#if OPENMRN_FEATURE_BSD_SOCKETS
#include <sys/socket.h>
#include <sys/types.h>
#endif

#include "executor/AsyncNotifiableBlock.hxx"
#include "executor/StateFlow.hxx"
#include "nmranet_config.h"
#include "utils/logging.h"
#include "utils/socket_listener.hxx"

/// This object forwards allocations to mainBufferPool. The blocks allocated
/// here are all the same size. They are used to read bytes from a tcp socket
/// into memory.
DataBufferPool g_direct_hub_data_pool(
    config_directhub_port_incoming_buffer_size());
/// This object forwards allocations to mainBufferPool. The blocks allocated
/// here are all the same size. They are used to render outgoing CAN packets
/// into gridconnect format.
DataBufferPool g_direct_hub_kbyte_pool(1024);

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
                /// @todo there is a short period of priority inversion here,
                /// because we insert an executable into a separate queue here
                /// than the Executor. We dequeue the highest priority in
                /// on_done(), but if that happens to be a low priority, that
                /// might get stuck in the Executor for a long time, even if in
                /// the meantime a higher priority message arrives here. A
                /// better strategy would be to enqueue the Executable's in the
                /// Service's executor directly instead of queueing them and
                /// pushing them one by one to the Service's executor. In that
                /// case we'd need a third state between busy and not busy:
                /// whether we're queueing executable's or whether we're
                /// dumping them into the executor. We would also need to keep
                /// track of how many went to the Executor already, such that
                /// we know when busy_ gets back to false.
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

    ~DirectHubImpl()
    {
        delete service();
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

/// Connects a (bytes typed) hub to an FD. This state flow is the write flow;
/// i.e., it waits for messages coming from the hub and writes them into the fd.
/// The object is self-owning, i.e. will delete itself when the input goes dead
/// or when the port is shutdown (eventually).
class DirectHubPortSelect : public DirectHubPort<uint8_t[]>,
                            private StateFlowBase
{
private:
    /// State flow that reads the FD and sends the read data to the direct hub.
    class DirectHubReadFlow : public StateFlowBase
    {
    public:
        DirectHubReadFlow(DirectHubPortSelect *parent,
            std::unique_ptr<MessageSegmenter> segmenter)
            : StateFlowBase(parent->service())
            , parent_(parent)
            , segmenter_(std::move(segmenter))
        {
            segmenter_->clear();
        }

        /// Starts the current flow.
        void start()
        {
            start_flow(STATE(alloc_for_read));
        }

        /// Requests the read port to shut down. Must be called on the main
        /// executor. Causes the flow to notify the parent via the
        /// read_flow_exit() function then terminate, either inline or not.
        void read_shutdown()
        {
            auto *e = this->service()->executor();
            if (e->is_selected(&helper_))
            {
                // We're waiting in select on reads, we can cancel right now.
                e->unselect(&helper_);
                set_terminated();
                buf_.reset();
                /// @todo We should first clean up the async notifiable block
                /// and only signal the exit afterwards.
                parent_->read_flow_exit();
            }
            // Else we're waiting for the regular progress to wake up the
            // flow. It will check fd_ < 0 to exit.
        }

    private:
        /// Root of the read flow. Starts with getting the barrier notifiable,
        /// either synchronously if one is available, or asynchronously.
        Action alloc_for_read()
        {
            QMember *bn = pendingLimiterPool_.next().item;
            if (bn)
            {
                bufferNotifiable_ = pendingLimiterPool_.initialize(bn);
                return get_read_buffer();
            }
            else
            {
                pendingLimiterPool_.next_async(this);
                return wait_and_call(STATE(barrier_allocated));
            }
        }

        /// Intermediate step if asynchronous allocation was necessary for the
        /// read barrier.
        Action barrier_allocated()
        {
            QMember *bn;
            cast_allocation_result(&bn);
            HASSERT(bn);
            bufferNotifiable_ = pendingLimiterPool_.initialize(bn);
            return get_read_buffer();
        }

        /// Invoked when we have a bufferNotifiable_ from the barrier pool.
        Action get_read_buffer()
        {
            DataBuffer *p;
            LOG(VERBOSE, "read flow %p (fd %d): notif %p alloc() %u", this,
                parent_->fd_, (BarrierNotifiable *)bufferNotifiable_,
                (unsigned)mainBufferPool->total_size());
            // Since there is a limit on how many bufferNotifiable_'s can be,
            // and they are uniquely assigned to the buffers, we know that this
            // synchronous allocation can only happen for a few buffers
            // only. The buffers will get recycled through the main buffer pool
            // exactly at the time when the bufferNotifiable_ comes back to the
            // pendingLimiterPool_.
            g_direct_hub_data_pool.alloc(&p);
            if (buf_.head())
            {
                buf_.append_empty_buffer(p);
            }
            else
            {
                buf_.reset(p);
            }
            p->set_done(bufferNotifiable_);
            bufferNotifiable_ = nullptr;
            return do_some_read();
        }

        Action do_some_read()
        {
            if (parent_->fd_ < 0)
            {
                // Socket closed, terminate and exit.
                set_terminated();
                buf_.reset();
                parent_->read_flow_exit();
                return wait();
            }
            return read_single(&helper_, parent_->fd_,
                buf_.data_write_pointer(), buf_.free(), STATE(read_done));
        }

        Action read_done()
        {
            if (helper_.hasError_)
            {
                LOG(INFO, "%p: Error reading from fd %d: (%d) %s", parent_,
                    parent_->fd_, errno, strerror(errno));
                set_terminated();
                buf_.reset();
                parent_->report_read_error();
                return wait();
            }
            size_t bytes_arrived = buf_.free() - helper_.remaining_;
            segmentSize_ = segmenter_->segment_message(
                buf_.data_write_pointer(), bytes_arrived);
            buf_.data_write_advance(bytes_arrived);
            return eval_segment();
        }

        /// Checks the segmenter output; if it indicates a complete message,
        /// clears the segmenter and sends off the message.
        Action eval_segment()
        {
            if (segmentSize_ > 0)
            {
                // Complete message.
                segmenter_->clear();
                return call_immediately(STATE(send_prefix));
            }
            else
            {
                return incomplete_message();
            }
        }

        /// Clears the segmenter and starts segmenting from the beginning of
        /// the buf_.
        Action call_head_segmenter()
        {
            uint8_t *ptr;
            unsigned available;
            auto *n =
                buf_.head()->get_read_pointer(buf_.skip(), &ptr, &available);
            HASSERT(!n); // We must be at the tail.
            segmentSize_ = segmenter_->segment_message(ptr, available);
            return eval_segment();
        }

        /// Called when the segmenter says that we need to read more bytes to
        /// complete the current message.
        Action incomplete_message()
        {
            if (!buf_.free())
            {
                return call_immediately(STATE(alloc_for_read));
            }
            return call_immediately(STATE(do_some_read));
        }

        /// Called to send a given prefix segment to the hub.
        /// segmentSize_ is filled in before.
        Action send_prefix()
        {
            // We expect either an inline call to our run() method or
            // later a callback on the executor. This sequence of calls
            // prepares for both of those options.
            wait_and_call(STATE(send_callback));
            inlineCall_ = 1;
            sendComplete_ = 0;
            parent_->hub_->enqueue_send(this); // causes the callback
            inlineCall_ = 0;
            if (sendComplete_)
            {
                return send_done();
            }
            return wait();
        }

        /// This is the callback state that is invoked inline by the hub. Since
        /// the hub invokes this->run(), a standard StateFlow will execute
        /// whatever state is current. We have set STATE(send_callback) as the
        /// current state above, hence the code continues in this function.
        Action send_callback()
        {
            auto *m = parent_->hub_->mutable_message();
            m->set_done(buf_.tail()->new_child());
            m->source_ = parent_;
            // This call transfers the chained head of the current buffers,
            // taking additional references where necessary or transferring the
            // existing reference. It adjusts the skip_ and size_ arguments in
            // buf_ to continue from where we left off.
            m->buf_ = buf_.transfer_head(segmentSize_);
            parent_->hub_->do_send();
            sendComplete_ = 1;
            if (inlineCall_)
            {
                // do not disturb current state.
                return wait();
            }
            else
            {
                // we were called queued; go back to running the flow on the
                // main executor.
                return yield_and_call(STATE(send_done));
            }
        }

        Action send_done()
        {
            if (buf_.size())
            {
                // We still have unused data in the current buffer. We have to
                // segment that and send it to the hub.
                return call_head_segmenter();
            }
            if (buf_.free())
            {
                // We still have space in the current buffer. We can read more
                // data into that space.
                return do_some_read();
            }
            else
            {
                /// @todo consider not resetting here, but allowing an empty
                /// but linked DataBuffer* start the next chain.
                buf_.reset();
                return alloc_for_read();
            }
        }

        /// Current buffer that we are filling.
        LinkedDataBufferPtr buf_;
        /// Barrier notifiable to keep track of the buffer's contents.
        BarrierNotifiable *bufferNotifiable_;
        /// Output of the last segmenter call.
        ssize_t segmentSize_;
        /// 1 if we got the send callback inline from the read_done.
        uint16_t inlineCall_ : 1;
        /// 1 if the run callback actually happened inline.
        uint16_t sendComplete_ : 1;
        /// Pool of BarrierNotifiables that limit the amount of inflight bytes
        /// we have.
        AsyncNotifiableBlock pendingLimiterPool_ {
            (unsigned)config_directhub_port_max_incoming_packets()};
        /// Helper object for Select.
        StateFlowSelectHelper helper_ {this};
        /// Pointer to the owninng port.
        DirectHubPortSelect *parent_;
        /// Implementation (and state) of the business logic that segments
        /// incoming bytes into messages that shall be given to the hub.
        std::unique_ptr<MessageSegmenter> segmenter_;
    } readFlow_;

    friend class DirectHubReadFlow;

public:
    DirectHubPortSelect(DirectHubInterface<uint8_t[]> *hub, int fd,
        std::unique_ptr<MessageSegmenter> segmenter,
        Notifiable *on_error = nullptr)
        : StateFlowBase(hub->get_service())
        , readFlow_(this, std::move(segmenter))
        , readFlowPending_(1)
        , writeFlowPending_(1)
        , hub_(hub)
        , fd_(fd)
        , onError_(on_error)
    {
#ifdef __WINNT__
        unsigned long par = 1;
        ioctlsocket(fd_, FIONBIO, &par);
#else
        ::fcntl(fd, F_SETFL, O_RDWR | O_NONBLOCK);
#endif

        // Sets the initial state of the write flow to the stage where we read
        // the next entry from the queue.
        wait_and_call(STATE(read_queue));
        notRunning_ = 1;

        hub_->register_port(this);
        readFlow_.start();
        LOG(VERBOSE, "%p create fd %d", this, fd_);
    }

    ~DirectHubPortSelect()
    {
    }

    /// Synchronous output routine called by the hub.
    void send(MessageAccessor<uint8_t[]> *msg) override
    {
        if (fd_ < 0)
        {
            // Port already closed. Ignore data to send.
            return;
        }
        {
            AtomicHolder h(lock());
            if (pendingTail_ && pendingTail_->buf_.try_append_from(msg->buf_))
            {
                // Successfully enqueued the bytes into the tail of the queue.
                // Nothing else to do here.
                return;
            }
        }

        /// @todo we should try to collect the bytes into a buffer first before
        /// enqueueing them.
        BufferType *b;
        mainBufferPool->alloc(&b);
        b->data()->buf_.reset(msg->buf_);
        if (msg->done_)
        {
            b->set_done(msg->done_->new_child());
        }
        // Checks if we need to wake up the flow.
        {
            AtomicHolder h(lock());
            if (fd_ < 0)
            {
                // Catch race condition when port is already closed.
                b->unref();
                return;
            }
            pendingQueue_.insert_locked(b);
            totalPendingSize_ += msg->buf_.size();
            pendingTail_ = b->data();
            if (notRunning_)
            {
                notRunning_ = 0;
            }
            else
            {
                // flow already running. Skip notify.
                return;
            }
        }
        notify();
    }

private:
    /// Called on the main executor when a read error wants to cancel the write
    /// flow. Before calling, fd_ must be -1.
    void shutdown()
    {
        HASSERT(fd_ < 0);
        {
            AtomicHolder h(lock());
            if (notRunning_)
            {
                // Queue is empty, waiting for new entries. There will be no new
                // entries because fd_ < 0.
                hub_->unregister_port(this, this);
                wait_and_call(STATE(report_and_exit));
            }
            // Else eventually we will get to check_for_new_message() which will
            // flush the queue, unregister the port and exit.
        }
    }

    Action read_queue()
    {
        BufferType *head;
        {
            AtomicHolder h(lock());
            head = static_cast<BufferType *>(pendingQueue_.next_locked().item);
            HASSERT(head);
            if (head->data() == pendingTail_)
            {
                pendingTail_ = nullptr;
            }
        }
        currentHead_.reset(head);
        nextToWrite_ = currentHead_->data()->buf_.head();
        nextToSkip_ = currentHead_->data()->buf_.skip();
        nextToSize_ = currentHead_->data()->buf_.size();
        return do_write();
    }

    Action do_write()
    {
        if (fd_ < 0)
        {
            // fd closed. Drop data to the floor.
            totalPendingSize_ -= nextToSize_;
            return check_for_new_message();
        }
        uint8_t *data;
        unsigned len;
        nextToWrite_ = nextToWrite_->get_read_pointer(nextToSkip_, &data, &len);
        if (len > nextToSize_)
        {
            len = nextToSize_;
        }
        nextToSkip_ = 0;
        nextToSize_ -= len;
        totalPendingSize_ -= len;
        totalWritten_ += len;
        LOG(VERBOSE, "write %u total %zu", (unsigned)len, totalWritten_);
        return write_repeated(
            &selectHelper_, fd_, data, len, STATE(write_done));
    }

    Action write_done()
    {
        if (selectHelper_.hasError_)
        {
            LOG(INFO, "%p: Error writing to fd %d: (%d) %s", this, fd_, errno,
                strerror(errno));
            // will close fd and notify the reader flow to exit.
            report_write_error();
            // Flushes the queue of messages. fd_ == -1 now so no write will be
            // attempted.
            return check_for_new_message();
        }
        if (nextToSize_)
        {
            return do_write();
        }
        return check_for_new_message();
    }

    Action check_for_new_message()
    {
        currentHead_.reset();
        AtomicHolder h(lock());
        if (pendingQueue_.empty())
        {
            if (fd_ < 0)
            {
                // unregisters the port. All the queue has been flushed now.
                hub_->unregister_port(this, this);
                return wait_and_call(STATE(report_and_exit));
            }
            notRunning_ = 1;
            return wait_and_call(STATE(read_queue));
        }
        else
        {
            return call_immediately(STATE(read_queue));
        }
    }

    /// Terminates the flow, reporting to the barrier.
    Action report_and_exit()
    {
        set_terminated();
        currentHead_.reset();
        write_flow_exit();
        return wait();
    }

    /// Called by the write flow when it sees an error. Called on the main
    /// executor. The assumption here is that the write flow still has entries
    /// in its queue that need to be removed. Closes the socket, and notifies
    /// the read flow to exit. Does not typically delete this, because the write
    /// flow needs to exit separately.
    void report_write_error()
    {
        int close_fd = -1;
        {
            AtomicHolder h(lock());
            if (fd_ >= 0)
            {
                std::swap(fd_, close_fd);
            }
        }
        if (close_fd >= 0)
        {
            ::close(close_fd);
        }
        readFlow_.read_shutdown();
    }

    /// Callback from the ReadFlow when the read call has seen an error. The
    /// read flow is assumed to be exited. Takes the read entry out of the
    /// barrier, notifies the write flow to stop and possibly deletes *this.
    /// Called on the main executor.
    void report_read_error()
    {
        int close_fd = -1;
        {
            AtomicHolder h(lock());
            if (fd_ >= 0)
            {
                std::swap(fd_, close_fd);
            }
        }
        if (close_fd >= 0)
        {
            ::close(close_fd);
        }
        // take read barrier
        read_flow_exit();
        // kill write flow
        shutdown();
    }

    /// Callback from the read flow that it has exited. This is triggered after
    /// the shutdown() call. May delete this.
    void read_flow_exit()
    {
        LOG(VERBOSE, "%p exit read", this);
        flow_exit(true);
    }

    /// Marks the write flow as exited. May delete this.
    void write_flow_exit()
    {
        LOG(VERBOSE, "%p exit write", this);
        flow_exit(false);
    }

    /// Marks a flow to be exited, and once both are exited, notifies done and
    /// deletes this.
    /// @param read if true, marks the read flow done, if false, marks the write
    /// flow done.
    void flow_exit(bool read)
    {
        bool del = false;
        {
            AtomicHolder h(lock());
            if (read)
            {
                readFlowPending_ = 0;
            }
            else
            {
                writeFlowPending_ = 0;
            }
            if (writeFlowPending_ == 0 && readFlowPending_ == 0)
            {
                del = true;
            }
        }
        if (del)
        {
            if (onError_)
            {
                onError_->notify();
            }
            delete this;
        }
    }

    /// @return lock usable for the write flow and the port altogether.
    Atomic *lock()
    {
        return pendingQueue_.lock();
    }

    /// Holds the necessary information we need to keep in the queue about a
    /// single output entry. Automatically unrefs the buffer whose pointer we
    /// are holding when released.
    struct OutputDataEntry
    {
        LinkedDataBufferPtr buf_;
    };

    friend class DirectHubReadFlow;

    /// Type of buffers we are enqueuing for output.
    typedef Buffer<OutputDataEntry> BufferType;
    /// Type of the queue used to keep the output buffer queue.
    typedef Q QueueType;

    /// total number of bytes written to the port.
    size_t totalWritten_ {0};

    /// The buffer that is taken out of the queue while flushing.
    BufferPtr<OutputDataEntry> currentHead_;
    /// Data we are currently writing to a buffer.
    DataBuffer *nextToWrite_;
    /// Skip_ parameter matching nextToWrite_;
    unsigned nextToSkip_;
    /// Size_ parameter matching nextToWrite_;
    unsigned nextToSize_;
    /// Helper object for performing asynchronous writes.
    StateFlowSelectHelper selectHelper_ {this};
    /// Time when the last buffer flush has happened. Not used yet.
    // long long lastWriteTimeNsec_ = 0;

    /// Contains buffers of OutputDataEntries to write.
    QueueType pendingQueue_;
    /// Last tail pointer in the pendingQueue. If queue is empty,
    /// nullptr. Protected by pendingQueue_.lock().
    OutputDataEntry *pendingTail_ = nullptr;
    /// Total numberof bytes in the pendingQueue.
    size_t totalPendingSize_ = 0;
    /// 1 if the state flow is paused, waiting for the notification.
    uint8_t notRunning_ : 1;
    /// 1 if the read flow is still running.
    uint8_t readFlowPending_;
    /// 1 if the write flow is still running.
    uint8_t writeFlowPending_;
    /// Parent hub where output data is coming from.
    DirectHubInterface<uint8_t[]> *hub_;
    /// File descriptor for input/output.
    int fd_;
    /// This notifiable will be called before exiting.
    Notifiable *onError_ = nullptr;
};

extern DirectHubPortSelect *g_last_direct_hub_port;
DirectHubPortSelect *g_last_direct_hub_port = nullptr;

void create_port_for_fd(DirectHubInterface<uint8_t[]> *hub, int fd,
    std::unique_ptr<MessageSegmenter> segmenter, Notifiable *on_error)
{
    g_last_direct_hub_port =
        new DirectHubPortSelect(hub, fd, std::move(segmenter), on_error);
}

class DirectGcTcpHub
{
public:
    /// Constructor.
    ///
    /// @param can_hub Which CAN-hub should we attach the TCP gridconnect hub
    /// onto.
    /// @param port TCp port number to listen on.
    DirectGcTcpHub(DirectHubInterface<uint8_t[]> *gc_hub, int port);
    ~DirectGcTcpHub();

    /// @return true of the listener is ready to accept incoming connections.
    bool is_started()
    {
        return tcpListener_.is_started();
    }

private:
    /// Callback when a new connection arrives.
    ///
    /// @param fd filedes of the freshly established incoming connection.
    ///
    void OnNewConnection(int fd);

    /// Direct GridConnect hub.
    DirectHubInterface<uint8_t[]> *gcHub_;
    /// Helper object representing the listening on the socket.
    SocketListener tcpListener_;
};

void DirectGcTcpHub::OnNewConnection(int fd)
{
#if 0    
    uint32_t rcvbuf;
    socklen_t len = sizeof(rcvbuf);
    int ret = getsockopt(fd, SOL_SOCKET, SO_RCVBUF, &rcvbuf, &len);
    if (ret >= 0)
    {
        LOG(ALWAYS, "Socket rcvbuf %u", (unsigned)rcvbuf);
    }
#endif    
    create_port_for_fd(gcHub_, fd,
        std::unique_ptr<MessageSegmenter>(create_gc_message_segmenter()));
}

DirectGcTcpHub::DirectGcTcpHub(DirectHubInterface<uint8_t[]> *gc_hub, int port)
    : gcHub_(gc_hub)
    , tcpListener_(port,
          std::bind(
              &DirectGcTcpHub::OnNewConnection, this, std::placeholders::_1))
{
}

DirectGcTcpHub::~DirectGcTcpHub()
{
    tcpListener_.shutdown();
}

void create_direct_gc_tcp_hub(DirectHubInterface<uint8_t[]> *hub, int port)
{
    new DirectGcTcpHub(hub, port);
}

#endif // OPENMRN_FEATURE_BSD_SOCKETS
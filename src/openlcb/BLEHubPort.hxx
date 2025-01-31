/** \copyright
 * Copyright (c) 2024, Balazs Racz
 * All rights reserved
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
 * \file BLEHubPort.hxx
 *
 * DirectHubPort for sending/receiving traffic through a BLE stack.
 *
 * @author Balazs Racz
 * @date 7 Jul 2024
 */

#ifndef _OPENLCB_BLEHUBPORT_HXX_
#define _OPENLCB_BLEHUBPORT_HXX_

#include <functional>

#include "executor/StateFlow.hxx"
#include "openlcb/BLEDefs.hxx"
#include "os/OS.hxx"
#include "utils/DirectHub.hxx"

extern DataBufferPool g_direct_hub_data_pool;

namespace openlcb
{

class BLEHubPort : public DirectHubPort<uint8_t[]>,
                   private StateFlowBase,
                   public BLEProtocolEngine
{
public:
    /// This function needs to be implemented by the application that has the
    /// specific BLE stack. Performs the actual send. Synchronous, with the
    /// expectation being that once this function returns, the data is copied
    /// away from the buffer and is sent or will eventually be sent.
    ///
    /// @param data data payload to send
    /// @param len number of bytes to send
    ///
    using SendFunction = std::function<void(const uint8_t *data, size_t len)>;

    /// How big can a single attribute write be? ESP's BLE implementation says
    /// 600 bytes. We keep some buffer.
    static constexpr size_t MAX_BYTES_PER_WRITE = 220;

    /// Constructor
    ///
    /// @param hub pointer to the direct hub instance. Externally owned.
    /// @param segmenter Segments incoming data into messages. Ownership will be
    /// taken. Typically created using creat_gc_message_segmenter().
    /// @param ble_write_service Thread on which the send_function will be
    /// invoked.
    /// @param send_function Function that performs the actual send. See {\link
    /// SendFunction}
    /// @param on_error Will be notified upon disconnect.
    ///
    BLEHubPort(DirectHubInterface<uint8_t[]> *hub,
        std::unique_ptr<MessageSegmenter> segmenter, Service *ble_write_service,
        SendFunction send_function, Notifiable *on_error = nullptr)
        : StateFlowBase(ble_write_service)
        , pendingShutdown_(false)
        , waitingForAck_(false)
        , sendFunction_(std::move(send_function))
        , onError_(on_error)
        , input_(this, hub, std::move(segmenter))
    {
        // Sets the initial state of the write flow to the stage where we read
        // the next entry from the queue.
        wait_and_call(STATE(read_queue));
        notRunning_ = true;

        hub->register_port(this);
    }

    void disconnect_and_delete() override
    {
        {
            AtomicHolder l(lock());
            HASSERT(!pendingShutdown_);
            input_.hub_->unregister_port(this);
            // After this, the next notify will eventually reach the shutdown
            // and delete state.
            pendingShutdown_ = true;
            if (notRunning_)
            {
                notRunning_ = 0;
                notify();
            }
        }
        // Synchronization point that ensures that there is no currently
        // running Executable on this executor. This ensures that is no
        // currently pending nor will there be any future invocations of
        // sendFunction_ after this function returns.
        service()->executor()->sync_run([]() {});
    }

    // ===== API from the BLE stack connection =====

    /// Called by the BLE stack, when a send function is completed.
    void ack()
    {
        ack_helper();
        LOG(ALWAYS, "BLE ack, pend %d", sendPending_);
    }

    /// Called by the BLE stack, when a send has failed.
    void nack()
    {
        ack_helper();
        LOG(ALWAYS, "BLE nack, pend %d", sendPending_);
    }

    /// Called by the BLE stack when input data arrives from this remote
    /// endpoint.
    ///
    /// @param data payload that arrived. The data will be copied inline, and
    /// does not need to exist beyond when this function returns.
    /// @param len number of bytes in the data payload.
    ///
    void input_data(const uint8_t *data, size_t len)
    {
        input_.input_data(data, len);
    }

    /// Synchronous output routine called by the hub.
    void send(MessageAccessor<uint8_t[]> *msg) override
    {
        if (pendingShutdown_)
        {
            // Port already closed. Ignore data to send.
            return;
        }
        {
            AtomicHolder h(lock());
            if (pendingTail_ && pendingTail_->buf_.try_append_from(msg->buf_))
            {
                totalPendingSize_ += msg->buf_.size();
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
            if (pendingShutdown_)
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
                // When we have exactly one buffer at hand, we release the done
                // notify of it to allow the upstream stack to generate more
                // data quicker.
                b->set_done(nullptr);
            }
            else
            {
                // flow already running. Skip notify.
                return;
            }
        }
        notify();
    }

    /// Entry point to the flow, when an outgoing message got into the queue
    /// and we are woken up.
    Action read_queue()
    {
        if (pendingShutdown_)
        {
            return call_immediately(STATE(shutdown_and_exit));
        }
        BufferType *head;
        {
            AtomicHolder h(lock());
            head = static_cast<BufferType *>(pendingQueue_.next_locked().item);
            if (!head)
            {
                notRunning_ = true;
                return wait();
            }
            HASSERT(head);
            if (head->data() == pendingTail_)
            {
                pendingTail_ = nullptr;
            }
            totalPendingSize_ -= head->data()->buf_.size();
        }

        currentHead_.reset(head);
        return call_immediately(STATE(do_write));
    }

    Action do_write()
    {
        if (pendingShutdown_)
        {
            return call_immediately(STATE(shutdown_and_exit));
        }

        const uint8_t *read_ptr;
        size_t num_bytes;
        auto &b = currentHead_->data()->buf_;
        read_ptr = b.data_read_pointer(&num_bytes);
        if (num_bytes > MAX_BYTES_PER_WRITE)
        {
            num_bytes = MAX_BYTES_PER_WRITE;
        }
        {
            AtomicHolder h(lock());
            ++sendPending_;
        }
        LOG(INFO, "BLE send %d bytes pendcount %d queuesize %d/%d", (int)num_bytes, sendPending_, (int)totalPendingSize_, pendingQueue_.pending());
        sendFunction_(read_ptr, num_bytes);
        b.data_read_advance(num_bytes);
        if (b.size())
        {
            return yield();
        }
        else
        {
            currentHead_.reset();
            AtomicHolder h(lock());
            if (pendingShutdown_)
            {
                return call_immediately(STATE(shutdown_and_exit));
            }
            if (sendPending_ > 1)
            {
                waitingForAck_ = 1;
                return wait_and_call(STATE(read_queue));
            }
            else if (pendingQueue_.empty())
            {
                // go back to sleep
                notRunning_ = 1;
                return wait_and_call(STATE(read_queue));
            }
            else
            {
                return yield_and_call(STATE(read_queue));
            }
        }
    }

    /// Invoked after pendingShutdown == true. At this point nothing gets added
    /// to the pending queue.
    Action shutdown_and_exit()
    {
        // Synchronization with other threads that might have written
        // pendingShutdown == true.
        {
            AtomicHolder h(lock());
        }

        // Releases all buffers.
        currentHead_.reset();
        while (auto *h = static_cast<BufferType *>(pendingQueue_.next().item))
        {
            h->unref();
        }
        return delete_this();
    }

protected:
    void ack_helper()
    {
        {
            AtomicHolder h(lock());
            --sendPending_;
            if (waitingForAck_ && sendPending_ <= 1)
            {
                waitingForAck_ = false;
                notify();
            }
        }
    }

    /// State flow that handles data arriving from this bluetooth connection.
    class InputFlow : public StateFlowBase, private Atomic
    {
    public:
        InputFlow(BLEHubPort* parent, DirectHubInterface<uint8_t[]> *hub,
            std::unique_ptr<MessageSegmenter> segmenter)
            : StateFlowBase(hub->get_service())
            , parent_(parent)
            , segmenter_(std::move(segmenter))
            , hub_(hub)
        {
            segmenter_->clear();
            flowWaiting_ = true;
            wait_and_call(STATE(take_input));
        }

        /// Called by the BLE stack when input data arrives from this remote
        /// endpoint.
        ///
        /// @param data payload that arrived. The data will be copied inline,
        /// and does not need to exist beyond when this function returns.
        /// @param len number of bytes in the data payload.
        ///
        void input_data(const uint8_t *data, size_t len)
        {
            while (len)
            {
                uint8_t *dst = nullptr;
                size_t free = 0;
                if (!appendBuf_.free())
                {
                    DataBuffer *p;
                    g_direct_hub_data_pool.alloc(&p);
                    appendBuf_.append_empty_buffer(p);
                }
                dst = appendBuf_.data_write_pointer();
                free = appendBuf_.free();
                if (len < free) {
                    free = len;
                }
                memcpy(dst, data, free);
                len -= free;
                data += free;
                appendBuf_.data_write_advance(free);
            }
            {
                AtomicHolder h(this);
                HASSERT(transferBuf_.try_append_from(appendBuf_, true));
                if (flowWaiting_) {
                    flowWaiting_ = false;
                    notify();
                }
            }
            appendBuf_.data_read_advance(appendBuf_.size());
        }

        /// Moves over data from the other thread which is in the
        /// transferBuf_. Continues on to segment and send them as messages to
        /// the hub.
        Action take_input() {
            AtomicHolder h(this);
            HASSERT(segmentBuf_.try_append_from(transferBuf_, true));
            transferBuf_.data_read_advance(transferBuf_.size());
            return call_immediately(STATE(segment_head));
        }

        /// Takes the head of segmentBuf_, and performs the message
        /// segmentation on it. Puts the resulting byte offsets into the
        /// outputBuf_.
        Action segment_head() {
            size_t len = 0;
            const uint8_t* ptr = segmentBuf_.data_read_pointer(&len);
            if (!len) {
                AtomicHolder h(this);
                if (transferBuf_.size()) {
                    return call_immediately(STATE(take_input));
                }
                flowWaiting_ = true;
                return wait_and_call(STATE(take_input));
            }
            ssize_t segment_size = segmenter_->segment_message(ptr, len);
            size_t xfer = len;
            if (segment_size) {
                xfer = segment_size - outputBuf_.size();
            }
            LinkedDataBufferPtr p;
            p.reset(segmentBuf_, xfer);
            segmentBuf_.data_read_advance(xfer);
            HASSERT(outputBuf_.try_append_from(p, true));
            if (segment_size) {
                segmenter_->clear();
                // completed data
                return call_immediately(STATE(send_output));
            } else {
                // Need to segment more data.
                return again();
            }
        }

        /// Called when one full message is segmented into outputBuf_. Sends
        /// this to the hub.
        Action send_output() {
            // We expect either an inline call to our run() method or
            // later a callback on the executor. This sequence of calls
            // prepares for both of those options.
            wait_and_call(STATE(send_callback));
            inlineCall_ = 1;
            sendComplete_ = 0;
            hub_->enqueue_send(this); // causes the callback
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
            auto *m = hub_->mutable_message();
            /// @todo do we need to add barriernotifiables here?
            //m->set_done(buf_.tail()->new_child());
            m->source_ = parent_;
            // This call transfers the chained head of the current buffers,
            // taking additional references where necessary or transferring the
            // existing reference. It adjusts the skip_ and size_ arguments in
            // buf_ to continue from where we left off.
            m->buf_ = std::move(outputBuf_);
            hub_->do_send();
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
            // Goes back to looking at more data from the transfered buffers.
            return call_immediately(STATE(segment_head));
        }
        
    private:
        friend class BLEHubPort;

        /// Owning outside flow.
        BLEHubPort* parent_;

        /// True if the flow is paused and waiting for more data to
        /// arrive. Guarded by atomic *this.
        bool flowWaiting_;
        /// 1 if we got the send callback inline from the read_done.
        uint8_t inlineCall_ : 1;
        /// 1 if the run callback actually happened inline.
        uint8_t sendComplete_ : 1;
        
        /// Current buffer for the input data. Owned by the input thread.
        LinkedDataBufferPtr appendBuf_;
        /// Buffer for transferring data rfrom the input thread to the service
        /// thread. Guarded by atomic *this.
        LinkedDataBufferPtr transferBuf_;
        /// Buffer for the data being segmented. Owned by (only manipulated on)
        /// the Service executor.
        LinkedDataBufferPtr segmentBuf_;
        /// Buffer for one sent message. This is the output of the segmenter.
        LinkedDataBufferPtr outputBuf_;
        /// Implementation (and state) of the business logic that segments
        /// incoming bytes into messages that shall be given to the hub.
        std::unique_ptr<MessageSegmenter> segmenter_;
        /// Parent hub where output data is coming from.
        DirectHubInterface<uint8_t[]> *hub_;
        
    };

    /// Holds the necessary information we need to keep in the queue about a
    /// single output entry. Automatically unrefs the buffer whose pointer we
    /// are holding when released.
    struct OutputDataEntry
    {
        LinkedDataBufferPtr buf_;
    };
    /// Type of buffers we are enqueuing for output.
    typedef Buffer<OutputDataEntry> BufferType;
    /// Type of the queue used to keep the output buffer queue.
    typedef Q QueueType;

    /// @return lock usable for the write flow and the port altogether.
    Atomic *lock()
    {
        return pendingQueue_.lock();
    }

    /// True if we have an error and we are trying to shut down.  Causes all
    /// outgoing data to be thrown away.
    bool pendingShutdown_ : 1;
    /// 1 if the state flow is paused, waiting for the notification.
    bool notRunning_ : 1;
    /// true if the write flow is paused waiting for the BLE stack to ack the
    /// data. Should be notified when the acknowledgements make sendPending_ <=
    /// 1.
    bool waitingForAck_ : 1;

    /// Contains buffers of OutputDataEntries to write. lock() is the internal
    /// lock of this object.
    QueueType pendingQueue_;
    /// Last tail pointer in the pendingQueue. If queue is empty,
    /// nullptr. Protected by pendingQueue_.lock().
    OutputDataEntry *pendingTail_ = nullptr;
    /// Total number of bytes in the pendingQueue. This does not include data
    /// that is in currentHead_. Protected by lock().
    size_t totalPendingSize_ = 0;

    /// The buffer that is taken out of the queue while flushing. This variable
    /// is not locked, because it is owned by the state flow states.
    BufferPtr<OutputDataEntry> currentHead_;

    /// Function object used to send out actual data. This is synchronously
    /// operating, meaning it makes a copy of the data to the stack for sending
    /// it out.
    SendFunction sendFunction_;
    /// This notifiable will be called before exiting.
    Notifiable *onError_ = nullptr;

    /// Number of in-flight messages sent but not acknowledged.
    int sendPending_ {0};

    InputFlow input_;

}; // class BLEHubPort

} // namespace openlcb

#endif // _OPENLCB_BLEHUBPORT_HXX_

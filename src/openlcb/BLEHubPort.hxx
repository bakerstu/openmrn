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
        , segmenter_(std::move(segmenter))
        , hub_(hub)
        , sendFunction_(std::move(send_function))
        , onError_(on_error)
    {
        // Sets the initial state of the write flow to the stage where we read
        // the next entry from the queue.
        wait_and_call(STATE(read_queue));
        notRunning_ = 1;

        segmenter_->clear();

        hub_->register_port(this);
    }

    void disconnect_and_delete() override
    {
        {
            AtomicHolder l(lock());
            HASSERT(!pendingShutdown_);
            hub_->unregister_port(this);
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
            HASSERT(head);
            if (head->data() == pendingTail_)
            {
                pendingTail_ = nullptr;
            }
        }

        currentHead_.reset(head);
        return do_write();
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
            if (pendingQueue_.empty())
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
    uint8_t notRunning_ : 1;

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

    /// Implementation (and state) of the business logic that segments
    /// incoming bytes into messages that shall be given to the hub.
    std::unique_ptr<MessageSegmenter> segmenter_;
    /// Parent hub where output data is coming from.
    DirectHubInterface<uint8_t[]> *hub_;
    /// Function object used to send out actual data. This is synchronously
    /// operating, meaning it makes a copy of the data to the stack for sending
    /// it out.
    SendFunction sendFunction_;
    /// This notifiable will be called before exiting.
    Notifiable *onError_ = nullptr;
}; // class BLEHubPort

} // namespace openlcb

#endif // _OPENLCB_BLEHUBPORT_HXX_

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
 * \file DirectHub.hxx
 *
 * Optimized class for ingress and egress of write-once objects, aimed to
 * support multi-recipient messages with fast internal fan-out but low compute
 * overhead.
 *
 * @author Balazs Racz
 * @date 9 Feb 2020
 */

#ifndef _UTILS_DIRECTHUB_HXX_
#define _UTILS_DIRECTHUB_HXX_

#include "utils/DataBuffer.hxx"
#include "executor/Executor.hxx"

class Service;

/// Empty class that can be used as a pointer for identifying where a piece of
/// data came from. Used as base class for hub ports.
class HubSource {};


/// Metadata that is the same about every message (independent of data type).
struct MessageMetadata {
    void clear()
    {
        if (done_)
        {
            done_->notify();
            done_ = nullptr;
        }
        source_ = dst_ = nullptr;
        isFlush_ = false;
    }

    /// This must be notified when the processing of the message is
    /// complete. All forks coming from the message must take children.
    BarrierNotifiable* done_ = nullptr;
    /// Represents the input port where the message came in.
    HubSource* source_ = nullptr;
    /// Represents the output port where the message will leave. nullptr means
    /// broadcast.
    HubSource* dst_ = nullptr;
    /// If true, this message should flush the output buffer.
    bool isFlush_ = false;
};

/// Typed message class. Senders to the hub will use this interface to fill in
/// the message details on the hub.
template<class T>
struct MessageAccessor : public MessageMetadata {
public:
    void clear()
    {
        payload_.reset();
        MessageMetadata::clear();
    }

    /// Contains a reference of the actual data.
    BufferPtr<T> payload_;
};

/// Type specializer for message interface when we are sending untyped data
/// (i.e. byte streams).
template<>
struct MessageAccessor<uint8_t[]> : public MessageMetadata {
    void clear() {
        // Walks the buffer links and unrefs everything we own.
        payload_->unref_all(skip_ + size_);
        payload_ = nullptr;
        skip_ = 0;
        size_ = 0;
        MessageMetadata::clear();
    }
    /// Pointer to the first buffer that contains this data. The first skip_
    /// bytes have to be ignored from this buffer. If size_ is larger than the
    /// remaining bytes, then the next pointer has to be followed to get to
    /// additional DataBuffer* objects. All of these objects have
    /// exactly one ref owned by *this.
    DataBuffer* payload_ = nullptr;
    /// How many bytes to skip at the beginning of the payload.
    unsigned skip_ = 0;
    /// How many bytes to read from the payload.
    unsigned size_ = 0;
};

/// Abstract base class for segmenting a byte stream typed input into
/// meaningful packet sized chunks.
///
/// Implementations have to be stateful and are instantiated per specific input
/// port. Implementations have to be thread-compatible.
class MessageSegmenter : public Destructable {
public:
    /// Makes a segmenting decision given more input data. This function will
    /// be called by the input routine repeatedly with the additional payload
    /// (non-overlapping) until the function returns a non-zero response.
    ///
    /// That response tells how many bytes long the last packet was, which must
    /// be <= the sum of the size arguments passed in. Thereafter the read flow
    /// will call clear() and call segment_message() again with the remaining
    /// partial buffer.
    ///
    /// @param data beginning of the buffer pointing to the next unsegmented
    /// data array.
    /// @param size how many bytes of data are available at this address. Must
    /// be >0.
    /// @return 0 if no complete packet was yet seen; positive value N if the
    /// packet that starts in the first segment_message call is N bytes
    /// long. Negative return are reserved (shall not be returned at this
    /// point).
    virtual ssize_t segment_message(const void* data, size_t size) = 0;

    /// Resets internal state machine. The next call to segment_message()
    /// assumes no previous data present.
    virtual void clear() = 0;
};

/// Interface for a downstream port of a hub (aka a target to send data to).
template <class T> class DirectHubPort : public HubSource
{
public:
    /// Send some data out on this port. The callee is responsible for
    /// buffering or enqueueing the data that came in this call.
    /// @param msg represents the message that needs to be sent. The callee
    /// must not modify the message.
    virtual void send(MessageAccessor<T> *msg) = 0;
};


/// Interface for a the central part of a hub.
template<class T>
class DirectHubInterface {
public:
    /// @return an executor service.
    virtual Service* get_service() = 0;

    /// Adds a port to this hub. This port will be receiving all further
    /// messages.
    /// @param port the downstream port.
    virtual void register_port(DirectHubPort<T>* port) = 0;
    /// Synchronously removes a port from this hub. This port must have been
    /// registered previously. Must not be called on the main executor.
    /// @param port the downstream port.
    virtual void unregister_port(DirectHubPort<T>* port) = 0;
    /// Asynchronously removes a port from this hub. This port must have been
    /// registered previously.
    /// @param port the downstream port.
    /// @param done will be notified when the removal is complete.
    virtual void unregister_port(DirectHubPort<T>* port, Notifiable* done) = 0;

    /// Signals that the caller wants to send a message to the hub. When the
    /// hub is ready for that, will execute *caller. This might happen inline
    /// within this function call, or on a different executor.
    /// @param caller callback that actually sends the message. It is required
    /// to call do_send() inline.
    virtual void enqueue_send(Executable* caller) = 0;

    /// Accessor to fill in the message payload. Must be called only from
    /// within the callback as invoked by enqueue_send.
    /// @return mutable structure to fill in the message. This structure was
    /// cleared by the hub before it is returned here.
    virtual MessageAccessor<T>* mutable_message() = 0;

    /// Sends a message to the hub. Before this is called, the message has to
    /// be filled in via mutable_message().
    virtual void do_send() = 0;
    
    /// @param source is the sender of the message. This must be equal of a
    /// registered hub port, otherwise the message will be returned to the
    /// caller as well.
    /// @param data is the payload to send.
    /// @param done non-null. If the hub cannot accept the incoming data, the hub will take a share of this barrier, and notify it 
    //virtual void send(HubSource* source, shared_ptr<const T> data, BarrierNotifiable* done, unsigned priority) = 0;
};

/// Creates a new byte stream typed hub.
DirectHubInterface<uint8_t[]>* create_hub(ExecutorBase* e);

/// Creates a hub port of byte stream type reading/writing a given fd. This
/// port will be automaticelly deleted upon any error reading/writing the fd
/// (unregistered and memory released).
/// @param hub hub instance on which to register the new port. Onwership
/// retained by caller.
/// @param fd where to read and write data.
/// @param segmenter is an newly allocated object for the given protocol to
/// segment incoming data into messages. Transfers ownership to the function.
/// @param on_error this will be notified if the port closes due to an error.
void create_port_for_fd(DirectHubInterface<uint8_t[]>* hub, int fd, std::unique_ptr<MessageSegmenter> segmenter, Notifiable* on_error = nullptr);

/// Creates a new GridConnect listener on a given TCP port. The object is
/// leaked (never destroyed).
/// @param hub incoming and outgoing data will be multiplexed through this hub
/// instance.
/// @param port the TCP port to listen on.
void create_direct_gc_tcp_hub(DirectHubInterface<uint8_t[]>* hub, int port);

/// Creates a message segmenter for gridconnect data.
/// @return a newly allocated message segmenter that chops gridconnect packets
/// off of a data stream.
MessageSegmenter* create_gc_message_segmenter();

/// Creates a message segmenter for arbitrary data. Each buffer is left alone.
/// @return a newly allocated message segmenter.
MessageSegmenter* create_trivial_message_segmenter();

#endif // _UTILS_DIRECTHUB_HXX_

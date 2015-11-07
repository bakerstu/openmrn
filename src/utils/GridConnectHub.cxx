/** \copyright
 * Copyright (c) 2013, Balazs Racz
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
 * \file GridConnectHub.cxx
 * Gridconnect parser/renderer pipe components.
 *
 * @author Balazs Racz
 * @date 20 May 2013
 */

//#define LOGLEVEL VERBOSE

#include "utils/GridConnectHub.hxx"

#include "executor/StateFlow.hxx"
#include "can_frame.h"
#include "utils/Buffer.hxx"
#include "utils/HubDevice.hxx"
#include "utils/Hub.hxx"
#include "utils/gc_format.h"

/// Actual implementation for the gridconnect bridge between a string-typed Hub
/// and a CAN-frame-typed Hub.
class GCAdapter : public GCAdapterBase
{
public:
    GCAdapter(HubFlow *gc_side, CanHubFlow *can_side, bool double_bytes)
        : parser_(can_side->service(), can_side, &formatter_)
        , formatter_(can_side->service(), gc_side, &parser_, double_bytes)
    {
        gc_side->register_port(&parser_);
        can_side->register_port(&formatter_);
        isRegistered_ = 1;
    }

    GCAdapter(HubFlow *gc_side_read, HubFlow *gc_side_write,
              CanHubFlow *can_side, bool double_bytes)
        : parser_(can_side->service(), can_side, &formatter_)
        , formatter_(can_side->service(), gc_side_write, &parser_, double_bytes)
    {
        gc_side_read->register_port(&parser_);
        can_side->register_port(&formatter_);
        isRegistered_ = 1;
    }

    virtual ~GCAdapter()
    {
        unregister();
    }

    void unregister()
    {
        if (isRegistered_)
        {
            parser_.destination()->unregister_port(&formatter_);
            /// @TODO(balazs.racz) This is incorrect if the 3-pipe constructor
            /// is
            /// used.
            formatter_.destination()->unregister_port(&parser_);
            isRegistered_ = 0;
        }
    }

    bool shutdown() OVERRIDE
    {
        unregister();
        return parser_.is_waiting() && formatter_.is_waiting();
    }

    /// HubPort (on a CAN-typed hub) that turns a binary CAN packet into a
    /// string-formatted CAN packet, and sends it off to the HubFlow (of type
    /// string).
    class BinaryToGCMember : public CanHubPort
    {
    public:
        BinaryToGCMember(Service *service, HubFlow *destination,
                         HubPort *skip_member, int double_bytes)
            : CanHubPort(service)
            , destination_(destination)
            , skipMember_(skip_member)
            , double_bytes_(double_bytes)
        {
        }

        HubFlow *destination()
        {
            return destination_;
        }

        Action entry()
        {
            LOG(VERBOSE, "can packet arrived: %" PRIx32,
                GET_CAN_FRAME_ID_EFF(*message()->data()));
            char *end =
                gc_format_generate(message()->data(), dbuf_, double_bytes_);
            size_t size = (end - dbuf_);
            if (size)
            {
                Buffer<HubData> *target_buffer;
                /// @todo(balazs.racz) switch to asynchronous allocation here.
                mainBufferPool->alloc(&target_buffer);
                target_buffer->data()->skipMember_ = skipMember_;
                /// @todo(balazs.racz) try to use an assign function for better
                /// performance.
                target_buffer->data()->resize(size);
                memcpy((char *)target_buffer->data()->data(), dbuf_, size);
                destination_->send(target_buffer, 0);
            }
            else
            {
                LOG(INFO, "gc generate failed.");
            }
            return release_and_exit();
        }

    private:
        /// Destination buffer (characters).
        char dbuf_[56];
        /// Pipe to send data to.
        HubFlow *destination_;
        /// The pipe member that should be sent as "source".
        HubPort *skipMember_;
        /// Non-zero if doubling was requested.
        int double_bytes_;
    };

    /// HubPort (on a string hub) that turns a gridconnect-formatted CAN packet
    /// into a binary CAN packet, and sends them off to the HubFlow (of CAN
    /// frame).
    class GCToBinaryMember : public HubPort
    {
    public:
        GCToBinaryMember(Service *service, CanHubFlow *destination,
                         CanHubPort *skip_member)
            : HubPort(service)
            , offset_(-1)
            , destination_(destination)
            , skipMember_(skip_member)
        {
        }

        CanHubFlow *destination()
        {
            return destination_;
        }

        /** Takes more characters from the pending incoming buffer. */
        Action entry()
        {
            inBuf_ = message()->data()->data();
            inBufSize_ = message()->data()->size();
            return call_immediately(STATE(parse_more_data));
        }

        Action parse_more_data()
        {
            while (inBufSize_--)
            {
                char c = *inBuf_++;
                if (consume_byte(c))
                {
                    // End of frame. Allocate an output buffer and parse the
                    // frame.
                    /// @todo(balazs.racz) use a configurable buffer pool
                    mainBufferPool->alloc(&outBuf_);
                    return call_immediately(STATE(parse_to_output_frame));
                }
            }
            // Will notify the caller.
            return release_and_exit();
        }

        /** Takes the completed frame in cbuf_, parses it into the allocation
         * result (a can pipe buffer) and sends off frame. Then comes back to
         * process buffer. */
        Action parse_to_output_frame()
        {
            // CanPipeBuffer *pbuf = GetTypedAllocationResult(&g_can_alloc);
            // pbuf->Reset();
            LOG(VERBOSE, "gc packet arrived: %s", cbuf_);
            int ret = gc_format_parse(cbuf_, outBuf_->data());
            if (!ret)
            {
                outBuf_->data()->skipMember_ = skipMember_;
                destination_->send(outBuf_);
            }
            else
            {
                // Releases the buffer.
                outBuf_->unref();
            }
            return call_immediately(STATE(parse_more_data));
        }

        /** Adds the next character from the source stream. Returns true if
         * cbuf_ contains a complete frame. */
        bool consume_byte(char c)
        {
            if (c == ':')
            {
                // Frame is starting here.
                offset_ = 0;
                return false;
            }
            if (c == ';')
            {
                if (offset_ < 0)
                {
                    return false;
                }
                // Frame ends here.
                cbuf_[offset_] = 0;
                offset_ = -1;
                return true;
            }
            if (offset_ >= static_cast<int>(sizeof(cbuf_) - 1))
            {
                // We overran the buffer, so this can't be a valid frame.
                // Reset and look for sync byte again.
                offset_ = -1;
                return false;
            }
            if (offset_ >= 0)
            {
                cbuf_[offset_++] = c;
            }
            else
            {
                // Drop byte to the floor -- we're not in the middle of a
                // packet.
            }
            return false;
        }

    private:
        /// Collects data from a partial GC packet.
        char cbuf_[32];
        /// offset of next byte in cbuf to write.
        int offset_;

        /// The incoming characters.
        const char *inBuf_;
        /// The remaining number of characters in inBuf_.
        size_t inBufSize_;

        /// The buffer to send to the destination hub.
        Buffer<CanHubData> *outBuf_;

        // ==== static data ====

        /// Pipe to send data to.
        CanHubFlow *destination_;
        /// The pipe member that should be sent as "source".
        CanHubPortInterface *skipMember_;
    };

private:
    /// PipeMember doing the parsing.
    GCToBinaryMember parser_;
    /// PipeMember doing the formatting.
    BinaryToGCMember formatter_;
    unsigned isRegistered_ : 1; //< 1 if the flows are registered.
};

GCAdapterBase *GCAdapterBase::CreateGridConnectAdapter(HubFlow *gc_side,
                                                       CanHubFlow *can_side,
                                                       bool double_bytes)
{
    return new GCAdapter(gc_side, can_side, double_bytes);
}

GCAdapterBase *GCAdapterBase::CreateGridConnectAdapter(HubFlow *gc_side_read,
                                                       HubFlow *gc_side_write,
                                                       CanHubFlow *can_side,
                                                       bool double_bytes)
{
    return new GCAdapter(gc_side_read, gc_side_write, can_side, double_bytes);
}

/// Implementation for the gridconnect bridge. Owns all necessary structures,
/// and is responsible for the initialization, registering, unregistering and
/// destruction of these structures.
struct GcPacketPrinter::Impl
{
    Impl(CanHubFlow *can_hub, bool timestamped)
        : canHub_(can_hub)
        , gcHub_(canHub_->service())
        , displayPort_(canHub_->service(), timestamped)
        , formatter_(canHub_->service(), &gcHub_, nullptr, false)
    {
        gcHub_.register_port(&displayPort_);
        canHub_->register_port(&formatter_);
    }

    ~Impl()
    {
        canHub_->unregister_port(&formatter_);
        gcHub_.unregister_port(&displayPort_);
    }

    CanHubFlow *canHub_;
    HubFlow gcHub_;
    DisplayPort displayPort_;
    GCAdapter::BinaryToGCMember formatter_;
};

GcPacketPrinter::GcPacketPrinter(CanHubFlow *can_hub, bool timestamped) : impl_(new Impl(can_hub, timestamped))
{
}

GcPacketPrinter::~GcPacketPrinter()
{
}

/// Implementation class that adds a device to a CAN hub with dynamic
/// translation of the packets to/from GridConnect format.
///
/// Sends a notification to the application level when there is an error on the
/// device and the connection is closed.
struct GcHubPort : public Executable
{
    GcHubPort(CanHubFlow *can_hub, int fd, Notifiable *on_exit)
        : gcHub_(can_hub->service())
        , bridge_(
              GCAdapterBase::CreateGridConnectAdapter(&gcHub_, can_hub, false))
        , gcWrite_(&gcHub_, fd, this)
        , onExit_(on_exit)
    {
        LOG(VERBOSE, "gchub port %p", (Executable *)this);
    }
    virtual ~GcHubPort()
    {
    }

    /** This hub sees the character-based representation of the packets. The
     * members of it are: the bridge and the physical device (fd).
     *
     * Destruction requirement: HubFlow should be empty. This means after the
     * disconnection of the bridge (write side) and the FdHubport (read side)
     * we need to wait for the executor until this flow drains. */
    HubFlow gcHub_;
    /** Translates packets between the can-hub of the device and the char-hub
     * of this port.
     *
     * Destruction requirement: Call shutdown() on the can-side executor (and
     * yield) until it returns true. */
    std::unique_ptr<GCAdapterBase> bridge_;
    /** Reads the characters from the char-hub and sends them to the
     * fd. Similarly, listens to the fd and sends the read charcters to the
     * char-hub. */
    FdHubPort<HubFlow> gcWrite_;
    /** If not null, this notifiable will be called when the device is
     * closed. */
    Notifiable* onExit_;

    /** Callback in case the connection is closed due to error. */
    void notify() OVERRIDE
    {
        /* We would like to delete *this but we cannot do that in this
         * callback, because we don't know what executor we are running
         * on. Deleting on the write executor would cause a deadlock for
         * example. */
        gcHub_.service()->executor()->add(this);
    }

    void run() OVERRIDE
    {
        if (!bridge_->shutdown() || !gcHub_.is_waiting())
        {
            // Yield.
            gcHub_.service()->executor()->add(this);
            return;
        }
        LOG(INFO, "GCHubPort: Shutting down gridconnect port %d. (%p)",
            gcWrite_.fd(), bridge_.get());
        if (onExit_) {
            onExit_->notify();
            onExit_ = nullptr;
        }
        /* We get this call when something is wrong with the FDs and we need to
         * close the connection. It is guaranteed that by the time we got this
         * call the device is unregistered from the char bridge, and the
         * service thread is ready to be stopped. */
        delete this;
    }
};

void create_gc_port_for_can_hub(CanHubFlow *can_hub, int fd, Notifiable* on_exit)
{
    new GcHubPort(can_hub, fd, on_exit);
}

/** \copyright
 * Copyright (c) 2022, Balazs Racz
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
 * \file StreamReceiver.hxx
 *
 * Implementation flow for the Stream Service that receives data to a remote
 * source using the stream protocol.
 *
 * @author Balazs Racz
 * @date 3 May 2022
 */

#ifndef _OPENLCB_STREAMRECEIVER_HXX_
#define _OPENLCB_STREAMRECEIVER_HXX_

#include "openlcb/StreamReceiverInterface.hxx"

#include "openlcb/IfCan.hxx"
#include "openlcb/StreamDefs.hxx"
#include "utils/ByteBuffer.hxx"
#include "utils/LimitedPool.hxx"

namespace openlcb
{

class StreamReceiverCan : public StreamReceiverInterface
{
public:
    /// Constructor.
    ///
    /// @param interface the CAN interface that owns this stream receiver.
    /// @param local_stream_id what should be the local stream ID for the
    /// streams used for this receiver.
    StreamReceiverCan(IfCan *interface, uint8_t local_stream_id);

    ~StreamReceiverCan();

    /// Implements the flow interface for the request API. This is not based on
    /// entry() because the registration has to be synchrnous with the calling
    /// of send().
    void send(Buffer<StreamReceiveRequest> *msg, unsigned prio = 0) override;

    /// Cancels the currently pending stream receive request. The message will
    /// then be asynchronously returned using the regular mechanism with a
    /// temporary error.
    void cancel_request() override;
    
private:
    /// Helper function for send() when a stream has to start synchronously.
    void announced_stream();

    /// This state is not used, but it's virtual abstract.
    Action entry() override
    {
        return return_ok();
    }

    Action wait_for_wakeup()
    {
        if (pendingCancel_)
        {
            return call_immediately(STATE(wakeup));
        }
        isWaiting_ = 1;
        return wait_and_call(STATE(wakeup));
    }

    /// Root of the flow when something happens in the handlers.
    Action wakeup();

    /// Invoked when we get the stream initiate request. Initializes receive
    /// buffers and sends stream init response.
    Action init_reply();
    Action init_buffer_ready();

    /// Invoked when the stream window runs out. Maybe waits for the data to be
    /// consumed below the low-watermark.
    Action window_reached();
    /// Called when the allocation of the raw buffer is successful. Sends off
    /// the stream proceed message.
    Action have_raw_buffer();

    /// Invoked by the GenericHandler when a stream initiate message arrives.
    ///
    /// @param message buffer with stream initiate message.
    ///
    void handle_stream_initiate(Buffer<GenMessage> *message);

    /// Handles data arriving from the network.
    inline void handle_bytes_received(const uint8_t *data, size_t len);

    /// Invoked by the GenericHandler when a stream complete message arrives.
    ///
    /// @param message buffer with stream complete message.
    ///
    void handle_stream_complete(Buffer<GenMessage> *message);

    /// Removes all handlers that are registered.
    void unregister_handlers();
    
    /// @return the local CAN interface.
    IfCan *if_can()
    {
        return static_cast<IfCan *>(service());
    }

    /// @return the local node pointer.
    Node *node()
    {
        return request()->dst_;
    }

    /// Helper class for incoming message for stream initiate.
    MessageHandler::GenericHandler streamInitiateHandler_ {
        this, &StreamReceiverCan::handle_stream_initiate};

    class StreamDataHandler;
    friend class StreamDataHandler;

    /// Helper class for incoming message for stream complete.
    MessageHandler::GenericHandler streamCompleteHandler_ {
        this, &StreamReceiverCan::handle_stream_complete};

    /// This pool is used to allocate one raw buffer per stream window
    /// size. This pool therefore functions as a throttling for the data
    /// producer. We have a fixed size of 2, meaning that we are allowing
    /// ourselves to load 2x the stream window size into our RAM.
    LimitedPool lastBufferPool_ {sizeof(RawBuffer), 2, rawBufferPool};

    /// The buffer that we are currently filling with incoming data.
    ByteBufferPtr currentBuffer_;

    /// The buffer that will be the last one in this stream window. This buffer
    /// comes from the lastBufferPool_ to function as throttling signal.
    RawBufferPtr lastBuffer_;

    /// Helper object that receives the actual stream CAN frames.
    std::unique_ptr<StreamDataHandler> dataHandler_;

    /// How many bytes we have transmitted in this stream so far.
    size_t totalByteCount_;

    /// Remaining stream window size.
    uint16_t streamWindowRemaining_;

    /// Unique stream ID at the destination (local) node, assigned at
    /// construction time.
    const uint8_t assignedStreamId_;

    /// 1 if we received the stream complete message.
    uint8_t streamClosed_ : 1;
    /// 1 if we received the stream init request message.
    uint8_t pendingInit_ : 1;
    /// 1 if we received a cancel request
    uint8_t pendingCancel_ : 1;
    /// 1 if we are currently waiting for a notification
    uint8_t isWaiting_ : 1;
}; // class StreamReceiver

} // namespace openlcb

#endif // _OPENLCB_STREAMRECEIVER_HXX_

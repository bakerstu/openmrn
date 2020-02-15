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


/// Empty class that can be used as a pointer for identifying where a piece of
/// data came from.
class HubSource {};

/// Interface for a downstream port of a hub (aka a target to send data to).
template <class T> class DirectHubPort : public HubSource
{
public:
    /// Send some data out on this port. The callee is reqponsible for
    /// enqueueing the data that came in this call if the available buffer
    /// space allows that.
    /// @param data is a share of the payload. The callee must not modify this
    /// copy.
    /// @param done non-null. If the callee cannot accept the incoming data,
    /// they should take a share of this barrier. The callee is then
    /// responsible for notifying it, and then the caller will re-try the call.
    /// @param priority hint to the callee on where to enqueue this message, if
    /// reordering is supported.
    virtual void send(
        shared_ptr<const T> data, BarrierNotifiable *done, unsigned priority) = 0;
};


/// Interface for a the central part of a hub.
///
/// Theory of operation:

/// ===Admission controller===
///
/// When a caller has a packet to send, it goes first through an admission
/// controller. The admission controller is specific to the source port. If the
/// admission controller rejects the packet, we hold the caller, and save the
/// notification the caller gave us to let the caller know when they can
/// proceed.
///
/// QQ is it possible that a single source has more than one packet to send us,
/// possibly at different priorities, and the admission controller would block
/// one then allow a different? Probably yes; a priority based traffic selector
/// behaves like this. We must ensure that traffic of the same priority and
/// src/dst pair always is queued and never independently presented to the
/// admission controller.
///
/// Failed requests of the admission controller have to line up in a queue like
/// the Async Allocator. When there is a token available, the queue front
/// should be woken up / notified. This gives a constraint on what the class
/// interface of the incoming request should be: an Executable has an
/// alloc_result() call.
///
/// The most straightforward implementation of an admission controller is to
/// keep track of the number of inflight objects. This automatically pushes
/// back on the src-dst pairs that generate too high traffic even in the face
/// of smart routing.
///
/// QQ how long do we need to keep around the existence of the pending object?
/// Ideally exactly until the pending object fully leaves the memory of the
/// device. If there is an outgoing assembly buffer that collects bytes of
/// outgoing data, we must ensure that this outgoing buffer is also kept in
/// consideration. However, so long as there is empty space in this buffer we
/// should not prevent further ingress. Once there is no empty space in the
/// buffer, we have to limit the number of items that queue up to be >= the
/// maximum single-source input entries. This will cause pushback on the
/// ingress path. This means that after the buffer is complete, we still have
/// to queue some packets.

/// ===Entry flow / queueing===
/// 
/// We need to make the router be have as little overhead as possible. This
/// means that almost everything about the router should be happening inline
/// instead of asynchronously / via queueing.
///
/// If a message passed the admission controller, we want to first check if the
/// router is empty. If yes, we should grab a lock and start processing the
/// message inline. If the router is busy, we should queue the incoming
/// message.
///
/// The router goes back to empty if after processing the message the queue is
/// found to be empty. Otherwise the router remains busy and the queue front is
/// taken out for processing.
///
/// QQ should an inline exeution of the router also de-queue the front message
/// and continue there or rather wakeup the flow and yield? Probably yield to
/// prevent the caller form being held up with uninteresting packet processing.

/// ===Runner flow===
///
/// The hub has a current message at any point in time. This is the message
/// that was taken out of the queue. When the router is running inline, the
/// current message is what the caller passed in, and the queue is empty.
///
/// The first thing the runner flow determines is if we have a broadcast or
/// unicast packet. For unicast packet we will have to look up directly the
/// output port that it should go to. For broadcast packet we have to
/// iterate. The output port might have to go through an indirection: some
/// output ports are going to be represented by a bridge (e.g. CAN to TCP
/// bridge) first and another routing hub on the far side of it.
///
/// When we give the packet to an output port, that operation should never
/// block the router. We should rather block the incoming port than the
/// router. It's the job of the incoming admission controller to hold back.

/// ===Output buffering===
///
/// For TCP based output ports (both gridconnect-CAN-TCP and native TCP, but
/// not gridconnect-USB) we should create a packet reassembly buffer. This is
/// needed because we want to reduce the number of kernel calls to make
/// everything cheaper, especially that on the CC32xx platform each kernel call
/// turns effectively into a packet to be sent to the network.
///
/// We have two choices on how to handle this output buffer:
///
/// 1) have a fixed memory pre-allocated, and as packets arrive at the output
/// port, we copy the bytes into this memory.
///
/// 2) We queue up the packets in the output port with their original buffer,
/// keeping a ref to those buffers, and when we decide to flush, we allocate
/// the output buffer, copy the bytes into it, send off the data, then free the
/// output buffer.
///
/// Option 2 uses less memory because the half empty output buffer is not kept
/// around. Both options have the same number of memory copies (we cannot avoid
/// the etra copy unless we have some scatter-gather DMA thing). Option 1 is
/// easier for understanding when the original source admission controller can
/// release the token. Option 2 allows high priority packets to jump to the
/// front of the queue just before the flush is happening.
///
/// I have no decision yet on which option to take.


/// ===Message representation in transit===
///
/// It is important to separate the queuing concept from the shared ownership
/// of the data payload. This is because the same data payload might be owned
/// by multiple output ports, but if the queue next pointer is owned internally
/// to it, we cannot effectively share.
///
/// Option 1) BufferBase has the ability to do refcounting and shared
/// ownership. It is possible to have a BufferBase that has untyped payload
/// (i.e., just bytes).
///
/// Option 2) shared_ptr<string> is a standard template library solution to the
/// same problem. However, shared_ptr<string> causes two memory allocations for
/// payloads that are longer than 16 bytes, and it has a minimum of 36 bytes
/// length (+payload length+4 if more than 16 bytes).
///
/// Assume that the input message could be split between two shared buffer
/// ownerships. This means that the queue entry needs two buffer pointers, an
/// offset, and a length.
///
/// We could use the buffer base next pointers to string up buffers that have
/// data from the same message, even if it's more than two. This way we only
/// need one buffer pointer. We have to assume that the respective bytes always
/// go together.

///
/// There should be a "flush" porperty of a message, which should cause
/// outgoing buffers to be immediately transmitted in order to achieve lower
/// roundtrip latency of important requests. We can be smart about what to
/// flush:
/// - last segment of datagrams (:X1Axxx, :X1Dxxx)
/// - last segments of stream writes (:X1F that has less than 8 data bytes)
/// - datagram ACK and stream proceed messages
/// - loco function control messages
/// - emergency stop messages
///

template<class T>
class DirectHubInterface : public HubSource {
public:
    /// Adds a port to this hub. This port will be receiving all further
    /// messages.
    /// @param port the downstream port.
    virtual void register_port(DirectHubPort<T>* port) = 0;
    /// Removes a port from this hub. This port must have been registered
    /// previously.
    /// @param port the downstream port.
    virtual void unregister_port(DirectHubPort<T>* port) = 0;

    /// Send a piece of data to the hub.
    /// @param source is the sender of the message. This must be equal of a
    /// registered hub port, otherwise the message will be returned to the
    /// caller as well.
    /// @param data is the payload to send.
    /// @param done non-null. If the hub cannot accept the incoming data, the hub will take a share of this barrier, and notify it 
    virtual void send(HubSource* source, shared_ptr<const T> data, BarrierNotifiable* done, unsigned priority) = 0;
}

#endif // _UTILS_DIRECTHUB_HXX_

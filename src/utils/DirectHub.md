# DirectHub design

DirectHub is a high performance router component that is suited to do the
forwarding of packets to multiple receivers across multiple interface types,
with or without packet filtering, with low CPU and latency overhead.

## Theory of operation

### Admission controller

When a caller has a packet to send, it goes first through an admission
controller. The admission controller is specific to the source port. If the
admission controller rejects the packet, we hold the caller, and save the
notification the caller gave us to let the caller know when they can
proceed.

QQ is it possible that a single source has more than one packet to send us,
possibly at different priorities, and the admission controller would block
one then allow a different? Probably yes; a priority based traffic selector
behaves like this. We must ensure that traffic of the same priority and
src/dst pair always is queued and never independently presented to the
admission controller.

Failed requests of the admission controller have to line up in a queue like
the Async Allocator. When there is a token available, the queue front
should be woken up / notified. This gives a constraint on what the class
interface of the incoming request should be: an Executable has an
alloc_result() call.

The most straightforward implementation of an admission controller is to
keep track of the number of inflight objects. This automatically pushes
back on the src-dst pairs that generate too high traffic even in the face
of smart routing.

There is also no admission control for CAN hardware, because we do not have the
ability to push back on traffic.

When admission control has woken up an input stream, that does not mean the
stream has the ability to execute directly on the hub. The hub might be busy.

QQ how long do we need to keep around the existence of the pending object?
Ideally exactly until the pending object fully leaves the memory of the
device. If there is an outgoing assembly buffer that collects bytes of
outgoing data, we must ensure that this outgoing buffer is also kept in
consideration. However, so long as there is empty space in this buffer we
should not prevent further ingress. Once there is no empty space in the
buffer, we have to limit the number of items that queue up to be >= the
maximum single-source input entries. This will cause pushback on the
ingress path. This means that after the buffer is complete, we still have
to queue some packets.

### Entry flow / queueing

We need to make the router be have as little overhead as possible. This
means that almost everything about the router should be happening inline
instead of asynchronously / via queueing.

If a message passed the admission controller, we want to first check if the
router is empty. If yes, we should grab a lock and start processing the
message inline. If the router is busy, we should queue the incoming
message.

The router goes back to empty if after processing the message the queue is
found to be empty. Otherwise the router remains busy and the queue front is
taken out for processing.

QQ should an inline exeution of the router also de-queue the front message
and continue there or rather wakeup the flow and yield? Probably yield to
prevent the caller form being held up with uninteresting packet processing.

### Entry API

This is an integrated API that will internally consult the admission
controller. There are three possible outcomes of an entry call:
1. admitted and execute inline
2. admitted but queued
3. not admitted, blocked asynchronously.

To support the synchronous execution and queueing at the same time, there
has to be a boolean parameter that comes back to tell whether we are
allowed to proceed or not.

When blocked, the best solution is to queue Executables (these are
queueable). We can choose to schedule them onto the executor (yield) or run
them inline when the admission controller says they are released.

It is also meaningful to say that the inline execution just means that we
run() the executable instead of notify()'ing it.

The syntax to do this from a calling StateFlow:

```
Action have_message() {
  wait_and_call(STATE(fill_request));
  target->try_send(this);
  return wait();
}

Action fill_request() {
  target->do_send(params);
  // should not be call_immediately() because threading is not certain at this point.
  return yield_and_call(STATE(something_next)); 
}
```

A simplified implementation of the admission controller and inbound queueing
would rely on just queueing the Executable's that are arriving on the API. When
a given input has its turn, we just call that Executable.


### Exit API

It is pretty important to have the exit API compatible with the Entry API in
case we have a bridge that connects to different Hubs with a data conversion
inbetween.
However, there is no admission control between different Hubs.

For hubs that are 1:1 in their representation of messages, a synchronous
execution is desirable.


### Runner flow

The hub has a current message at any point in time. This is the message
that was taken out of the queue. When the router is running inline, the
current message is what the caller passed in, and the queue is empty.

The first thing the runner flow determines is if we have a broadcast or
unicast packet. For unicast packet we will have to look up directly the
output port that it should go to. For broadcast packet we have to
iterate. The output port might have to go through an indirection: some
output ports are going to be represented by a bridge (e.g. CAN to TCP
bridge) first and another routing hub on the far side of it.

When we give the packet to an output port, that operation should never
block the router. We should rather block the incoming port than the
router. It's the job of the incoming admission controller to hold back.

### Output buffering

For TCP based output ports (both gridconnect-CAN-TCP and native TCP, but
not gridconnect-USB) we should create a packet reassembly buffer. This is
needed because we want to reduce the number of kernel calls to make
everything cheaper, especially that on the CC32xx platform each kernel call
turns effectively into a packet to be sent to the network.

We have two choices on how to handle this output buffer:

1) have a fixed memory pre-allocated, and as packets arrive at the output
port, we copy the bytes into this memory.

2) We queue up the packets in the output port with their original buffer,
keeping a ref to those buffers, and when we decide to flush, we allocate
the output buffer, copy the bytes into it, send off the data, then free the
output buffer.

Option 2 uses less memory because the half empty output buffer is not kept
around. Both options have the same number of memory copies (we cannot avoid
the etra copy unless we have some scatter-gather DMA thing). Option 1 is
easier for understanding when the original source admission controller can
release the token. Option 2 allows high priority packets to jump to the
front of the queue just before the flush is happening. Option 1 takes less
memory in that broadcast packets do not need to be queued at each single
output port.

I have no decision yet on which option to take.

QQ. How does the admission controller keep track of when the message really
exited the system? We must keep it in mind that the message might have been
translated to a differen physical format, and the memory for the specific
byte buffer be freed.

### Message representation in transit

It is important to separate the queuing concept from the shared ownership
of the data payload. This is because the same data payload might be owned
by multiple output ports, but if the queue next pointer is owned internally
to it, we cannot effectively share.

Option 1) BufferBase has the ability to do refcounting and shared
ownership. It is possible to have a BufferBase that has untyped payload
(i.e., just bytes). However, the BufferBase needs to know the amount of
bytes as input; we cannot trim down the actual bytes read from the
BufferBase's size field, or else we lose memory because after freeing the
buffer will not be returned to the right size. An alternative possibility
is to have a buffer pool that generates a single size buffer so everything
is returned to the same queue. Then size can be adjusted to the exact
number of bytes read. This might utilize a proxy buffer pool that takes
buffer of a given size from the main buffer pool and then returns them
there upon freeing, resetting the size to have them land in the appropriate
bucket.

Option 2) `shared_ptr<string>` is a standard template library solution to the
same problem. However, `shared_ptr<string>` causes two memory allocations for
payloads that are longer than 16 bytes, and it has a minimum of 36 bytes
length (+payload length+4 if more than 16 bytes).

Assume that the input message could be split between two shared buffer
ownerships. This means that the queue entry needs two buffer pointers, an
offset, and a length.

We could use the buffer base next pointers to string up buffers that have
data from the same message, even if it's more than two. This way we only
need one buffer pointer. We have to assume that the respective bytes always
go together.

Message type should then have:

BufferPtr<> first; // first chunk of message
BufferPtr<> second; if chunks are represented then the second chunk.
uint16_t ofs; // offset of bytes in the first message
uint16_t size; // how many bytes starting from that offset.
bool is_flush;
unsigned priority; // maybe only 4 bands.
HubSource* source;  // input / caller
HubSource* destination; // routing hint
something typing back to the original sender admission controller, for
    example a BarrierNotifiable?
TODO

It might make sense to support appending another message to the end of the
buffers. This be especially true if the last buffer pointer is just
partially used up, and thus the bufferptr at the end of the string of
buffers is the same as the incoming next buffer.



### Input segmentation

When data arrives from the socket to be read, we will allocate a shareable
buffer, then execute the asynchronous read. As the read completes, the
input data will be passed on to the segmenter. The goal of the segmenter is
to find the boundary of the message, for gridconnect the : ... ; delimiter,
and on TCP the binary length of the message. Then the message can be passed
on to routing.

The segmenter might come in in with one ref of a buffer, and output two
independent refs of the same buffer. This happens if a single kernel read
ends up with more than one message.

It is also possible that the segmenter will retain a ref of the last read
buffer, waiting for the completion of the message that is present therein.

We must keep reading bytes from the hardware until the segmenter is happy
to send at least one packet onwards. Only thereafter should we consult the
admission controller.

QQ do we ask the admission controller first, before we allocate the buffer
to read, or do we ask it afterwards when the message is complete?

We would have the ability to have select wake us up first, then allocate
the buffer, then execute the read.

### Flushing

There should be a "flush" porperty of a message, which should cause
outgoing buffers to be immediately transmitted in order to achieve lower
roundtrip latency of important requests. We can be smart about what to
flush:
- last segment of datagrams (:X1Axxx, :X1Dxxx)
- last segments of stream writes (:X1F that has less than 8 data bytes)
- datagram ACK and stream proceed messages
- loco function control messages
- emergency stop messages


### Bridges

In a real CAN-TCP router we will need to have three separate instances of
this router type: one for TCP messages, one for CAN frames, one for
GridConnect text.

The respective instances have to be connected by bridges. A bridge is an
output port from one router and an input port to another router, allocates
memory for the data type conversion, but does not perform admisson control
(this is because admission control is end to end). The bridge might be a
state flow, because for ->CAN messages the bridge has to perform alias
lookups or allocation.

It is unclear whether we also need routers / bridges for internal only
reference of messages (e.g. Buffer<GenMessage>). Is the
AddressedMessageWriteFlow a component on the If side or is it a component
that is part of the connecting bridge? It has to exist in the bridge for
the purpose of routing messages between different types of Ifs.

### Route lookups

When a message shows up in a router, we have to perform a lookup to decide
where it goes. The output of this lookup is either "everywhere" (broadcast
message), which means every output except the same as where it came
from. The other alternative is "to a specific target" which is then one
single port. That port might not be on the current router though. Thus we
need a secondary routing table as well, which maps HubSources to ports or
bridges in the current, type-specific router. This secondary routing table
exists independently for each router; however, the primary routing table
only exists once globally.

In every case we will test the packet just before output against the
specific HubInput that it came from in order to avoid echoing it back.  The
HubInput may have to be changed between specific routers; or alternatively
we could use the secondary lookup table for the current router. It is
sufficient to perform that secondary lookup only once, as the current
router starts to process the message, if it has been identified that it is
a broadcast message. This basically means that the skipMember (source) can
be a class variable inside the hub flow itself.

The bridge could actually also directly represent the skipMember when
sending the packet to the target router. However, we want to make sure the
enqueueing of the message does not cause a loss of information. We could
have the enqueueing be based on an execution of a piece of code.

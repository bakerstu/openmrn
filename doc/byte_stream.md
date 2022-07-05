# Byte Streams

***WARNING*** this is currently a proposal that is not implemented yet.

This document describes a mechanism by which OpenMRN components can exchange a
unidirectional untyped data stream. The API is laid out for compatibility with
the StateFlow concept and provides asynchronous implementations to limit memory
usage and flow control.

## Use-cases

- OpenLCB Stream sending. The client component sends data to the stream
  transmitter using Byte Streams.
  
- OpenLCB Stream receiving. The client component is receiving data from the
  Stream service using Byte Streams.
  
- A fast implementation of a TCP Hub should use Byte Streams to represent the
  TCP messages for proxying between ports.
  
## Non-goals

- It is not a goal to use the Byte Stream API to write from a State Flow to an
  fd. (Neither sockets, pipes, nor physical files.) Writing to an fd should be
  done by the native StateFlow features like `write_repeated`.

## Requirements

- Memory allocation

  - Allocation has to avoid memory fragmentation. While the expectation is that
    larger blocks of memory are allocated in one go, these blocks should be
    reused between different users of Byte Streams and at different times,
    instead of returning to malloc/free.
  
  - Block allocation size should be 1 kbyte. This is small enough that even
    32kbyte MCUs can use the codebase, but large enough to capture meaningful
    TCP packet sizes.
  
- Use and copy

  - The implementation should be zero-copy. Once some data is in a byte buffer,
    that data should not need to be copied in order to forward it to other byte
    buffers or byte streams.
    
  - Reference counting should be available when the payload needs to be used
    in multiple places.
    
  - It should be possible to take a subset of a buffer and send it to a
    different byte stream. (Routers will do this.)
    
  - A special zero-copy mechanism should be available when the source data is
    already in memory or in flash. Sending these bytes into the flow should not
    need block allocation and data copy.

- Flow control

  - The data source has to be blocked in memory allocation when the sink has
    not yet consumed the data.
    
  - The amount of read-ahead should be configurable (i.e., how much memory does
    the source fill in before it gets blocked on waiting for the sink).


## Implementation

We define two buffer types.

- `using RawBuffer = Buffer<char[1024]>;`

  This buffer holds the raw data. This buffer is reference counted, shared
  between all sinks, and never entered into a `Q`. All sinks consider this
  buffer as read-only.

- `using ByteBuffer = Buffer<ByteChunk>;`

  This buffer holds a reference to a RawBuffer, and start/end pointers within
  that describe the exact range of bytes to use. This buffer is not shareable,
  it can be entered into a queue, i.e., sent to a `StateFlowWithQueue` /
  `FlowInterface<ByteBuffer>`.

### Memory allocation

The `RawBuffer` data blocks should come from `rawBufferPool`, which is a
`DynamicPool` that is not the same as mainBufferPool, but instantiated with a
single bucket of ~1 kbyte size (technically, `sizeof(RawBuffer)`). This ensures
that the memory fragmentation requirements are met. When a RawBuffer is
released, it goes back to the freelist of the `rawBufferPool`.

To limit the amount of memory allocated, a `LimitedPool` can be instantiated by
the data source which specifies a fixed number of 1kbyte blocks. The backing
pool shall be set to `rawBufferPool`.

### Memory ownership / deallocation

`ByteChunk` contains an `BufferPtr<RawBuffer>`, which is a unique_ptr that
unref's the buffer upon the destructor. This represents the ownership of a
reference to a `RawBuffer`. The destructor of `ByteChunk` will automatically
release this reference.

It is optional for a ByteChunk to own a RawBuffer reference. A ByteChunk can
also be created from externally-owned memory, such as flash. In this case the
`unique_ptr` remains as nullptr, and no unref happens in the destructor.

### Zero-copy

To make a copy of a piece of data, a new ByteBuffer is allocated, and
initialized with a new reference of the same RawBuffer. The copy can have the
same data, or a contiguous substring of the data. Using a substring is helpful
for example when we take the payload of an incoming TCP stream message and
forward it to the stream reader client. It can also be helpful when taking an
incoming message, taking off the header, prepending a new header, and sending
it out on a different port.

## Definition

```
struct RawData
{
    uint8_t payload[1024];
};

using RawBuffer = Buffer<RawData>;

struct ByteChunk
{
    BufferPtr<RawData> ownedData_;
    
    uint8_t* data_ {nullptr};

    size_t size_ {0};
};

using ByteBuffer = Buffer<ByteChunk>;

using ByteSink = FlowInterface<Buffer<ByteChunk>>;
```

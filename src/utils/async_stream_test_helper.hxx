#include "utils/async_datagram_test_helper.hxx"
#include "openlcb/StreamReceiver.hxx"

namespace openlcb
{

static constexpr uint8_t LOCAL_STREAM_ID = 0x3a;
static constexpr uint8_t SRC_STREAM_ID = 0xa7;

/// Generates some deterministic data to send via streams.
string get_payload_data(size_t length)
{
    string r(length, 0);
    for (size_t i = 0; i < length; ++i)
    {
        r[i] = i & 0xff;
    }
    return r;
}

/// Helper class that acts as a data sink for a stream receiver. This class
/// collects the bytes in a string. By default the stream is unthrottled (all
/// buffers are immediately freed), but there are provisions to let the test
/// manually drive how fast the incoming data buffers get freed.
struct CollectData : public ByteSink
{
    /// Bytes that arrived so far.
    string data;
    /// Holds buffers.
    Q q;
    /// if true, the buffers are added to the queue instead of unref'ed.
    bool keepBuffers_ {false};

    void send(ByteBuffer *msg, unsigned prio) override
    {
        auto rb = get_buffer_deleter(msg);
        data.append((char *)msg->data()->data_, msg->data()->size());
        if (keepBuffers_)
        {
            q.insert(msg->ref());
        }
    }

    /// Takes a single element from the queue, and releases it.
    string qtake()
    {
        ByteBuffer *b = (ByteBuffer *)q.next(0);
        HASSERT(b != nullptr);
        string ret((char *)b->data()->data_, b->data()->size());
        b->unref();
        return ret;
    }
};

class StreamTestBase : public TwoNodeDatagramTest
{
protected:
    StreamTestBase()
    {
        mainBufferPool->alloc(&recvRequest_);
    }

    BufferPtr<StreamReceiveRequest> recvRequest_;
    CollectData sink_;
};

} // namespace openlcb

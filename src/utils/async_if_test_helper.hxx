// Helper classes for writing unittests testing the entire asynchronous
// stack. Allows to send incoming messages (in gridconnect format) and set
// expectations on messages produced.
//
// Only include this file in unittests.

#ifndef _UTILS_ASYNC_IF_TEST_HELPER_HXX_
#define _UTILS_ASYNC_IF_TEST_HELPER_HXX_

#include "nmranet/AliasAllocator.hxx"
#include "nmranet/IfCan.hxx"
#include "nmranet/EventService.hxx"
#include "nmranet/DefaultNode.hxx"
#include "nmranet_config.h"
#include "utils/GridConnectHub.hxx"
#include "utils/test_main.hxx"

using ::testing::AtLeast;
using ::testing::AtMost;
using ::testing::Eq;
using ::testing::Field;
using ::testing::Invoke;
using ::testing::IsNull;
using ::testing::Mock;
using ::testing::NiceMock;
using ::testing::NotNull;
using ::testing::Pointee;
using ::testing::Return;
using ::testing::StrCaseEq;
using ::testing::StrictMock;
using ::testing::WithArg;
using ::testing::_;

///@todo(balazs.racz) remove
// void (*g_invoke)(Notifiable *) = &InvokeNotification;

HubFlow gc_hub0(&g_service);
CanHubFlow can_hub0(&g_service);
GCAdapterBase *g_gc_adapter = nullptr;

HubFlow gc_hub1(&g_service);
CanHubFlow can_hub1(&g_service);
GCAdapterBase *g_gc_adapter1 = nullptr;

/** Helper class for setting expectation on the CANbus traffic in unit
 * tests. */
class MockSend : public HubPort
{
public:
    MockSend()
        : HubPort(&g_service)
    {
    }

    MOCK_METHOD1(mwrite, void(const string &s));

    virtual Action entry()
    {
        string s(message()->data()->data(), message()->data()->size());
        mwrite(s);
        return release_and_exit();
    }
};

void InvokeNotification(Notifiable *done)
{
    done->notify();
}

static void print_packet(const string &pkt)
{
    fprintf(stderr, "%s\n", pkt.c_str());
}

class AsyncCanTest : public testing::Test
{
public:
    static void SetUpTestCase()
    {
        g_gc_adapter =
            GCAdapterBase::CreateGridConnectAdapter(&gc_hub0, &can_hub0, false);
    }

    static void TearDownTestCase()
    {
        delete g_gc_adapter;
    }

protected:
    AsyncCanTest()
    {
        gc_hub0.register_port(&canBus_);
    }

    ~AsyncCanTest()
    {
        gc_hub0.unregister_port(&canBus_);
        if (printer_.get())
        {
            gc_hub0.unregister_port(printer_.get());
        }
    }

    /** Delays the current thread until we are certain that all asynchrnous
        processing has completed. */
    void wait()
    {
        wait_for_main_executor();
    }

/** Adds an expectation that the code will send a packet to the CANbus.

    Example:
    expect_packet(":X1954412DN05010101FFFF0000;");

    @param gc_packet the packet in GridConnect format, including the leading
    : and trailing ;
*/
#define expect_packet(gc_packet)                                               \
    EXPECT_CALL(canBus_, mwrite(StrCaseEq(gc_packet)))

    /** Ignores all produced packets.
     *
     *  Tihs can be used in tests where the expectations are tested in a higher
     *  level than monitoring the CANbus traffic.
    */
    void expect_any_packet()
    {
        EXPECT_CALL(canBus_, mwrite(_)).Times(AtLeast(0)).WillRepeatedly(
            WithArg<0>(Invoke(print_packet)));
    }

    /** Prints all packets sent to the canbus until the end of the current test
     * function.
    */
    void print_all_packets()
    {
        NiceMock<MockSend> *m = new NiceMock<MockSend>();
        EXPECT_CALL(*m, mwrite(_)).Times(AtLeast(0)).WillRepeatedly(
            WithArg<0>(Invoke(print_packet)));
        gc_hub0.register_port(m);
        printer_.reset(m);
    }

    /** Injects a packet to the interface. This acts as if a different node on
        the CANbus had sent that packet.

        Example:
        send_packet(":X195B4001N05010101FFFF0000;");

        @param gc_packet the packet in GridConnect format, including the leading
        : and trailing ;
    */
    void send_packet(const string &gc_packet)
    {
        Buffer<HubData> *packet;
        mainBufferPool->alloc(&packet);
        packet->data()->assign(gc_packet);
        packet->data()->skipMember_ = &canBus_;
        gc_hub0.send(packet);
    }

/** Injects an incoming packet to the interface and expects that the node
    will send out a response packet for it.

    As a side effect, clears all pending expectations on the CANbus.

    Example:
    send_packet_and_expect_response(":X198F4001N05010101FFFF0000;",
                                    ":X194C412DN05010101FFFF0000;");

    @param pkt is the packet to inject, in GridConnect format.
    @param resp is the response to expect, also in GridConnect format.
*/
#define send_packet_and_expect_response(pkt, resp)                             \
    do                                                                         \
    {                                                                          \
        expect_packet(resp);                                                   \
        send_packet_and_flush_expect(pkt);                                     \
    } while (0)

    void send_packet_and_flush_expect(const string &pkt)
    {
        send_packet(pkt);
        wait();
        Mock::VerifyAndClear(&canBus_);
    }

    /// Helper object for setting expectations on the packets sent on the bus.
    NiceMock<MockSend> canBus_;
    /// Object for debug-printing every packet (if requested).
    std::unique_ptr<HubPort> printer_;
};

namespace nmranet
{

/*
const char *Node::MANUFACTURER = "Stuart W. Baker";
const char *Node::HARDWARE_REV = "N/A";
const char *Node::SOFTWARE_REV = "0.1";

const size_t Datagram::POOL_SIZE = 10;
const size_t Datagram::THREAD_STACK_SIZE = 512;
const size_t Stream::CHANNELS_PER_NODE = 10;
const uint16_t Stream::MAX_BUFFER_SIZE = 512;*/

static const NodeID TEST_NODE_ID = 0x02010d000003ULL;

/** Test fixture base class with helper methods for exercising the asynchronous
 * interface code.
 *
 * Usage:
 *
 * Inherit your test fixture class from AsyncIfTest.
 */
class AsyncIfTest : public AsyncCanTest
{
protected:
    AsyncIfTest()
        : pendingAliasAllocation_(false)
    {
        ifCan_.reset(new IfCan(&g_executor, &can_hub0, 10, 10, 5));
        ifCan_->local_aliases()->add(TEST_NODE_ID, 0x22A);
    }

    ~AsyncIfTest()
    {
        wait();
        if (pendingAliasAllocation_)
        {
            ifCan_->alias_allocator()->TEST_finish_pending_allocation();
            wait();
        }
    }

    /** Creates an alias allocator flow, and injects an already allocated
     *  alias. */
    void create_allocated_alias()
    {
        ifCan_->set_alias_allocator(
            new AliasAllocator(TEST_NODE_ID, ifCan_.get()));
        Buffer<AliasInfo> *a;
        mainBufferPool->alloc(&a);
        a->data()->alias = 0x33A;
        a->data()->state = AliasInfo::STATE_RESERVED;
        ifCan_->local_aliases()->add(AliasCache::RESERVED_ALIAS_NODE_ID,
                                     a->data()->alias);
        ifCan_->alias_allocator()->reserved_aliases()->insert(a);
        aliasSeed_ = 0x44C;
        pendingAliasAllocation_ = false;
    }

    void expect_next_alias_allocation(NodeAlias a = 0)
    {
        pendingAliasAllocation_ = true;
        if (!a)
        {
            ifCan_->alias_allocator()->seed_ = aliasSeed_;
            a = aliasSeed_;
            aliasSeed_++;
        }
        EXPECT_CALL(canBus_, mwrite(StringPrintf(":X17020%03XN;", a)))
            .Times(1)
            .RetiresOnSaturation();
        EXPECT_CALL(canBus_, mwrite(StringPrintf(":X1610D%03XN;", a)))
            .Times(1)
            .RetiresOnSaturation();
        EXPECT_CALL(canBus_, mwrite(StringPrintf(":X15000%03XN;", a)))
            .Times(1)
            .RetiresOnSaturation();
        EXPECT_CALL(canBus_, mwrite(StringPrintf(":X14003%03XN;", a)))
            .Times(1)
            .RetiresOnSaturation();

        EXPECT_CALL(canBus_, mwrite(StringPrintf(":X10700%03XN;", a)))
            .Times(AtMost(1))
            .RetiresOnSaturation();
    }


    BarrierNotifiable *get_notifiable()
    {
        bn_.reset(&n_);
        return &bn_;
    }

    void wait_for_notification()
    {
        n_.wait_for_notification();
    }

    SyncNotifiable n_;
    BarrierNotifiable bn_;

    /// The interface under test.
    std::unique_ptr<IfCan> ifCan_;
    /** Temporary object used to send aliases around in the alias allocator
     *  flow. */
    AliasInfo testAlias_;
    /// The next alias we will make the allocator create.
    NodeAlias aliasSeed_;
    /// true if we have a pending async alias allocation task.
    bool pendingAliasAllocation_;
};

class AsyncNodeTest : public AsyncIfTest
{
protected:
    AsyncNodeTest()
        : eventService_(ifCan_.get())
    {
        EXPECT_CALL(canBus_, mwrite(":X1910022AN02010D000003;")).Times(1);
        ownedNode_.reset(new DefaultNode(ifCan_.get(), TEST_NODE_ID));
        node_ = ownedNode_.get();
        ifCan_->add_addressed_message_support();
        wait();
        // AddEventHandlerToIf(ifCan_.get());
    }

    ~AsyncNodeTest()
    {
        wait_for_event_thread();
    }

    void wait_for_event_thread()
    {
        while (EventService::instance->event_processing_pending())
        {
            usleep(100);
        }
        AsyncIfTest::wait();
    }

    EventService eventService_;
    std::unique_ptr<DefaultNode> ownedNode_;
    Node *node_;
};

class MockMessageHandler : public MessageHandler
{
public:
    MOCK_METHOD2(handle_message,
                 void(NMRAnetMessage *message, unsigned priority));
    virtual void send(Buffer<NMRAnetMessage> *message, unsigned priority)
    {
        handle_message(message->data(), priority);
        message->unref();
    }
};

MATCHER_P(IsBufferValue, id, "")
{
    uint64_t value = htobe64(id);
    if (arg.size() != 8)
        return false;
    if (memcmp(&value, arg.data(), 8))
        return false;
    return true;
}

MATCHER_P(IsBufferValueString, expected, "")
{
    string s(expected);
    return arg == s;
}

MATCHER_P(IsBufferNodeValue, id, "")
{
    uint64_t value = htobe64(id);
    if (arg->used() != 6)
        return false;
    uint8_t *expected = reinterpret_cast<uint8_t *>(&value) + 2;
    uint8_t *actual = static_cast<uint8_t *>(arg->start());
    if (memcmp(expected, actual, 6))
    {
        for (int i = 0; i < 6; ++i)
        {
            if (expected[i] != actual[i])
            {
                LOG(INFO, "mismatch at position %d, expected %02x actual %02x",
                    i, expected[i], actual[i]);
            }
        }
        return false;
    }
    return true;
}

MATCHER_P(IsBufferNodeValueString, id, "")
{
    uint64_t value = htobe64(id);
    if (arg.size() != 6)
        return false;
    uint8_t *expected = reinterpret_cast<uint8_t *>(&value) + 2;
    uint8_t *actual = static_cast<uint8_t *>(arg->start());
    if (memcmp(expected, actual, 6))
    {
        for (int i = 0; i < 6; ++i)
        {
            if (expected[i] != actual[i])
            {
                LOG(INFO, "mismatch at position %d, expected %02x actual %02x",
                    i, expected[i], actual[i]);
            }
        }
        return false;
    }
    return true;
}

} // namespace nmranet

#endif // _UTILS_ASYNC_IF_TEST_HELPER_HXX_

// Helper classes for writing unittests testing the entire asynchronous
// stack. Allows to send incoming messages (in gridconnect format) and set
// expectations on messages produced.
//
// Only include this file in unittests.

#ifndef _UTILS_ASYNC_IF_TEST_HELPER_HXX_
#define _UTILS_ASYNC_IF_TEST_HELPER_HXX_

#include "nmranet/AsyncAliasAllocator.hxx"
#include "nmranet/AsyncIfCan.hxx"
#include "nmranet/GlobalEventHandler.hxx"
#include "nmranet/NMRAnetAsyncDefaultNode.hxx"
#include "nmranet/NMRAnetAsyncEventHandler.hxx"
#include "nmranet/NMRAnetWriteFlow.hxx"
#include "nmranet_config.h"
#include "utils/gc_pipe.hxx"
#include "utils/pipe.hxx"
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

void (*g_invoke)(Notifiable*) = &InvokeNotification;

HubFlow gc_hub0(&g_service);
CanHubFlow can_hub0(&g_service);
GCAdapterBase* g_gc_adapter = nullptr;

/** Helper class for setting expectation on the CANbus traffic in unit
 * tests. */
class MockSend : public HubPort {
 public:
    MockPipeMember() : HubPort(&g_service) {}

    MOCK_METHOD1(mwrite, void(const string& s));

    virtual Action entry() {
        string s(message()->data()->data(), message()->data()->size());
        mwrite(s);
        return release_and_exit();
    }
};



namespace NMRAnet
{

const char* Node::MANUFACTURER = "Stuart W. Baker";
const char* Node::HARDWARE_REV = "N/A";
const char* Node::SOFTWARE_REV = "0.1";

const size_t Datagram::POOL_SIZE = 10;
const size_t Datagram::THREAD_STACK_SIZE = 512;
const size_t Stream::CHANNELS_PER_NODE = 10;
const uint16_t Stream::MAX_BUFFER_SIZE = 512;

static const NodeID TEST_NODE_ID = 0x02010d000003ULL;

static void PrintPacket(const string& pkt)
{
    fprintf(stderr, "%s\n", pkt.c_str());
}

/** Test fixture base class with helper methods for exercising the asynchronous
 * interface code.
 *
 * Usage:
 *
 * Inherit your test fixture class from AsyncIfTest.
 */
class AsyncIfTest : public testing::Test
{
public:
    static void SetUpTestCase()
    {
        g_gc_adapter = GCAdapterBase::CreateGridConnectAdapter(
            &gc_hub0, &can_hub0, false);
    }

    static void TearDownTestCase()
    {
        delete g_gc_adapter;
    }

protected:
    AsyncIfTest()
    {
        gc_pipe0.register_port(&canBus_);
        ifCan_.reset(new AsyncIfCan(&g_executor, &can_pipe0, 10, 10, 1, 1, 5));
        ifCan_->local_aliases()->add(TEST_NODE_ID, 0x22A);
    }

    ~AsyncIfTest()
    {
        Wait();
        gc_pipe0.UnregisterMember(&canBus_);
        if (printer_.get()) {
            gc_pipe0.UnregisterMember(printer_.get());
        }
    }

    /** Creates an alias allocator flow, and injects an already allocated
     *  alias. */
    void CreateAllocatedAlias()
    {
        ifCan_->set_alias_allocator(
            new AsyncAliasAllocator(TEST_NODE_ID, ifCan_.get()));
        testAlias_.alias = 0x33A;
        testAlias_.state = AliasInfo::STATE_RESERVED;
        ifCan_->local_aliases()->add(AliasCache::RESERVED_ALIAS_NODE_ID,
                                      testAlias_.alias);
        ifCan_->alias_allocator()->reserved_aliases()->Release(&testAlias_);
        aliasSeed_ = 0x44C;
    }

    void ExpectNextAliasAllocation(NodeAlias a = 0)
    {
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

/** Adds an expectation that the code will send a packet to the CANbus.

    Example:
    ExpectPacket(":X1954412DN05010101FFFF0000;");

    @param gc_packet the packet in GridConnect format, including the leading
    : and trailing ;
*/
#define ExpectPacket(gc_packet)                                                \
    EXPECT_CALL(canBus_, mwrite(StrCaseEq(gc_packet)))

    /** Ignores all produced packets.
     *
     *  Tihs can be used in tests where the expectations are tested in a higher
     *  level than monitoring the CANbus traffic.
    */
    void ExpectAnyPacket()
    {
        EXPECT_CALL(canBus_, mwrite(_)).Times(AtLeast(0)).WillRepeatedly(
            WithArg<0>(Invoke(PrintPacket)));
    }

    /** Prints all packets sent to the canbus until the end of the current test
     * function.
    */
    void PrintAllPackets()
    {
        NiceMock<MockSend>* m = new NiceMock<MockSend>();
        EXPECT_CALL(*m, mwrite(_)).Times(AtLeast(0)).WillRepeatedly(
            WithArg<0>(Invoke(PrintPacket)));
        gc_pipe0.RegisterMember(m);
        printer_.reset(m);
    }

    /** Injects a packet to the interface. This acts as if a different node on
        the CANbus had sent that packet.

        Example:
        SendPacket(":X195B4001N05010101FFFF0000;");

        @param gc_packet the packet in GridConnect format, including the leading
        : and trailing ;
    */
    void SendPacket(const string& gc_packet)
    {
        gc_pipe0.WriteToAll(&canBus_, gc_packet.data(), gc_packet.size());
    }

    /** Delays the current thread until we are certain that all asynchrnous
        processing has completed. */
    void Wait()
    {
        do
        {
            bool exit = true;
            {
                LockHolder h1(&g_executor);
                LockHolder h2(DefaultWriteFlowExecutor());
                LockHolder h3(&g_gc_pipe_executor);

                if (!g_executor.empty() || !g_gc_pipe_executor.empty() ||
                    !DefaultWriteFlowExecutor()->empty() ||
                    !ifCan_->frame_dispatcher()->IsNotStarted() ||
                    !ifCan_->dispatcher()->IsNotStarted() ||
                    !can_pipe0.empty() || !gc_pipe0.empty())
                {
                    exit = false;
                }
            }
            if (exit)
            {
                return;
            }
            else
            {
                usleep(100);
            }
        } while (true);
    }

    /** Injects an incoming packet to the interface and expects that the node
        will send out a response packet for it.

        As a side effect, clears all pending expectations on the CANbus.

        Example:
        SendPacketAndExpectResponse(":X198F4001N05010101FFFF0000;",
                                    ":X194C412DN05010101FFFF0000;");

        @param pkt is the packet to inject, in GridConnect format.
        @param resp is the response to expect, also in GridConnect format.
    */
    void SendPacketAndExpectResponse(const string& pkt, const string& resp)
    {
        ExpectPacket(resp);
        SendPacket(pkt);
        Wait();
        Mock::VerifyAndClear(&canBus_);
    }

    //! Helper object for setting expectations on the packets sent on the bus.
    NiceMock<MockSend> canBus_;
    //! Object for debug-printing every packet (if requested).
    std::unique_ptr<PipeMember> printer_;
    //! The interface under test.
    std::unique_ptr<AsyncIfCan> ifCan_;
    /** Temporary object used to send aliases around in the alias allocator
     *  flow. */
    AliasInfo testAlias_;
    //! The next alias we will make the allocator create.
    NodeAlias aliasSeed_;
};

class AsyncNodeTest : public AsyncIfTest
{
protected:
    AsyncNodeTest() : eventFlow_(&g_executor, 10)
    {
        EXPECT_CALL(canBus_, mwrite(":X1910022AN02010D000003;")).Times(1);
        ownedNode_.reset(new DefaultAsyncNode(ifCan_.get(), TEST_NODE_ID));
        node_ = ownedNode_.get();
        ifCan_->add_addressed_message_support(2);
        Wait();
        AddEventHandlerToIf(ifCan_.get());
    }

    ~AsyncNodeTest()
    {
        WaitForEventThread();
    }

    void WaitForEventThread()
    {
        while (GlobalEventFlow::instance->EventProcessingPending())
        {
            usleep(100);
        }
        AsyncIfTest::Wait();
    }

    GlobalEventFlow eventFlow_;
    std::unique_ptr<DefaultAsyncNode> ownedNode_;
    AsyncNode* node_;
};

class MockMessageHandler : public IncomingMessageHandler
{
public:
    MOCK_METHOD0(get_allocator, AllocatorBase*());
    MOCK_METHOD2(handle_message,
                 void(IncomingMessage* message, Notifiable* done));
};

MATCHER_P(IsBufferValue, id, "")
{
    uint64_t value = htobe64(id);
    if (arg->used() != 8)
        return false;
    if (memcmp(&value, arg->start(), 8))
        return false;
    return true;
}

MATCHER_P(IsBufferValueString, expected, "")
{
    string s(expected);
    if (arg->used() != s.size())
        return false;
    if (memcmp(s.data(), arg->start(), arg->used()))
        return false;
    return true;
}

MATCHER_P(IsBufferNodeValue, id, "")
{
    uint64_t value = htobe64(id);
    if (arg->used() != 6)
        return false;
    uint8_t* expected = reinterpret_cast<uint8_t*>(&value) + 2;
    uint8_t* actual = static_cast<uint8_t*>(arg->start());
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

} // namespace NMRAnet

#endif // _UTILS_ASYNC_IF_TEST_HELPER_HXX_

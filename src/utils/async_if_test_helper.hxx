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

static void InvokeNotification(Notifiable* done)
{
    done->Notify();
}

void (*g_invoke)(Notifiable*) = &InvokeNotification;

DEFINE_PIPE(gc_pipe0, 1);
GCAdapterBase* g_gc_adapter = nullptr;
ThreadExecutor g_executor("async_exec", 0, 2000);

/** Helper class for setting expectation on the CANbus traffic in unit
 * tests. */
class MockSend : public PipeMember
{
public:
    virtual void write(const void* buf, size_t count)
    {
        string s(static_cast<const char*>(buf), count);
        MWrite(s);
    }

    MOCK_METHOD1(MWrite, void(const string& s));
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
            &gc_pipe0, &can_pipe0, false);
    }

    static void TearDownTestCase()
    {
        delete g_gc_adapter;
    }

protected:
    AsyncIfTest()
    {
        gc_pipe0.RegisterMember(&can_bus_);
        if_can_.reset(new AsyncIfCan(&g_executor, &can_pipe0, 10, 10));
        if_can_->local_aliases()->add(TEST_NODE_ID, 0x22A);
    }

    ~AsyncIfTest()
    {
        Wait();
        gc_pipe0.UnregisterMember(&can_bus_);
    }

    /** Creates an alias allocator flow, and injects an already allocated
     *  alias. */
    void CreateAllocatedAlias()
    {
        if_can_->set_alias_allocator(
            new AsyncAliasAllocator(TEST_NODE_ID, if_can_.get()));
        testAlias_.alias = 0x33A;
        testAlias_.state = AliasInfo::STATE_RESERVED;
        if_can_->local_aliases()->add(AliasCache::RESERVED_ALIAS_NODE_ID,
                                      testAlias_.alias);
        if_can_->alias_allocator()->reserved_aliases()->Release(&testAlias_);
        aliasSeed_ = 0x44C;
    }

    void ExpectNextAliasAllocation(NodeAlias a = 0)
    {
        if (!a)
        {
            if_can_->alias_allocator()->seed_ = aliasSeed_;
            a = aliasSeed_;
            aliasSeed_++;
        }
        EXPECT_CALL(can_bus_, MWrite(StringPrintf(":X17020%03XN;", a)))
            .Times(1)
            .RetiresOnSaturation();
        EXPECT_CALL(can_bus_, MWrite(StringPrintf(":X1610D%03XN;", a)))
            .Times(1)
            .RetiresOnSaturation();
        EXPECT_CALL(can_bus_, MWrite(StringPrintf(":X15000%03XN;", a)))
            .Times(1)
            .RetiresOnSaturation();
        EXPECT_CALL(can_bus_, MWrite(StringPrintf(":X14003%03XN;", a)))
            .Times(1)
            .RetiresOnSaturation();

        EXPECT_CALL(can_bus_, MWrite(StringPrintf(":X10700%03XN;", a)))
            .Times(AtMost(1))
            .RetiresOnSaturation();
    }

    /** Adds an expectation that the code will send a packet to the CANbus.

        Example:
        ExpectPacket(":X1954412DN05010101FFFF0000;");

        @param gc_packet the packet in GridConnect format, including the leading
        : and trailing ;
    */
    void ExpectPacket(const string& gc_packet)
    {
        EXPECT_CALL(can_bus_, MWrite(StrCaseEq(gc_packet)));
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
        gc_pipe0.WriteToAll(&can_bus_, gc_packet.data(), gc_packet.size());
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

                if (!g_executor.empty() ||
                    !DefaultWriteFlowExecutor()->empty() ||
                    !if_can_->frame_dispatcher()->IsNotStarted() ||
                    !if_can_->dispatcher()->IsNotStarted())
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
        Mock::VerifyAndClear(&can_bus_);
    }

    //! Helper object for setting expectations on the packets sent on the bus.
    NiceMock<MockSend> can_bus_;
    //! The interface under test.
    std::unique_ptr<AsyncIfCan> if_can_;
    /** Temporary object used to send aliases around in the alias allocator
     *  flow. */
    AliasInfo testAlias_;
    //! The next alias we will make the allocator create.
    NodeAlias aliasSeed_;
};

class AsyncNodeTest : public AsyncIfTest
{
protected:
    AsyncNodeTest()
        : eventFlow_(&g_executor, 10),
          ownedNode_(if_can_.get(), TEST_NODE_ID),
          node_(&ownedNode_)
    {
        EXPECT_CALL(can_bus_, MWrite(":X1910022AN02010D000003;")).Times(1);
        if_can_->AddWriteFlows(2, 2);
        Wait();
        AddEventHandlerToIf(if_can_.get());
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
    DefaultAsyncNode ownedNode_;
    AsyncNode* node_;
};

} // namespace NMRAnet

#endif // _UTILS_ASYNC_IF_TEST_HELPER_HXX_

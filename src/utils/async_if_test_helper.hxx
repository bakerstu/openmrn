// Helper classes for writing unittests testing the entire asynchronous
// stack. Allows to send incoming messages (in gridconnect format) and set
// expectations on messages produced.
//
// Only include this file in unittests.

#ifndef _UTILS_ASYNC_IF_TEST_HELPER_HXX_
#define _UTILS_ASYNC_IF_TEST_HELPER_HXX_

#include "nmranet/AsyncIfCan.hxx"
#include "nmranet_config.h"
#include "utils/gc_pipe.hxx"
#include "utils/pipe.hxx"
#include "utils/test_main.hxx"

using ::testing::Invoke;
using ::testing::Mock;
using ::testing::NiceMock;
using ::testing::Return;
using ::testing::StrCaseEq;
using ::testing::StrictMock;
using ::testing::WithArg;
using ::testing::_;

static void InvokeNotification(Notifiable* done)
{
    done->Notify();
}

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
        if_can_.reset(new AsyncIfCan(&g_executor, &can_pipe0));
    }

    ~AsyncIfTest()
    {
        Wait();
        gc_pipe0.UnregisterMember(&can_bus_);
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
        while (!g_executor.empty() ||
               !if_can_->frame_dispatcher()->IsNotStarted() ||
               !if_can_->dispatcher()->IsNotStarted())
        {
            usleep(100);
        }
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
};

} // namespace NMRAnet

#endif // _UTILS_ASYNC_IF_TEST_HELPER_HXX_

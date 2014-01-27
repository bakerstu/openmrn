#include "utils/async_if_test_helper.hxx"

#include "nmranet/NMRAnetAsyncDatagram.hxx"
#include "nmranet/NMRAnetAsyncDatagramCan.hxx"

namespace NMRAnet
{

class AsyncDatagramTest : public AsyncNodeTest
{
protected:
    AsyncDatagramTest() : parser_(if_can_.get())
    {
    }

    CanDatagramParser parser_;
};

class AsyncRawDatagramTest : public AsyncDatagramTest
{
protected:
    AsyncRawDatagramTest()
    {
        if_can_->dispatcher()->RegisterHandler(0x1C48, 0xFFFF, &handler_);
        EXPECT_CALL(handler_, get_allocator()).WillRepeatedly(Return(nullptr));
        ON_CALL(handler_, handle_message(_, _))
            .WillByDefault(WithArg<1>(Invoke(&InvokeNotification)));
    }
    ~AsyncRawDatagramTest()
    {
        Wait();
        if_can_->dispatcher()->UnregisterHandler(0x1C48, 0xFFFF, &handler_);
    }

    StrictMock<MockMessageHandler> handler_;
};

TEST_F(AsyncRawDatagramTest, CreateDestroy)
{
}

TEST_F(AsyncRawDatagramTest, SingleFrameDatagramArrivesWrongTarget)
{
    SendPacket(":X1A333555NFF01020304050607;");
}

TEST_F(AsyncRawDatagramTest, MultiFrameDatagramArrivesWrongTarget)
{
    SendPacket(":X1B333555NFF01020304050607;");
    SendPacket(":X1C333555NFF01020304050607;");
    SendPacket(":X1C333555NFF01020304050607;");
    SendPacket(":X1D333555NFF01020304050607;");
}

TEST_F(AsyncRawDatagramTest, SingleFrameDatagramArrivesRightTarget)
{
    EXPECT_CALL(
        handler_,
        handle_message(
            Pointee(AllOf(Field(&IncomingMessage::mti, If::MTI_DATAGRAM),
                          Field(&IncomingMessage::dst_node, node_),
                          Field(&IncomingMessage::payload, NotNull()),
                          Field(&IncomingMessage::payload,
                                IsBufferValue(0xFF01020304050607ULL)) //,
                          )),
            _)).WillOnce(WithArg<1>(Invoke(&InvokeNotification)));
    SendPacket(":X1A22A555NFF01020304050607;");
}

TEST_F(AsyncRawDatagramTest, MultiFrameDatagramArrivesRightTarget)
{
    EXPECT_CALL(
        handler_,
        handle_message(
            Pointee(AllOf(Field(&IncomingMessage::mti, If::MTI_DATAGRAM),
                          Field(&IncomingMessage::dst_node, node_),
                          Field(&IncomingMessage::payload, NotNull()),
                          Field(&IncomingMessage::payload,
                                IsBufferValueString(
                                    "01234567112345672123456731234567")) //,
                          )),
            _)).WillOnce(WithArg<1>(Invoke(&InvokeNotification)));
    SendPacket(":X1B22A555N3031323334353637;");
    SendPacket(":X1C22A555N3131323334353637;");
    SendPacket(":X1C22A555N3231323334353637;");
    SendPacket(":X1D22A555N3331323334353637;");
}

TEST_F(AsyncRawDatagramTest, MultiFrameIntermixed)
{
    SendPacket(":X1B22A555N3031323334353637;");
    SendPacket(":X1C22A555N3131323334353637;");
    SendPacket(":X1C22A555N3231323334353637;");
    EXPECT_CALL(
        handler_,
        handle_message(
            Pointee(AllOf(
                Field(&IncomingMessage::mti, If::MTI_DATAGRAM),
                Field(&IncomingMessage::dst_node, node_),
                Field(&IncomingMessage::src, Field(&NodeHandle::alias, 0x577)),
                Field(&IncomingMessage::payload, NotNull()),
                Field(&IncomingMessage::payload,
                      IsBufferValueString("0123456711234567")) //,
                )),
            _)).WillOnce(WithArg<1>(Invoke(&InvokeNotification)));
    SendPacket(":X1B22A577N3031323334353637;");
    EXPECT_CALL(
        handler_,
        handle_message(
            Pointee(AllOf(
                Field(&IncomingMessage::mti, If::MTI_DATAGRAM),
                Field(&IncomingMessage::dst_node, node_),
                Field(&IncomingMessage::src, Field(&NodeHandle::alias, 0x555)),
                Field(&IncomingMessage::payload, NotNull()),
                Field(
                    &IncomingMessage::payload,
                    IsBufferValueString("01234567112345672123456731234567")) //,
                )),
            _)).WillOnce(WithArg<1>(Invoke(&InvokeNotification)));
    SendPacket(":X1D22A555N3331323334353637;");
    SendPacket(":X1D22A577N3131323334353637;");
}

TEST_F(AsyncRawDatagramTest, MultiFrameIntermixedDst)
{
    EXPECT_CALL(can_bus_, MWrite(":X1910022BN02010D000004;")).Times(1);
    DefaultAsyncNode other_node(if_can_.get(), TEST_NODE_ID + 1);
    if_can_->local_aliases()->add(TEST_NODE_ID + 1, 0x22B);

    SendPacket(":X1B22A555N3031323334353637;");
    SendPacket(":X1C22A555N3131323334353637;");
    SendPacket(":X1C22A555N3231323334353637;");
    EXPECT_CALL(
        handler_,
        handle_message(
            Pointee(AllOf(
                Field(&IncomingMessage::mti, If::MTI_DATAGRAM),
                Field(&IncomingMessage::dst_node, &other_node),
                Field(&IncomingMessage::src, Field(&NodeHandle::alias, 0x555)),
                Field(&IncomingMessage::payload, NotNull()),
                Field(&IncomingMessage::payload,
                      IsBufferValueString("0123456711234567")) //,
                )),
            _)).WillOnce(WithArg<1>(Invoke(&InvokeNotification)));
    SendPacket(":X1B22B555N3031323334353637;");
    EXPECT_CALL(
        handler_,
        handle_message(
            Pointee(AllOf(
                Field(&IncomingMessage::mti, If::MTI_DATAGRAM),
                Field(&IncomingMessage::dst_node, node_),
                Field(&IncomingMessage::src, Field(&NodeHandle::alias, 0x555)),
                Field(&IncomingMessage::payload, NotNull()),
                Field(
                    &IncomingMessage::payload,
                    IsBufferValueString("01234567112345672123456731234567")) //,
                )),
            _)).WillOnce(WithArg<1>(Invoke(&InvokeNotification)));
    SendPacket(":X1D22A555N3331323334353637;");
    SendPacket(":X1D22B555N3131323334353637;");
}

class MockDatagramHandlerBase : public DatagramHandler, public ControlFlow
{
public:
    MockDatagramHandlerBase() : ControlFlow(&g_executor, nullptr)
    {
        StartFlowAt(ST(wait_for_datagram));
    }

    virtual void handle_datagram(IncomingDatagram* d) = 0;

    ControlFlowAction wait_for_datagram()
    {
        return Allocate(&queue_, ST(process_datagram));
    }

    ControlFlowAction process_datagram()
    {
        IncomingDatagram* d = GetTypedAllocationResult(&queue_);
        handle_datagram(d);
        d->free();
        return CallImmediately(ST(wait_for_datagram));
    }
};

class MockDatagramHandler : public MockDatagramHandlerBase
{
public:
    MOCK_METHOD1(handle_datagram, void(IncomingDatagram* d));
};

TEST_F(AsyncDatagramTest, DispatchTest)
{
    StrictMock<MockDatagramHandler> dg;
    DatagramDispatcher dispatcher(if_can_.get(), 10);
    dispatcher.registry()->insert(nullptr, 0x30, &dg);
    EXPECT_CALL(
        dg, handle_datagram(Pointee(AllOf(
                Field(&IncomingDatagram::src, Field(&NodeHandle::alias, 0x555)),
                Field(&IncomingDatagram::dst, node_),
                Field(&IncomingDatagram::payload, NotNull()),
                Field(&IncomingDatagram::payload,
                      IsBufferValueString("01234567")) //,
                ))));
    SendPacket(":X1A22A555N3031323334353637;");
    Wait();
    usleep(3000);
}

InitializedAllocator<IncomingDatagram> g_incoming_datagram_allocator(10);

} // namespace NMRAnet

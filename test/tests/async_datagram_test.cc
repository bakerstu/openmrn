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

} // namespace NMRAnet

#include "utils/async_if_test_helper.hxx"

namespace NMRAnet
{

class MockCanFrameHandler : public IncomingFrameHandler
{
public:
    MOCK_METHOD2(HandleMessage,
                 TypedAllocator<ParamHandler<struct can_frame>>*(
                     struct can_frame* message, Notifiable* done));
};

MATCHER_P(IsExtCanFrameWithId, id, "")
{
    if (!IS_CAN_FRAME_EFF(*arg)) return false;
    return ((uint32_t) id) == GET_CAN_FRAME_ID_EFF(*arg);
}

TEST_F(AsyncIfTest, Setup)
{
}

TEST_F(AsyncIfTest, InjectFrame)
{
    SendPacket(":X195B432DN05010103;");
    Wait();
}

TEST_F(AsyncIfTest, InjectFrameAndExpectHandler)
{
    StrictMock<MockCanFrameHandler> h;
    if_can_->frame_dispatcher()->RegisterHandler(0x195B4000, 0x1FFFF000, &h);
    EXPECT_CALL(h, HandleMessage(IsExtCanFrameWithId(0x195B432D), _)).WillOnce(
        DoAll(WithArg<1>(Invoke(&InvokeNotification)), Return(nullptr)));

    SendPacket(":X195B432DN05010103;");
    Wait();

    SendPacket(":X195F432DN05010103;");
    SendPacket(":X195F432DN05010103;");

    Wait();
    EXPECT_CALL(h, HandleMessage(IsExtCanFrameWithId(0x195B4777), _)).WillOnce(
        DoAll(WithArg<1>(Invoke(&InvokeNotification)), Return(nullptr)));
    EXPECT_CALL(h, HandleMessage(IsExtCanFrameWithId(0x195B4222), _)).WillOnce(
        DoAll(WithArg<1>(Invoke(&InvokeNotification)), Return(nullptr)));
    SendPacket(":X195B4777N05010103;");
    SendPacket(":X195F4333N05010103;");
    SendPacket(":X195B4222N05010103;");
    Wait();
    if_can_->frame_dispatcher()->UnregisterHandler(0x195B4000, 0x1FFFF000, &h);
}

TEST_F(AsyncIfTest, WriteFrame)
{
    ExpectPacket(":X195B432DNAA;");
    TypedSyncAllocation<CanFrameWriteFlow> w(if_can_->write_allocator());
    struct can_frame* f = w.result()->mutable_frame();
    SET_CAN_FRAME_EFF(*f);
    SET_CAN_FRAME_ID_EFF(*f, 0x195B432D);
    f->can_dlc = 1;
    f->data[0] = 0xaa;
    w.result()->Send(nullptr);
}


} // namespace NMRAnet

#include "utils/async_if_test_helper.hxx"

#include "nmranet/NMRAnetWriteFlow.hxx"

using ::testing::Field;
using ::testing::Pointee;
using ::testing::NotNull;

namespace NMRAnet
{

class MockCanFrameHandler : public IncomingFrameHandler
{
public:
    MOCK_METHOD0(get_allocator, AllocatorBase*());
    MOCK_METHOD2(handle_message,
                 void(struct can_frame* message, Notifiable* done));
};

MATCHER_P(IsExtCanFrameWithId, id, "")
{
    if (!IS_CAN_FRAME_EFF(*arg)) return false;
    return ((uint32_t)id) == GET_CAN_FRAME_ID_EFF(*arg);
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
    EXPECT_CALL(h, get_allocator()).WillRepeatedly(Return(nullptr));
    EXPECT_CALL(h, handle_message(IsExtCanFrameWithId(0x195B432D), _)).WillOnce(
        WithArg<1>(Invoke(&InvokeNotification)));

    SendPacket(":X195B432DN05010103;");
    Wait();

    SendPacket(":X195F432DN05010103;");
    SendPacket(":X195F432DN05010103;");

    Wait();
    EXPECT_CALL(h, get_allocator()).WillRepeatedly(Return(nullptr));
    EXPECT_CALL(h, handle_message(IsExtCanFrameWithId(0x195B4777), _)).WillOnce(
        WithArg<1>(Invoke(&InvokeNotification)));
    EXPECT_CALL(h, handle_message(IsExtCanFrameWithId(0x195B4222), _)).WillOnce(
        WithArg<1>(Invoke(&InvokeNotification)));
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

TEST_F(AsyncIfTest, WriteMultipleFrames)
{
    for (int i = 0; i < 10; ++i)
    {
        ExpectPacket(":X195B432DNAA;");
        TypedSyncAllocation<CanFrameWriteFlow> w(if_can_->write_allocator());
        struct can_frame* f = w.result()->mutable_frame();
        SET_CAN_FRAME_EFF(*f);
        SET_CAN_FRAME_ID_EFF(*f, 0x195B432D);
        f->can_dlc = 1;
        f->data[0] = 0xaa;
        w.result()->Send(nullptr);
        TypedSyncAllocation<CanFrameWriteFlow> ww(if_can_->write_allocator());
        SET_CAN_FRAME_RTR(*ww.result()->mutable_frame());
        ww.result()->Cancel();
    }
}

class AsyncMessageCanTests : public AsyncIfTest
{
protected:
    AsyncMessageCanTests()
    {
        if_can_->AddWriteFlows(2, 2);
    }
};

TEST_F(AsyncMessageCanTests, WriteByMTI)
{
    TypedSyncAllocation<WriteFlow> falloc(if_can_->global_write_allocator());

    ExpectPacket(":X195B422AN0102030405060708;");
    falloc.result()->WriteGlobalMessage(If::MTI_EVENT_REPORT, TEST_NODE_ID,
                                        EventIdToBuffer(0x0102030405060708ULL),
                                        nullptr);
}

TEST_F(AsyncMessageCanTests, WriteByMTIShort)
{
    TypedSyncAllocation<WriteFlow> falloc(if_can_->global_write_allocator());

    ExpectPacket(":X195B422AN3132333435;");
    Buffer* b = buffer_alloc(5);
    const char data[] = "12345";
    memcpy(b->start(), data, 5);
    b->advance(5);
    falloc.result()->WriteGlobalMessage(If::MTI_EVENT_REPORT, TEST_NODE_ID, b,
                                        nullptr);
}

TEST_F(AsyncMessageCanTests, WriteByMTIAddressedShort)
{
    TypedSyncAllocation<WriteFlow> falloc(if_can_->global_write_allocator());

    ExpectPacket(":X1982822AN00003132333435;");
    Buffer* b = buffer_alloc(5);
    const char data[] = "12345";
    memcpy(b->start(), data, 5);
    b->advance(5);
    falloc.result()->WriteGlobalMessage(If::MTI_PROTOCOL_SUPPORT_INQUIRY,
                                        TEST_NODE_ID, b, nullptr);
}

TEST_F(AsyncMessageCanTests, WriteByMTIAddressedFragmented)
{
    TypedSyncAllocation<WriteFlow> falloc(if_can_->global_write_allocator());

    ExpectPacket(":X1982822AN1000303132333435;"); // first frame
    ExpectPacket(":X1982822AN3000363738393031;"); // middle frame
    ExpectPacket(":X1982822AN3000323334353637;"); // middle frame
    ExpectPacket(":X1982822AN20003839;");         // last frame
    Buffer* b = buffer_alloc(20);
    const char data[] = "01234567890123456789";
    memcpy(b->start(), data, 20);
    b->advance(20);
    falloc.result()->WriteGlobalMessage(If::MTI_PROTOCOL_SUPPORT_INQUIRY,
                                        TEST_NODE_ID, b, nullptr);
}

TEST_F(AsyncMessageCanTests, WriteByMTIMultiple)
{
    EXPECT_CALL(can_bus_, MWrite(":X195B422AN0102030405060708;")).Times(100);
    for (int i = 0; i < 100; ++i)
    {
        TypedSyncAllocation<WriteFlow> falloc(
            if_can_->global_write_allocator());
        falloc.result()->WriteGlobalMessage(
            If::MTI_EVENT_REPORT, TEST_NODE_ID,
            EventIdToBuffer(0x0102030405060708ULL), nullptr);
    }
}

TEST_F(AsyncMessageCanTests, WriteByMTIIgnoreDatagram)
{
    TypedSyncAllocation<WriteFlow> falloc(if_can_->global_write_allocator());

    EXPECT_CALL(can_bus_, MWrite(_)).Times(0);
    falloc.result()->WriteGlobalMessage(If::MTI_DATAGRAM, TEST_NODE_ID,
                                        EventIdToBuffer(0x0102030405060708ULL),
                                        nullptr);
}

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
    if (memcmp(&value, arg->start(), 8)) return false;
    return true;
}

TEST_F(AsyncMessageCanTests, WriteByMTIGlobalDoesLoopback)
{
    StrictMock<MockMessageHandler> h;
    EXPECT_CALL(h, get_allocator()).WillRepeatedly(Return(nullptr));
    EXPECT_CALL(
        h, handle_message(
               Pointee(AllOf(Field(&IncomingMessage::mti, If::MTI_EVENT_REPORT),
                             Field(&IncomingMessage::payload, NotNull()),
                             Field(&IncomingMessage::payload,
                                   IsBufferValue(0x0102030405060708ULL)))),
               _)).WillOnce(WithArg<1>(Invoke(&InvokeNotification)));
    if_can_->dispatcher()->RegisterHandler(0, 0, &h);

    TypedSyncAllocation<WriteFlow> falloc(if_can_->global_write_allocator());
    ExpectPacket(":X195B422AN0102030405060708;");
    falloc.result()->WriteGlobalMessage(If::MTI_EVENT_REPORT, TEST_NODE_ID,
                                        EventIdToBuffer(0x0102030405060708ULL),
                                        nullptr);
    Wait();
}

TEST_F(AsyncMessageCanTests, WriteByMTIAllocatesLocalAlias)
{
    TypedSyncAllocation<WriteFlow> falloc(if_can_->global_write_allocator());

    CreateAllocatedAlias();
    ExpectNextAliasAllocation();
    ExpectPacket(":X1070133AN02010D000004;");
    ExpectPacket(":X195B433AN0102030405060708;");
    falloc.result()->WriteGlobalMessage(If::MTI_EVENT_REPORT, TEST_NODE_ID + 1,
                                        EventIdToBuffer(0x0102030405060708ULL),
                                        nullptr);
    Wait();
    EXPECT_EQ(0x33AU, if_can_->local_aliases()->lookup(TEST_NODE_ID + 1));
    EXPECT_EQ(TEST_NODE_ID + 1,
              if_can_->local_aliases()->lookup(NodeAlias(0x33A)));
}

} // namespace NMRAnet

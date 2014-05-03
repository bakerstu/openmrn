#include "utils/async_if_test_helper.hxx"

#include "nmranet/GlobalEventHandler.hxx"
#include "nmranet/NMRAnetAsyncEventHandler.hxx"
#include "nmranet/NMRAnetEventTestHelper.hxx"

namespace NMRAnet
{

class AsyncEventTest : public AsyncNodeTest
{
protected:
    AsyncEventTest()
    {
    }

    ~AsyncEventTest()
    {
        wait();
    }

    void wait()
    {
        wait_for_event_thread();
        AsyncNodeTest::wait();
    }

    StrictMock<MockEventHandler> h1_;
    StrictMock<MockEventHandler> h2_;
    StrictMock<MockEventHandler> h3_;
    StrictMock<MockEventHandler> h4_;
};

TEST_F(AsyncEventTest, Setup)
{
}

TEST_F(AsyncEventTest, MockEventHandler)
{
    NMRAnetEventRegistry::instance()->register_handlerr(&h1_, 0, 64);
    EXPECT_CALL(h1_, HandleEventReport(_, _))
        .WillOnce(WithArg<1>(Invoke(&InvokeNotification)));
    send_packet(":X195B4621N0102030405060702;");
}

TEST_F(AsyncEventTest, EventReportEventField)
{
    NMRAnetEventRegistry::instance()->register_handlerr(&h1_, 0, 64);
    EXPECT_CALL(
        h1_, HandleEventReport(
                 Pointee(Field(&EventReport::event, 0x0102030405060702ULL)), _))
        .WillOnce(WithArg<1>(Invoke(&InvokeNotification)));
    send_packet(":X195B4621N0102030405060702;");
}

TEST_F(AsyncEventTest, EventReportFields)
{
    static const NodeAlias alias = 0x621U;
    static const NodeID node_id = 0x050101FFFF3DULL;
    ifCan_->remote_aliases()->add(node_id, alias);
    NMRAnetEventRegistry::instance()->register_handlerr(&h1_, 0, 64);
    EXPECT_CALL(
        h1_,
        HandleEventReport(
            Pointee(AllOf(
                Field(&EventReport::src_node, Field(&NodeHandle::alias, alias)),
                Field(&EventReport::src_node, Field(&NodeHandle::id, node_id)),
                Field(&EventReport::dst_node, IsNull()),
                Field(&EventReport::event, 0x0102030405060702ULL),
                Field(&EventReport::mask, 1))),
            _)).WillOnce(WithArg<1>(Invoke(&InvokeNotification)));
    send_packet(":X195B4621N0102030405060702;");
}

TEST_F(AsyncEventTest, EventReportUnknownNode)
{
    static const NodeAlias alias = 0x631U;
    NMRAnetEventRegistry::instance()->register_handlerr(&h1_, 0, 64);
    EXPECT_CALL(
        h1_,
        HandleEventReport(
            Pointee(AllOf(
                Field(&EventReport::src_node, Field(&NodeHandle::alias, alias)),
                Field(&EventReport::src_node, Field(&NodeHandle::id, 0)),
                Field(&EventReport::dst_node, IsNull()),
                Field(&EventReport::event, 0x0102030405060703ULL),
                Field(&EventReport::mask, 1))),
            _)).WillOnce(WithArg<1>(Invoke(&InvokeNotification)));
    send_packet(":X195B4631N0102030405060703;");
}

TEST_F(AsyncEventTest, ProducerRangeIdentified)
{
    NMRAnetEventRegistry::instance()->register_handlerr(&h1_, 0, 64);
    EXPECT_CALL(
        h1_,
        HandleProducerRangeIdentified(
            Pointee(AllOf(Field(&EventReport::event, 0x0102030405060000ULL),
                          Field(&EventReport::mask, 0xFFFF))),
            _)).WillOnce(WithArg<1>(Invoke(&InvokeNotification)));
    send_packet(":X19524621N010203040506FFFF;");
}

TEST_F(AsyncEventTest, ManyEvents)
{
    NMRAnetEventRegistry::instance()->register_handlerr(&h1_, 0, 64);
    EXPECT_CALL(
        h1_, HandleEventReport(
                 Pointee(Field(&EventReport::event, 0x01020304050655aaULL)), _))
        .Times(100)
        .WillRepeatedly(WithArg<1>(Invoke(&InvokeNotification)));
    for (int i = 0; i < 100; ++i) {
        send_packet(":X195B4621N01020304050655aa;");
    }
}

} // namespace NMRAnet

#include "utils/async_if_test_helper.hxx"
#include "nmranet/GlobalEventHandler.hxx"
#include "nmranet/GlobalEventHandlerImpl.hxx"
#include "nmranet/NMRAnetEventRegistry.hxx"
#include "nmranet/NMRAnetEventTestHelper.hxx"
#include "nmranet/EndianHelper.hxx"

using testing::Eq;
using testing::Field;
using testing::InSequence;
using testing::Invoke;
using testing::InvokeWithoutArgs;
using testing::StrictMock;
using testing::WithArg;
using testing::_;

namespace NMRAnet
{

extern void DecodeRange(EventReport *r);

class DecodeRangeTest : public testing::Test
{
protected:
    void ExpectDecode(uint64_t complex, uint64_t exp_event, uint64_t exp_mask)
    {
        EventReport r;
        r.event = complex;
        DecodeRange(&r);
        EXPECT_EQ(exp_event, r.event);
        EXPECT_EQ(exp_mask, r.mask);
    }

    uint64_t eventid_;
    uint64_t mask_;
};

TEST_F(DecodeRangeTest, TrivPositive)
{
    ExpectDecode(0b1100, 0b1100, 0b0011);
}

TEST_F(DecodeRangeTest, TrivNegative)
{
    ExpectDecode(0b110011, 0b110000, 0b0011);
}

TEST_F(DecodeRangeTest, SimplePositive)
{
    ExpectDecode(0b1010101110000, 0b1010101110000, 0b1111);
}

TEST_F(DecodeRangeTest, SimpleNegative)
{
    ExpectDecode(0b101010111000011111, 0b101010111000000000, 0b11111);
}

TEST_F(DecodeRangeTest, LongPositive)
{
    ExpectDecode(0xfffffffffffffff0ULL, 0xfffffffffffffff0ULL, 0xf);
}

TEST_F(DecodeRangeTest, LongNegative)
{
    ExpectDecode(0xffffffffffffff0fULL, 0xffffffffffffff00ULL, 0xf);
}

static const uint64_t kExitEventId = 0x0808080804040404ULL;
static const uint64_t kTestEventId = 0x0102030405060708ULL;
static const If::MTI kEventReportMti = If::MTI_EVENT_REPORT;
static const If::MTI kProducerIdentifiedResvdMti =
    If::MTI_PRODUCER_IDENTIFIED_RESERVED;
static const If::MTI kGlobalIdentifyEvents = If::MTI_EVENTS_IDENTIFY_GLOBAL;
static const If::MTI kAddressedIdentifyEvents =
    If::MTI_EVENTS_IDENTIFY_ADDRESSED;

class EventHandlerTests : public AsyncIfTest
{
protected:
    EventHandlerTests()
        : service_(ifCan_.get())
    {
    }

    ~EventHandlerTests()
    {
        wait();
    }

    void wait()
    {
        while (service_.event_processing_pending())
        {
            usleep(100);
        }
        AsyncIfTest::wait();
    }

    void send_message(If::MTI mti, uint64_t event)
    {
        auto *b = ifCan_->dispatcher()->alloc();
        b->data()->reset(mti, 0, {0, 0}, EventIDToPayload(event));
        ifCan_->dispatcher()->send(b);
    }

    GlobalEventService service_;
    StrictMock<MockEventHandler> h1_;
    StrictMock<MockEventHandler> h2_;
    StrictMock<MockEventHandler> h3_;
    StrictMock<MockEventHandler> h4_;
};

TEST_F(EventHandlerTests, SimpleRunTest)
{
    NMRAnetEventRegistry::instance()->register_handler(&h1_, 0, 0);
    NMRAnetEventRegistry::instance()->register_handler(&h2_, 0, 0);
    EXPECT_CALL(h1_, HandleEventReport(_, _))
        .WillOnce(WithArg<1>(Invoke(&InvokeNotification)));
    EXPECT_CALL(h2_, HandleEventReport(_, _))
        .WillOnce(WithArg<1>(Invoke(&InvokeNotification)));
    send_message(kEventReportMti, kTestEventId);
    wait();
}

TEST_F(EventHandlerTests, SimpleRunTest2)
{
    NMRAnetEventRegistry::instance()->register_handler(&h1_, 0, 0);
    NMRAnetEventRegistry::instance()->register_handler(&h2_, 0, 0);
    EXPECT_CALL(h1_, HandleEventReport(_, _))
        .WillOnce(WithArg<1>(Invoke(&InvokeNotification)));
    EXPECT_CALL(h2_, HandleEventReport(_, _))
        .WillOnce(WithArg<1>(Invoke(&InvokeNotification)));
    send_packet(":X195B4111N0102030405060708;");
    wait();
}

TEST_F(EventHandlerTests, Run100EventsTest)
{
    NMRAnetEventRegistry::instance()->register_handler(&h1_, 0, 0);
    NMRAnetEventRegistry::instance()->register_handler(&h2_, 0, 0);
    EXPECT_CALL(h1_, HandleEventReport(_, _)).Times(100).WillRepeatedly(
        WithArg<1>(Invoke(&InvokeNotification)));
    EXPECT_CALL(h2_, HandleEventReport(_, _)).Times(100).WillRepeatedly(
        WithArg<1>(Invoke(&InvokeNotification)));
    for (int i = 0; i < 100; i++)
    {
        send_message(kEventReportMti, 0x0102030405060708ULL);
    }
    wait();
}

TEST_F(EventHandlerTests, EventsOrderTest)
{
    NMRAnetEventRegistry::instance()->register_handler(&h1_, 0, 0);
    {
        InSequence s;

        EXPECT_CALL(h1_, HandleEventReport(
                             Field(&EventReport::event, kTestEventId + 0), _))
            .WillOnce(WithArg<1>(Invoke(&InvokeNotification)));
        EXPECT_CALL(h1_, HandleEventReport(
                             Field(&EventReport::event, kTestEventId + 1), _))
            .WillOnce(WithArg<1>(Invoke(&InvokeNotification)));
        EXPECT_CALL(h1_, HandleEventReport(
                             Field(&EventReport::event, kTestEventId + 2), _))
            .WillOnce(WithArg<1>(Invoke(&InvokeNotification)));
    }
    BlockExecutor block(nullptr);
    for (int i = 0; i < 3; i++)
    {
        send_message(kEventReportMti, kTestEventId + i);
    }
    block.release_block();
    wait();
}

TEST_F(EventHandlerTests, GlobalRunTest1)
{
    NMRAnetEventRegistry::instance()->register_handler(&h1_, 0, 0);
    NMRAnetEventRegistry::instance()->register_handler(&h2_, 0, 0);
    EXPECT_CALL(h1_, HandleIdentifyGlobal(_, _))
        .WillOnce(WithArg<1>(Invoke(&InvokeNotification)));
    EXPECT_CALL(h2_, HandleIdentifyGlobal(_, _))
        .WillOnce(WithArg<1>(Invoke(&InvokeNotification)));
    auto *b = ifCan_->dispatcher()->alloc();
    b->data()->reset(kGlobalIdentifyEvents, 0, "");
    ifCan_->dispatcher()->send(b);
    wait();
}

TEST_F(EventHandlerTests, GlobalAndLocal)
{
    NMRAnetEventRegistry::instance()->register_handler(&h1_, 0, 0);
    NMRAnetEventRegistry::instance()->register_handler(&h2_, 0, 0);
    EXPECT_CALL(h1_, HandleIdentifyGlobal(_, _))
        .WillOnce(WithArg<1>(Invoke(&InvokeNotification)));
    EXPECT_CALL(h2_, HandleIdentifyGlobal(_, _))
        .WillOnce(WithArg<1>(Invoke(&InvokeNotification)));
    EXPECT_CALL(h1_, HandleEventReport(_, _)).Times(100).WillRepeatedly(
        WithArg<1>(Invoke(&InvokeNotification)));
    EXPECT_CALL(h2_, HandleEventReport(_, _)).Times(100).WillRepeatedly(
        WithArg<1>(Invoke(&InvokeNotification)));
    send_packet(":X19970111N;");
    for (int i = 0; i < 100; i++)
    {
        send_message(kEventReportMti, 0x0102030405060708ULL);
    }
    wait();
}

} // namespace NMRAnet

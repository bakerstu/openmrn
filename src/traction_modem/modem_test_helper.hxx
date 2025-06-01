#include "traction_modem/ModemTrainHwInterface.hxx"
#include "traction_modem/Link.hxx"

#include "os/FakeClock.hxx"

namespace traction_modem
{

//
// Mock objects.
//
class MockRxFlow : public RxInterface
{
public:
    MOCK_METHOD1(start, void(int));
    MOCK_METHOD3(register_handler,
        void(PacketFlowInterface*, Message::id_type, Message::id_type));
    MOCK_METHOD3(unregister_handler,
        void(PacketFlowInterface*, Message::id_type, Message::id_type));
    MOCK_METHOD1(unregister_handler_all, void(PacketFlowInterface*));
    MOCK_METHOD1(register_fallback_handler, void(PacketFlowInterface*));
};

class MyMockRxFlow : public MockRxFlow
{
public:
    void register_handler(PacketFlowInterface *interface, Message::id_type id,
        Message::id_type mask = Message::EXACT_MASK) override
    {
        dispatcher_.register_handler(interface, id, mask);
        MockRxFlow::register_handler(interface, id, mask);
    }
 
    void unregister_handler_all(PacketFlowInterface *interface) override
    {
        dispatcher_.unregister_handler_all(interface);
        MockRxFlow::unregister_handler_all(interface);
    }

    DispatchFlow<Buffer<Message>, 2> dispatcher_{&g_service};
};

class MockTxFlow : public TxInterface
{
public:
    MOCK_METHOD1(start, void(int));
    MOCK_METHOD1(send_packet, void(Defs::Payload));
};

class MockTrainHwInterface : public ModemTrainHwInterface
{
public:
    MOCK_METHOD2(output_state, void(uint16_t, uint16_t));
    MOCK_METHOD1(output_restart, void(uint16_t));
};

/// Base class for tests that handles some of the "link" infrastructure.
class LinkTestBase : public ::testing::Test
{
protected:
    /// Constructor
    LinkTestBase()
        : link_(&mTxFlow_, &mRxFlow_)
        , linkManager_(&g_service, &link_)
    {
    }

    /// Destructor.
    ~LinkTestBase()
    {
        do_link_manager_shutdown();
    }

    /// Fake clock helper
    /// @param nsec how far to advance in nanoseconds
    /// @param step_size_msec step size in milliseconds
    /// @param pong true to send ping responses to keep the link up.
    void clk_advance(
        long long nsec, unsigned step_size_msec = 10, bool pong = true)
    {
        static long long last_pong_time = 0;
        static long long current_time = 0;
        using ::testing::StartsWith;
        using namespace std::literals;

        // Ignore ping/pong.
        EXPECT_CALL(mTxFlow_, send_packet(StartsWith(
            "\x41\xD2\xC3\x7A\x00\x00\x00\x00"s))).Times(::testing::AtLeast(0));
        while (nsec > 0)
        {
            nsec -= MSEC_TO_NSEC(step_size_msec);
            current_time += MSEC_TO_NSEC(step_size_msec);
            clk_.advance(MSEC_TO_NSEC(step_size_msec));
            if (current_time >= (last_pong_time + SEC_TO_NSEC(1)))
            {
                // Send pong to prevent link down
                last_pong_time = current_time;
                if (pong && link_.is_link_up())
                {
                    auto b = linkManager_.alloc();
                    Defs::prepare(&b->data()->payload, Defs::RESP_PING, 4);
                    Defs::append_uint8(&b->data()->payload, 0);
                    Defs::append_uint8(&b->data()->payload, 0);
                    Defs::append_uint8(&b->data()->payload, 0);
                    Defs::append_uint8(&b->data()->payload, 0);
                    Defs::append_crc(&b->data()->payload);
                    static_cast<PacketFlowInterface*>(&linkManager_)->send(b);
                }
            }
            wait_for_main_executor();
        }
    }

    /// Bring up the link.
    void do_link_up()
    {
        using ::testing::StartsWith;
        using namespace std::literals;

        {
            ::testing::Sequence s1, s2;
            EXPECT_CALL(mTxFlow_, start(50)).Times(1).InSequence(s1);
            EXPECT_CALL(mRxFlow_, start(50)).Times(1).InSequence(s2);
            EXPECT_CALL(mRxFlow_, register_handler(
                &linkManager_, Defs::RESP_PING, Message::EXACT_MASK)).Times(1)
                .InSequence(s1, s2);
            EXPECT_CALL(mTxFlow_, send_packet(StartsWith(
                "\x41\xD2\xC3\x7A\x00\x00\x00\x00"s))).Times(1)
                .InSequence(s1, s2);
            link_.start(50);
            wait_for_main_executor();
        }
        {
            // Now send the ping response.
            ::testing::Sequence s1;
            EXPECT_CALL(mRxFlow_, unregister_handler(
                &linkManager_, Defs::RESP_PING, Message::EXACT_MASK)).Times(1)
                .InSequence(s1);
            EXPECT_CALL(mRxFlow_, register_handler(
                &linkManager_, Defs::RESP_PING, Message::EXACT_MASK)).Times(1)
                .InSequence(s1);
            EXPECT_FALSE(link_.is_link_up());
            auto b = linkManager_.alloc();
            Defs::prepare(&b->data()->payload, Defs::RESP_PING, 4);
            Defs::append_uint8(&b->data()->payload, 0);
            Defs::append_uint8(&b->data()->payload, 0);
            Defs::append_uint8(&b->data()->payload, 0);
            Defs::append_uint8(&b->data()->payload, 0);
            Defs::append_crc(&b->data()->payload);
            static_cast<PacketFlowInterface*>(&linkManager_)->send(b);
            wait_for_main_executor();
            EXPECT_TRUE(link_.is_link_up());
        }
    }

    /// Shutdown the link manager
    void do_link_manager_shutdown()
    {
        using ::testing::StartsWith;
        using namespace std::literals;
        wait_for_main_executor();
        testing::Mock::VerifyAndClearExpectations(&mTxFlow_);
        testing::Mock::VerifyAndClearExpectations(&mRxFlow_);
        EXPECT_CALL(mRxFlow_, unregister_handler(
            &linkManager_, Defs::RESP_PING, Message::EXACT_MASK))
            .Times(::testing::AtLeast(0));
        EXPECT_CALL(mRxFlow_, unregister_handler(
            &linkManager_, Defs::RESP_BAUD_RATE_QUERY,
            Message::EXACT_MASK)).Times(::testing::AtLeast(0));
        EXPECT_CALL(mTxFlow_, send_packet(StartsWith(
            "\x41\xD2\xC3\x7A\x00\x00\x00\x00"s))).Times(::testing::AtLeast(0));
        linkManager_.TEST_shutdown();
        clk_advance(SEC_TO_NSEC(4), 10, false);
        wait_for_main_timers();
        testing::Mock::VerifyAndClearExpectations(&mTxFlow_);
        testing::Mock::VerifyAndClearExpectations(&mRxFlow_);
    }

    FakeClock clk_;
    ::testing::StrictMock<MockTxFlow> mTxFlow_; ///< mock transmit flow
    ::testing::StrictMock<MyMockRxFlow> mRxFlow_; ///< mock receive flow
    Link link_; ///< Link object
    LinkManager linkManager_; ///< Link Management.
};

} // namespace traction_modem
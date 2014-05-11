#include "utils/test_main.hxx"

#include "dcc/Packet.hxx"
#include "dcc/Loco.hxx"
#include "dcc/UpdateLoop.hxx"

using ::testing::StrictMock;
using ::testing::ElementsAre;
using ::testing::SaveArg;
using ::testing::Mock;
using ::testing::_;

namespace dcc {

class PacketTest : public ::testing::Test {
protected:
    vector<uint8_t> get_packet() {
        return vector<uint8_t>(pkt_.payload + 0, pkt_.payload + pkt_.dlc);
    }

    vector<uint8_t> packet(const std::initializer_list<int>& data) {
        vector<uint8_t> v;
        for (int b : data) {
            v.push_back(b & 0xff);
        }
        return v;
    }

    Packet pkt_;
};

TEST_F(PacketTest, ChecksumRegular) {
    pkt_.dlc = 3;
    pkt_.payload[0] = 0xA0;
    pkt_.payload[1] = 0x53;
    pkt_.payload[2] = 0x0F;
    pkt_.add_dcc_checksum();
    EXPECT_EQ(4, pkt_.dlc);
    EXPECT_EQ(0xFC, pkt_.payload[3]);
}

TEST_F(PacketTest, ChecksumEmpty) {
    pkt_.dlc = 0;
    pkt_.add_dcc_checksum();
    EXPECT_EQ(1, pkt_.dlc);
    EXPECT_EQ(0, pkt_.payload[3]);
}

TEST_F(PacketTest, ChecksumFF) {
    pkt_.dlc = 3;
    pkt_.payload[0] = 0xFF;
    pkt_.payload[1] = 0xFF;
    pkt_.payload[2] = 0xFF;
    pkt_.add_dcc_checksum();
    EXPECT_EQ(4, pkt_.dlc);
    EXPECT_EQ(0xFF, pkt_.payload[3]);
}


TEST_F(PacketTest, DccSpeed28) {
    pkt_.set_dcc_speed28(DccShortAddress(55), true, 6);
    EXPECT_THAT(get_packet(), ElementsAre(0b00110111, 0b01110100, 0b01000011));
}

TEST_F(PacketTest, DccSpeed28_zero) {
    pkt_.set_dcc_speed28(DccShortAddress(55), true, 0);
    EXPECT_THAT(get_packet(), ElementsAre(55, 0b01100000, _));
}

TEST_F(PacketTest, DccSpeed28_reversezero) {
    pkt_.set_dcc_speed28(DccShortAddress(55), false, 0);
    EXPECT_THAT(get_packet(), ElementsAre(55, 0b01000000, _));
}

TEST_F(PacketTest, DccSpeed28_estop) {
    pkt_.set_dcc_speed28(DccShortAddress(55), true, Packet::EMERGENCY_STOP);
    EXPECT_THAT(get_packet(), ElementsAre(55, 0b01100001, _));
}

TEST_F(PacketTest, DccSpeed28_step123) {
    pkt_.set_dcc_speed28(DccShortAddress(55), true, 1);
    EXPECT_THAT(get_packet(), ElementsAre(55, 0b01100010, _));

    pkt_.set_dcc_speed28(DccShortAddress(55), true, 2);
    EXPECT_THAT(get_packet(), ElementsAre(55, 0b01110010, _));

    pkt_.set_dcc_speed28(DccShortAddress(55), true, 3);
    EXPECT_THAT(get_packet(), ElementsAre(55, 0b01100011, _));
}

TEST_F(PacketTest, DccIdle) {
    pkt_.set_dcc_idle();
    EXPECT_THAT(get_packet(), ElementsAre(0xff, 0, 0xff));
}

TEST_F(PacketTest, DccReset) {
    pkt_.set_dcc_reset_all_decoders();
    EXPECT_THAT(get_packet(), ElementsAre(0, 0, 0));
}

class MockUpdateLoop;
MockUpdateLoop* g_update_loop = nullptr;

class MockUpdateLoop {
public:
    MockUpdateLoop() {
        HASSERT(!g_update_loop);
        g_update_loop = this;
    }
    ~MockUpdateLoop() {
        g_update_loop = nullptr;
    }
    MOCK_METHOD2(send_update, void(PacketSource* source, unsigned code));
};

void packet_processor_notify_update(PacketSource* source, unsigned code) {
    HASSERT(g_update_loop);
    g_update_loop->send_update(source, code);
}

class Train28Test : public PacketTest {
protected:
    Train28Test()
        : code_(0), train_(DccShortAddress(55)) {}

    void do_refresh() {
        new (&pkt_) Packet();
        train_.get_next_packet(0, &pkt_);
    }

    void do_callback() {
        Mock::VerifyAndClear(&loop_);
        new (&pkt_) Packet();
        train_.get_next_packet(code_, &pkt_);
        code_ = 0;
    }

    unsigned code_;
    StrictMock<MockUpdateLoop> loop_;
    Dcc28Train train_;
};

TEST_F(Train28Test, DefaultPacket) {
    do_refresh();
    EXPECT_THAT(get_packet(), ElementsAre(55, 0b01100000, _));
}

TEST_F(Train28Test, SetSpeed) {
    EXPECT_CALL(loop_, send_update(&train_, _)).WillOnce(SaveArg<1>(&code_));
    train_.set_speed(SpeedType(37.5));
    do_callback();
    EXPECT_THAT(get_packet(), ElementsAre(55, 0b01101011, _));
}

TEST_F(Train28Test, MaxSpeed) {
    EXPECT_CALL(loop_, send_update(&train_, _)).WillOnce(SaveArg<1>(&code_));
    SpeedType s;
    s.set_mph(128);
    train_.set_speed(s);
    do_callback();
    EXPECT_THAT(get_packet(), ElementsAre(55, 0b01111111, _));
}

TEST_F(Train28Test, BelowMaxSpeed) {
    EXPECT_CALL(loop_, send_update(&train_, _)).WillOnce(SaveArg<1>(&code_));
    SpeedType s;
    s.set_mph(123.42);
    train_.set_speed(s);
    do_callback();
    EXPECT_THAT(get_packet(), ElementsAre(55, 0b01101111, _));
}

TEST_F(Train28Test, MinSpeed) {
    EXPECT_CALL(loop_, send_update(&train_, _)).WillOnce(SaveArg<1>(&code_));
    SpeedType s;
    // Even the smallest nonzero velocity should set the train in motion. 
    s.set_mph(0.001);
    train_.set_speed(s);
    do_callback();
    EXPECT_THAT(get_packet(), ElementsAre(55, 0b01100010, _));

    SpeedType ss = train_.get_speed();
    EXPECT_NEAR(0.001, ss.mph(), 1e-7);
}

TEST_F(Train28Test, ZeroSpeed) {
    EXPECT_CALL(loop_, send_update(&train_, _)).Times(2).WillRepeatedly(SaveArg<1>(&code_));
    SpeedType s;
    // Even the smallest nonzero velocity should set the train in motion. 
    s.set_mph(0.001);
    train_.set_speed(s);
    do_callback();
    EXPECT_THAT(get_packet(), ElementsAre(55, 0b01100010, _));

    s.set_mph(0);
    train_.set_speed(s);
    do_callback();
    EXPECT_THAT(get_packet(), ElementsAre(55, 0b01100000, _));
}

}  // namespace dcc

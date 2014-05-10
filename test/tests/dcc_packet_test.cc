#include "utils/test_main.hxx"

#include "dcc/Packet.hxx"

namespace dcc {

class PacketTest : public ::testing::Test {

protected:
    Packet pkt_;
};

TEST_F(PacketTest, ChecksumRegular) {
    pkt_.dlc = 3;
    pkt_.payload[0] = 0xA0;
    pkt_.payload[1] = 0x53;
    pkt_.payload[2] = 0x0F;
    pkt_.AddDccChecksum();
    EXPECT_EQ(4, pkt_.dlc);
    EXPECT_EQ(0xFC, pkt_.payload[3]);
}

TEST_F(PacketTest, ChecksumEmpty) {
    pkt_.dlc = 0;
    pkt_.AddDccChecksum();
    EXPECT_EQ(1, pkt_.dlc);
    EXPECT_EQ(0, pkt_.payload[3]);
}

TEST_F(PacketTest, ChecksumFF) {
    pkt_.dlc = 3;
    pkt_.payload[0] = 0xFF;
    pkt_.payload[1] = 0xFF;
    pkt_.payload[2] = 0xFF;
    pkt_.AddDccChecksum();
    EXPECT_EQ(4, pkt_.dlc);
    EXPECT_EQ(0xFF, pkt_.payload[3]);
}

}  // namespace dcc

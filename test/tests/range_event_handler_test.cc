#include "utils/if_test_helper.hxx"

#include "nmranet/NMRAnetEventRegistry.hxx"
#include "nmranet/EventHandlerTemplates.hxx"

using ::testing::InSequence;

static const uint64_t kEventBase = 0x05010101FFFF0000ULL;

namespace NMRAnet {

class BitRangeEventTest : public IfTest {
 protected:
  BitRangeEventTest()
      : handler_(node_, kEventBase, (uint32_t*)storage_, 3000) {
    memset(storage_, 0, sizeof(storage_));
  }

  int32_t storage_[100];
  BitRangeEventPC handler_;
};


TEST_F(BitRangeEventTest, Simple) {
  WaitForEventThread();
}

TEST_F(BitRangeEventTest, ReportOnOff) {
  EXPECT_EQ(0x00000000, storage_[0]);
  EXPECT_FALSE(handler_.Get(16));
  SendPacket(":X195B4001N05010101FFFF0020;");
  WaitForEventThread();
  EXPECT_EQ(0x00010000, storage_[0]);
  EXPECT_TRUE(handler_.Get(16));
  SendPacket(":X195B4001N05010101FFFF0021;");
  WaitForEventThread();
  EXPECT_EQ(0x00000000, storage_[0]);
  EXPECT_FALSE(handler_.Get(16));
}

TEST_F(BitRangeEventTest, ReportMultiOnOff) {
  EXPECT_EQ(0, storage_[10]);
  SendPacket(":X195B4001N05010101FFFF0280;");
  SendPacket(":X195B4001N05010101FFFF0282;");
  WaitForEventThread();
  EXPECT_TRUE(handler_.Get(320));
  EXPECT_TRUE(handler_.Get(321));
  EXPECT_EQ(3, storage_[10]);
  SendPacket(":X195B4001N05010101FFFF0281;");
  WaitForEventThread();
  EXPECT_EQ(2, storage_[10]);
}

TEST_F(BitRangeEventTest, InquireProducerAndConsumer) {
  EXPECT_EQ(0, storage_[10]);
  SendPacket(":X195B4001N05010101FFFF0280;");
  SendPacket(":X195B4001N05010101FFFF0282;");
  WaitForEventThread();

  // identify producers event-on; identified valid.
  SendPacketAndExpectResponse(":X19914001N05010101FFFF0282;",
                              ":X1954412DN05010101FFFF0282;");

  // identify producers event-off; identified invalid.
  SendPacketAndExpectResponse(":X19914001N05010101FFFF0283;",
                              ":X1954512DN05010101FFFF0283;");

  // identify consumers event-on; identified valid.
  SendPacketAndExpectResponse(":X198F4001N05010101FFFF0282;",
                              ":X194C412DN05010101FFFF0282;");

  // identify consumers event-off; identified invalid.
  SendPacketAndExpectResponse(":X198F4001N05010101FFFF0283;",
                              ":X194C512DN05010101FFFF0283;");

  // set event off
  storage_[10] &= ~2;

    // identify producers event-off; identified valid.
  SendPacketAndExpectResponse(":X19914001N05010101FFFF0283;",
                              ":X1954412DN05010101FFFF0283;");

  // identify producers event-on; identified invalid.
  SendPacketAndExpectResponse(":X19914001N05010101FFFF0282;",
                              ":X1954512DN05010101FFFF0282;");

  // identify consumers event-on; identified invalid.
  SendPacketAndExpectResponse(":X198F4001N05010101FFFF0282;",
                              ":X194C512DN05010101FFFF0282;");

  // identify consumers event-off; identified valid.
  SendPacketAndExpectResponse(":X198F4001N05010101FFFF0283;",
                              ":X194C412DN05010101FFFF0283;");
}

TEST_F(BitRangeEventTest, IdentifyGlobal) {
  // We have 3000 bits, which need 6000 events, whioch should be rounded up to
  // a range of 8192 or 0x2000: the mask will end with 0x1FFF.
  ExpectPacket(":X194A412DN05010101FFFF1FFF;");
  ExpectPacket(":X1952412DN05010101FFFF1FFF;");
  SendPacket(":X19970001N;");
  WaitForEventThread();
  //nmranet_identify_consumers();
}

TEST_F(BitRangeEventTest, ProduceBits) {
  ExpectPacket(":X195B412DN05010101FFFF0280;");
  handler_.Set(320, true, &event_write_helper1, nullptr);
  WaitForEventThread(); Mock::VerifyAndClear(&can_bus_);

  // Another set will not produce another event.
  handler_.Set(320, true, &event_write_helper1, nullptr);
  WaitForEventThread(); Mock::VerifyAndClear(&can_bus_);

  ExpectPacket(":X195B412DN05010101FFFF0281;");
  handler_.Set(320, false, &event_write_helper1, nullptr);
  WaitForEventThread(); Mock::VerifyAndClear(&can_bus_);

  handler_.Set(320, false, &event_write_helper1, nullptr);
  WaitForEventThread(); Mock::VerifyAndClear(&can_bus_);
}


TEST_F(BitRangeEventTest, IgnoreUnrelated) {
  // Sets the expectation that no output packet shall be produced.
  EXPECT_CALL(can_bus_, MWrite(_)).Times(0);

  EXPECT_EQ(0, storage_[10]);
  SendPacket(":X195B4001N05010101FFFF176E;");
  WaitForEventThread();
  EXPECT_EQ(1<<23, storage_[93]);

  SendPacket(":X195B4001N05010101FFFF176F;");
  WaitForEventThread();
  EXPECT_EQ(0, storage_[93]);

  SendPacket(":X195B4001N05010101FFFF1770;");
  WaitForEventThread();
  EXPECT_EQ(0, storage_[93]);

  // No responses.
  SendPacket(":X19914001N05010101FFFF1770;");
  SendPacket(":X19914001N05010101FFFEFFFF;");
  WaitForEventThread();
}

TEST_F(BitRangeEventTest, DeathTooHighSet) {
  // Death tests are expensive for IfTests because they wait for the alias
  // reserve timeout, which is 1 second. Use them sparingly.
  EXPECT_DEATH({
      handler_.Set(3000, false, &event_write_helper1, nullptr);
    }, "bit < size");
}

}  // namespace NMRAnet

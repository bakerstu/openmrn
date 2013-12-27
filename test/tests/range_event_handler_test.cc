#include "utils/async_if_test_helper.hxx"

#include "nmranet/NMRAnetEventRegistry.hxx"
#include "nmranet/EventHandlerTemplates.hxx"

using ::testing::InSequence;

static const uint64_t kEventBase = 0x05010101FFFF0000ULL;

namespace NMRAnet {

class BitRangeEventTest : public AsyncNodeTest {
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
                              ":X1954422AN05010101FFFF0282;");

  // identify producers event-off; identified invalid.
  SendPacketAndExpectResponse(":X19914001N05010101FFFF0283;",
                              ":X1954522AN05010101FFFF0283;");

  // identify consumers event-on; identified valid.
  SendPacketAndExpectResponse(":X198F4001N05010101FFFF0282;",
                              ":X194C422AN05010101FFFF0282;");

  // identify consumers event-off; identified invalid.
  SendPacketAndExpectResponse(":X198F4001N05010101FFFF0283;",
                              ":X194C522AN05010101FFFF0283;");

  // set event off
  storage_[10] &= ~2;

    // identify producers event-off; identified valid.
  SendPacketAndExpectResponse(":X19914001N05010101FFFF0283;",
                              ":X1954422AN05010101FFFF0283;");

  // identify producers event-on; identified invalid.
  SendPacketAndExpectResponse(":X19914001N05010101FFFF0282;",
                              ":X1954522AN05010101FFFF0282;");

  // identify consumers event-on; identified invalid.
  SendPacketAndExpectResponse(":X198F4001N05010101FFFF0282;",
                              ":X194C522AN05010101FFFF0282;");

  // identify consumers event-off; identified valid.
  SendPacketAndExpectResponse(":X198F4001N05010101FFFF0283;",
                              ":X194C422AN05010101FFFF0283;");
}

TEST_F(BitRangeEventTest, IdentifyGlobal) {
  // We have 3000 bits, which need 6000 events, whioch should be rounded up to
  // a range of 8192 or 0x2000: the mask will end with 0x1FFF.
  ExpectPacket(":X194A422AN05010101FFFF1FFF;");
  ExpectPacket(":X1952422AN05010101FFFF1FFF;");
  SendPacket(":X19970001N;");
  WaitForEventThread();
  //nmranet_identify_consumers();
}

TEST_F(BitRangeEventTest, ProduceBits) {
  ExpectPacket(":X195B422AN05010101FFFF0280;");
  handler_.Set(320, true, &event_write_helper1, EmptyNotifiable::DefaultInstance());
  WaitForEventThread(); Mock::VerifyAndClear(&can_bus_);

  // Another set will not produce another event.
  handler_.Set(320, true, &event_write_helper1, EmptyNotifiable::DefaultInstance());
  WaitForEventThread(); Mock::VerifyAndClear(&can_bus_);

  ExpectPacket(":X195B422AN05010101FFFF0281;");
  handler_.Set(320, false, &event_write_helper1, EmptyNotifiable::DefaultInstance());
  WaitForEventThread(); Mock::VerifyAndClear(&can_bus_);

  handler_.Set(320, false, &event_write_helper1, EmptyNotifiable::DefaultInstance());
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
      handler_.Set(3000, false, &event_write_helper1, EmptyNotifiable::DefaultInstance());
    }, "bit < size");
}

class ByteRangeCTest : public AsyncNodeTest {
 protected:
  ByteRangeCTest()
      : handler_(node_, kEventBase, storage_, 10) {
    memset(storage_, 0, sizeof(storage_));
  }

  ~ByteRangeCTest() { Wait(); }

  uint8_t storage_[10];
  ByteRangeEventC handler_;
};

TEST_F(ByteRangeCTest, Simple) {
  WaitForEventThread();
}

TEST_F(ByteRangeCTest, ValueSet) {
  EXPECT_EQ(0, storage_[1]);
  EXPECT_EQ(0, storage_[2]);
  SendPacket(":X195B4001N05010101FFFF0120;");
  WaitForEventThread();
  EXPECT_EQ(0x20, storage_[1]);
  SendPacket(":X195B4001N05010101FFFF0242;");
  WaitForEventThread();
  EXPECT_EQ(0x42, storage_[2]);
  EXPECT_EQ(0x20, storage_[1]);
  EXPECT_EQ(0, storage_[0]);
  EXPECT_EQ(0, storage_[3]);
}

TEST_F(ByteRangeCTest, Query) {
  storage_[1] = 0x42;
  storage_[0] = 0x23;
  // identify consumers 1==42; identified valid.
  SendPacketAndExpectResponse(":X198F4001N05010101FFFF0142;",
                              ":X194C422AN05010101FFFF0142;");
  // 1==23, invalid
  SendPacketAndExpectResponse(":X198F4001N05010101FFFF0123;",
                              ":X194C522AN05010101FFFF0123;");
  // identify consumers 0==23; identified valid.
  SendPacketAndExpectResponse(":X198F4001N05010101FFFF0023;",
                              ":X194C422AN05010101FFFF0023;");
  // 0==42, invalid
  SendPacketAndExpectResponse(":X198F4001N05010101FFFF0042;",
                              ":X194C522AN05010101FFFF0042;");
}

TEST_F(ByteRangeCTest, IdentifyGlobal) {
  // We have 10 bytes of 256 events each, which should be rounded up to 16*256
  // events, mask 0x0fff.
  ExpectPacket(":X194A422AN05010101FFFF0FFF;");
  //pectPacket(":X1952422AN05010101FFFF0FFF;");
  SendPacket(":X19970001N;");
  WaitForEventThread();
}

class ByteRangePTest : public AsyncNodeTest {
 protected:
  ByteRangePTest()
      : handler_(node_, kEventBase, storage_, 10) {
    memset(storage_, 0, sizeof(storage_));
  }

  ~ByteRangePTest() { Wait(); }

  uint8_t storage_[10];
  ByteRangeEventP handler_;
  SyncNotifiable n_;
  WriteHelper write_helper_;
};

TEST_F(ByteRangePTest, Simple) {
  WaitForEventThread();
}

TEST_F(ByteRangePTest, Update) {
  storage_[1] = 0x42;
  storage_[0] = 0x23;
  ExpectPacket(":X195B422AN05010101FFFF0023;");
  handler_.Update(0, &write_helper_, &n_);
  n_.WaitForNotification();

  ExpectPacket(":X195B422AN05010101FFFF0142;");
  handler_.Update(1, &write_helper_, &n_);
  n_.WaitForNotification();
}

TEST_F(ByteRangePTest, Query) {
  storage_[1] = 0x42;
  storage_[0] = 0x23;
  // identify consumers 1==42; no response.
  SendPacket(":X198F4001N05010101FFFF0142;");

  // identify producer 1==42, valid.
  SendPacketAndExpectResponse(":X19914001N05010101FFFF0142;",
                              ":X1954422AN05010101FFFF0142;");
  // 1==23, invalid
  ExpectPacket(":X195B422AN05010101FFFF0142;"); // we also get the true value
  SendPacketAndExpectResponse(":X19914001N05010101FFFF0123;",
                              ":X1954522AN05010101FFFF0123;");
  // identify consumers 0==23; identified valid.
  SendPacketAndExpectResponse(":X19914001N05010101FFFF0023;",
                              ":X1954422AN05010101FFFF0023;");
  // 0==42, invalid
  ExpectPacket(":X195B422AN05010101FFFF0023;"); // we also get the true value
  SendPacketAndExpectResponse(":X19914001N05010101FFFF0042;",
                              ":X1954522AN05010101FFFF0042;");
}

TEST_F(ByteRangePTest, IdentifyGlobal) {
  // We have 10 bytes of 256 events each, which should be rounded up to 16*256
  // events, mask 0x0fff.
  ExpectPacket(":X1952422AN05010101FFFF0FFF;");
  SendPacket(":X19970001N;");
  WaitForEventThread();
}

TEST_F(AsyncNodeTest, ByteRangePCSync) {
  uint8_t consumer_storage[10] = {0,};
  uint8_t producer_storage[2] = {0x23, 0x42};
  uint8_t producer_storage_o[2] = {0x33, 0x52};
  ByteRangeEventP producer(node_, kEventBase, producer_storage, 2);
  ByteRangeEventP producer_o(node_, kEventBase + 512, producer_storage_o, 2);
  Wait();
  ExpectPacket(":X1952422AN05010101FFFF01FF;");
  ExpectPacket(":X1952422AN05010101FFFF0200;");
  SendPacket(":X19970001N;");

  Wait();
  ExpectAnyPacket();
  ByteRangeEventC consumer(node_, kEventBase, consumer_storage, 10);
  Wait();
  SendPacket(":X19970001N;");
  Wait();

  EXPECT_EQ(0x23, consumer_storage[0]);
  EXPECT_EQ(0x42, consumer_storage[1]);
  EXPECT_EQ(0x33, consumer_storage[2]);
  EXPECT_EQ(0x52, consumer_storage[3]);
}


}  // namespace NMRAnet

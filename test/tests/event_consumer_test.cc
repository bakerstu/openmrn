#include "utils/async_if_test_helper.hxx"

#include "nmranet/NMRAnetEventRegistry.hxx"
#include "nmranet/EventHandlerTemplates.hxx"

namespace NMRAnet {

static const uint64_t kEventBase = 0x05010101FFFF0000ULL;

class BitEventConsumerTest : public AsyncNodeTest {
 protected:
  BitEventConsumerTest()
      : storage_(0),
        bit_(node_, kEventBase, kEventBase + 1, &storage_, 1),
        consumer_(&bit_),
        bit2_(node_, kEventBase + 2, kEventBase + 3, &storage_, 2),
        consumer2_(&bit2_) {}

  uint8_t storage_;
  MemoryBit<uint8_t> bit_;
  BitEventConsumer consumer_;
  MemoryBit<uint8_t> bit2_;
  BitEventConsumer consumer2_;
};

TEST_F(BitEventConsumerTest, SimpleOnOff) {
  storage_ = 0;
  SendPacket(":X195B4001N05010101FFFF0000;");
  WaitForEventThread();
  EXPECT_EQ(1, storage_);

  SendPacket(":X195B4001N05010101FFFF0000;");
  WaitForEventThread();
  EXPECT_EQ(1, storage_);

  SendPacket(":X195B4001N05010101FFFF0001;");
  WaitForEventThread();
  EXPECT_EQ(0, storage_);

  SendPacket(":X195B4001N05010101FFFF0002;");
  WaitForEventThread();
  EXPECT_EQ(2, storage_);

  SendPacket(":X195B4001N05010101FFFF0002;");
  WaitForEventThread();
  EXPECT_EQ(2, storage_);

  SendPacket(":X195B4001N05010101FFFF0003;");
  WaitForEventThread();
  EXPECT_EQ(0, storage_);
}

TEST_F(BitEventConsumerTest, ProducerIdentifiedOnOff) {
  storage_ = 0;
  SendPacket(":X19544001N05010101FFFF0000;");
  WaitForEventThread();
  EXPECT_EQ(1, storage_);

  SendPacket(":X19544001N05010101FFFF0000;");
  WaitForEventThread();
  EXPECT_EQ(1, storage_);

  SendPacket(":X19544001N05010101FFFF0001;");
  WaitForEventThread();
  EXPECT_EQ(0, storage_);

  SendPacket(":X19544001N05010101FFFF0001;");
  WaitForEventThread();
  EXPECT_EQ(0, storage_);

  SendPacket(":X19545001N05010101FFFF0001;");
  WaitForEventThread();
  EXPECT_EQ(1, storage_);

  SendPacket(":X19545001N05010101FFFF0001;");
  WaitForEventThread();
  EXPECT_EQ(1, storage_);

  SendPacket(":X19545001N05010101FFFF0000;");
  WaitForEventThread();
  EXPECT_EQ(0, storage_);

  SendPacket(":X19545001N05010101FFFF0000;");
  WaitForEventThread();
  EXPECT_EQ(0, storage_);
}

TEST_F(BitEventConsumerTest, GlobalIdentify) {
  storage_ = 1;
  ExpectPacket(":X194C522AN05010101FFFF0001;");
  ExpectPacket(":X194C422AN05010101FFFF0000;");
  ExpectPacket(":X194C422AN05010101FFFF0003;");
  ExpectPacket(":X194C522AN05010101FFFF0002;");
  SendPacket(":X19970001N;");
  WaitForEventThread(); Mock::VerifyAndClear(&can_bus_);

  storage_ = 2;
  ExpectPacket(":X194C522AN05010101FFFF0000;");
  ExpectPacket(":X194C422AN05010101FFFF0001;");
  ExpectPacket(":X194C422AN05010101FFFF0002;");
  ExpectPacket(":X194C522AN05010101FFFF0003;");
  SendPacket(":X19970001N;");
  WaitForEventThread(); Mock::VerifyAndClear(&can_bus_);
}

TEST_F(BitEventConsumerTest, Query) {
  ExpectPacket(":X1991422AN05010101FFFF0000;");
  WriteHelper h;
  SyncNotifiable n;
  consumer_.SendQuery(&h, &n);
  n.WaitForNotification();
  WaitForEventThread();
}

TEST_F(BitEventConsumerTest, IdentifyConsumer) {
  storage_ = 1;
  SendPacketAndExpectResponse(":X198F4001N05010101FFFF0000;",
                              ":X194C422AN05010101FFFF0000;");
  SendPacketAndExpectResponse(":X198F4001N05010101FFFF0001;",
                              ":X194C522AN05010101FFFF0001;");
  SendPacketAndExpectResponse(":X198F4001N05010101FFFF0002;",
                              ":X194C522AN05010101FFFF0002;");
  SendPacketAndExpectResponse(":X198F4001N05010101FFFF0003;",
                              ":X194C422AN05010101FFFF0003;");
}
}  // namespace NMRAnet

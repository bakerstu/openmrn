#include "utils/async_if_test_helper.hxx"

#include "nmranet/NMRAnetEventRegistry.hxx"
#include "if/nmranet_if.h"
#include "nmranet/EventHandlerTemplates.hxx"

static const uint64_t kEventBase = 0x05010101FFFF0000ULL;

namespace NMRAnet {

class BitEventProducerTest : public AsyncNodeTest {
 protected:
  BitEventProducerTest()
      : storage_(0),
        bit_(node_, kEventBase, kEventBase + 1, &storage_, 1),
        producer_(&bit_),
        bit2_(node_, kEventBase + 2, kEventBase + 3, &storage_, 2),
        producer2_(&bit2_) {}

  uint8_t storage_;
  MemoryBit<uint8_t> bit_;
  BitEventProducer producer_;
  MemoryBit<uint8_t> bit2_;
  BitEventProducer producer2_;
};

TEST_F(BitEventProducerTest, SimpleOnOff) {
  storage_ = 0;
  ExpectPacket(":X195B422AN05010101FFFF0001;");
  producer_.Update(&event_write_helper1, EmptyNotifiable::DefaultInstance());
  WaitForEventThread(); Mock::VerifyAndClear(&can_bus_);

  storage_ = 1;
  ExpectPacket(":X195B422AN05010101FFFF0000;");
  producer_.Update(&event_write_helper1, EmptyNotifiable::DefaultInstance());
  WaitForEventThread(); Mock::VerifyAndClear(&can_bus_);

  ExpectPacket(":X195B422AN05010101FFFF0003;");
  producer2_.Update(&event_write_helper1, EmptyNotifiable::DefaultInstance());
  WaitForEventThread(); Mock::VerifyAndClear(&can_bus_);

  storage_ = 3;
  ExpectPacket(":X195B422AN05010101FFFF0002;");
  producer2_.Update(&event_write_helper1, EmptyNotifiable::DefaultInstance());
  WaitForEventThread(); Mock::VerifyAndClear(&can_bus_);
}

TEST_F(BitEventProducerTest, GlobalIdentify) {
  storage_ = 1;
  ExpectPacket(":X1954522AN05010101FFFF0001;");
  ExpectPacket(":X1954422AN05010101FFFF0000;");
  ExpectPacket(":X1954422AN05010101FFFF0003;");
  ExpectPacket(":X1954522AN05010101FFFF0002;");
  SendPacket(":X19970001N;");
  WaitForEventThread(); Mock::VerifyAndClear(&can_bus_);

  storage_ = 2;
  ExpectPacket(":X1954522AN05010101FFFF0000;");
  ExpectPacket(":X1954422AN05010101FFFF0001;");
  ExpectPacket(":X1954422AN05010101FFFF0002;");
  ExpectPacket(":X1954522AN05010101FFFF0003;");
  SendPacket(":X19970001N;");
  WaitForEventThread(); Mock::VerifyAndClear(&can_bus_);
}

TEST_F(BitEventProducerTest, IdentifyProducer) {
  storage_ = 1;
  SendPacketAndExpectResponse(":X19914001N05010101FFFF0000;",
                              ":X1954422AN05010101FFFF0000;");
  SendPacketAndExpectResponse(":X19914001N05010101FFFF0001;",
                              ":X1954522AN05010101FFFF0001;");
  SendPacketAndExpectResponse(":X19914001N05010101FFFF0002;",
                              ":X1954522AN05010101FFFF0002;");
  SendPacketAndExpectResponse(":X19914001N05010101FFFF0003;",
                              ":X1954422AN05010101FFFF0003;");
}

}  // namespace NMRAnet

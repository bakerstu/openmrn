#include "utils/if_test_helper.hxx"

#include "nmranet/NMRAnetEventRegistry.hxx"
#include "nmranet/EventHandlerTemplates.hxx"

namespace NMRAnet {

static const uint64_t kEventBase = 0x05010101FFFF0000ULL;

class BitEventPcTest : public IfTest {
 protected:
  BitEventPcTest()
      : storage_(0),
        bit_(node_, kEventBase, kEventBase + 1, &storage_, 1),
        pc_(&bit_),
        bit2_(node_, kEventBase + 2, kEventBase + 3, &storage_, 2),
        pc2_(&bit2_) {}

  uint8_t storage_;
  MemoryBit<uint8_t> bit_;
  BitEventPC pc_;
  MemoryBit<uint8_t> bit2_;
  BitEventPC pc2_;
};

TEST_F(BitEventPcTest, SimpleOnOff) {
  storage_ = 0;
  SendPacket(":X195B4000N05010101FFFF0000;");
  WaitForEventThread();
  EXPECT_EQ(1, storage_);

  SendPacket(":X195B4000N05010101FFFF0000;");
  WaitForEventThread();
  EXPECT_EQ(1, storage_);

  SendPacket(":X195B4000N05010101FFFF0001;");
  WaitForEventThread();
  EXPECT_EQ(0, storage_);

  SendPacket(":X195B4000N05010101FFFF0002;");
  WaitForEventThread();
  EXPECT_EQ(2, storage_);

  SendPacket(":X195B4000N05010101FFFF0002;");
  WaitForEventThread();
  EXPECT_EQ(2, storage_);

  SendPacket(":X195B4000N05010101FFFF0003;");
  WaitForEventThread();
  EXPECT_EQ(0, storage_);
}

TEST_F(BitEventPcTest, GlobalIdentify) {
  storage_ = 1;
  ExpectPacket(":X194C512DN05010101FFFF0001;");
  ExpectPacket(":X194C412DN05010101FFFF0000;");
  ExpectPacket(":X194C412DN05010101FFFF0003;");
  ExpectPacket(":X194C512DN05010101FFFF0002;");
  ExpectPacket(":X1954512DN05010101FFFF0001;");
  ExpectPacket(":X1954412DN05010101FFFF0000;");
  ExpectPacket(":X1954412DN05010101FFFF0003;");
  ExpectPacket(":X1954512DN05010101FFFF0002;");
  SendPacket(":X19970000N;");
  WaitForEventThread(); Mock::VerifyAndClear(&can_bus_);

  storage_ = 2;
  ExpectPacket(":X194C512DN05010101FFFF0000;");
  ExpectPacket(":X194C412DN05010101FFFF0001;");
  ExpectPacket(":X194C412DN05010101FFFF0002;");
  ExpectPacket(":X194C512DN05010101FFFF0003;");
  ExpectPacket(":X1954512DN05010101FFFF0000;");
  ExpectPacket(":X1954412DN05010101FFFF0001;");
  ExpectPacket(":X1954412DN05010101FFFF0002;");
  ExpectPacket(":X1954512DN05010101FFFF0003;");
  SendPacket(":X19970000N;");
  WaitForEventThread(); Mock::VerifyAndClear(&can_bus_);
}

TEST_F(BitEventPcTest, IdentifyPc) {
  storage_ = 1;
  SendPacketAndExpectResponse(":X198F4000N05010101FFFF0000;",
                              ":X194C412DN05010101FFFF0000;");
  SendPacketAndExpectResponse(":X198F4000N05010101FFFF0001;",
                              ":X194C512DN05010101FFFF0001;");
  SendPacketAndExpectResponse(":X198F4000N05010101FFFF0002;",
                              ":X194C512DN05010101FFFF0002;");
  SendPacketAndExpectResponse(":X198F4000N05010101FFFF0003;",
                              ":X194C412DN05010101FFFF0003;");
}

}  // namespace NMRAnet

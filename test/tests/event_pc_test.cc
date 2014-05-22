#include "utils/async_if_test_helper.hxx"

#include "nmranet/NMRAnetEventRegistry.hxx"
#include "nmranet/EventHandlerTemplates.hxx"

namespace nmranet {

static const uint64_t kEventBase = 0x05010101FFFF0000ULL;

class BitEventPcTest : public AsyncNodeTest {
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
  send_packet(":X195B4001N05010101FFFF0000;");
  wait_for_event_thread();
  EXPECT_EQ(1, storage_);

  send_packet(":X195B4001N05010101FFFF0000;");
  wait_for_event_thread();
  EXPECT_EQ(1, storage_);

  send_packet(":X195B4001N05010101FFFF0001;");
  wait_for_event_thread();
  EXPECT_EQ(0, storage_);

  send_packet(":X195B4001N05010101FFFF0002;");
  wait_for_event_thread();
  EXPECT_EQ(2, storage_);

  send_packet(":X195B4001N05010101FFFF0002;");
  wait_for_event_thread();
  EXPECT_EQ(2, storage_);

  send_packet(":X195B4001N05010101FFFF0003;");
  wait_for_event_thread();
  EXPECT_EQ(0, storage_);
}

TEST_F(BitEventPcTest, GlobalIdentify) {
  storage_ = 1;
  expect_packet(":X194C522AN05010101FFFF0001;");
  expect_packet(":X194C422AN05010101FFFF0000;");
  expect_packet(":X194C422AN05010101FFFF0003;");
  expect_packet(":X194C522AN05010101FFFF0002;");
  expect_packet(":X1954522AN05010101FFFF0001;");
  expect_packet(":X1954422AN05010101FFFF0000;");
  expect_packet(":X1954422AN05010101FFFF0003;");
  expect_packet(":X1954522AN05010101FFFF0002;");
  send_packet(":X19970001N;");
  wait_for_event_thread(); Mock::VerifyAndClear(&canBus_);

  storage_ = 2;
  expect_packet(":X194C522AN05010101FFFF0000;");
  expect_packet(":X194C422AN05010101FFFF0001;");
  expect_packet(":X194C422AN05010101FFFF0002;");
  expect_packet(":X194C522AN05010101FFFF0003;");
  expect_packet(":X1954522AN05010101FFFF0000;");
  expect_packet(":X1954422AN05010101FFFF0001;");
  expect_packet(":X1954422AN05010101FFFF0002;");
  expect_packet(":X1954522AN05010101FFFF0003;");
  send_packet(":X19970001N;");
  wait_for_event_thread(); Mock::VerifyAndClear(&canBus_);
}

TEST_F(BitEventPcTest, IdentifyPc) {
  storage_ = 1;
  send_packet_and_expect_response(":X198F4001N05010101FFFF0000;",
                              ":X194C422AN05010101FFFF0000;");
  send_packet_and_expect_response(":X198F4001N05010101FFFF0001;",
                              ":X194C522AN05010101FFFF0001;");
  send_packet_and_expect_response(":X198F4001N05010101FFFF0002;",
                              ":X194C522AN05010101FFFF0002;");
  send_packet_and_expect_response(":X198F4001N05010101FFFF0003;",
                              ":X194C422AN05010101FFFF0003;");
}

}  // namespace nmranet

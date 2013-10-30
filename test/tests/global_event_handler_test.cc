#include <stdio.h>
#include "gtest/gtest.h"
#include "os/os.h"
#include "nmranet/GlobalEventHandler.hxx"
#include "nmranet/NMRAnetEventRegistry.hxx"

extern void DecodeRange(EventReport* r);

class DecodeRangeTest : public testing::Test {
 protected:
  void ExpectDecode(uint64_t complex, uint64_t exp_event, uint64_t exp_mask) {
    EventReport r;
    r.event = complex;
    DecodeRange(&r);
    EXPECT_EQ(exp_event, r.event);
    EXPECT_EQ(exp_mask, r.mask);
  }

  uint64_t eventid_;
  uint64_t mask_;
};


TEST_F(DecodeRangeTest, TrivPositive) {
  ExpectDecode(0b1100, 0b1100, 0b0011);
}

TEST_F(DecodeRangeTest, TrivNegative) {
  ExpectDecode(0b110011, 0b110000, 0b0011);
}

TEST_F(DecodeRangeTest, SimplePositive) {
  ExpectDecode(0b1010101110000, 0b1010101110000, 0b1111);
}

TEST_F(DecodeRangeTest, SimpleNegative) {
  ExpectDecode(0b101010111000011111, 0b101010111000000000, 0b11111);
}

TEST_F(DecodeRangeTest, LongPositive) {
  ExpectDecode(0xfffffffffffffffeULL, 0xfffffffffffffffeULL, 1);
}

TEST_F(DecodeRangeTest, LongNegative) {
  ExpectDecode(0xffffffffffffff0fULL, 0xffffffffffffff00ULL, 0xf);
}

int appl_main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

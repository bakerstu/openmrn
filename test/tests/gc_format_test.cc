#include <string>

#include "gtest/gtest.h"
#include "os/os.h"

#include "utils/gc_format.h"
#include "nmranet_can.h"

using namespace std;

void ClearFrame(struct can_frame* frame) {
  memset(frame, 0, sizeof(*frame));
  SET_CAN_FRAME_EFF(*frame);
  CLR_CAN_FRAME_RTR(*frame);
  CLR_CAN_FRAME_ERR(*frame);
  frame->can_dlc = 0;
}


TEST(GCGenerateTest, ExtendedFrameNoData) {
  char buf[100];
  struct can_frame frame;
  ClearFrame(&frame);
  SET_CAN_FRAME_ID_EFF(frame, 0x195b4576);
  *gc_format_generate(&frame, buf, false) = '\0';
  EXPECT_EQ(string(":X195B4576N;"), buf);
}

TEST(GCGenerateTest, ExtendedFrameWithData) {
  char buf[100];
  struct can_frame frame;
  ClearFrame(&frame);
  for (int i = 0; i < 8; i++) {
    frame.data[i] = 0xf0 | i;
  }
  frame.can_dlc = 8;
  SET_CAN_FRAME_ID_EFF(frame, 0x195b4576);
  *gc_format_generate(&frame, buf, false) = '\0';

  EXPECT_EQ(string(":X195B4576NF0F1F2F3F4F5F6F7;"), buf);
}

TEST(GCGenerateTest, StdFrameWithData) {
  char buf[100];
  struct can_frame frame;
  ClearFrame(&frame);
  CLR_CAN_FRAME_EFF(frame);
  for (int i = 0; i < 8; i++) {
    frame.data[i] = 0xf0 | i;
  }
  frame.can_dlc = 8;
  SET_CAN_FRAME_ID(frame, 0x72d);
  *gc_format_generate(&frame, buf, false) = '\0';

  EXPECT_EQ(string(":S72DNF0F1F2F3F4F5F6F7;"), buf);
}

TEST(GCGenerateTest, StdFrameWithDataDouble) {
  char buf[100];
  struct can_frame frame;
  ClearFrame(&frame);
  CLR_CAN_FRAME_EFF(frame);
  for (int i = 0; i < 2; i++) {
    frame.data[i] = 0xf0 | i;
  }
  frame.can_dlc = 2;
  SET_CAN_FRAME_ID(frame, 0x72d);
  *gc_format_generate(&frame, buf, true) = '\0';

  EXPECT_EQ(string("!!SS7722DDNNFF00FF11;;"), buf);
}


TEST(GCParseTest, ExtFrameWithData) {
  struct can_frame frame;
  ASSERT_EQ(0, gc_format_parse("X195B4576NF0F1F2F3F4F5F6F7", &frame));
  EXPECT_TRUE(IS_CAN_FRAME_EFF(frame));
  EXPECT_EQ(0x195b4576UL, GET_CAN_FRAME_ID_EFF(frame));
  EXPECT_EQ(8, frame.can_dlc);
  for (int i = 0; i < 8; i++) {
    EXPECT_EQ(0xf0 | i, frame.data[i]);
  }
}

TEST(GCParseTest, ExtFrameWithNoData) {
  struct can_frame frame;
  ASSERT_EQ(0, gc_format_parse("X195B4576N", &frame));
  EXPECT_TRUE(IS_CAN_FRAME_EFF(frame));
  EXPECT_EQ(0x195b4576UL, GET_CAN_FRAME_ID_EFF(frame));
  EXPECT_EQ(0, frame.can_dlc);
}

TEST(GCParseTest, SFrameWithNoData) {
  struct can_frame frame;
  ASSERT_EQ(0, gc_format_parse("S721N", &frame));
  EXPECT_FALSE(IS_CAN_FRAME_EFF(frame));
  EXPECT_EQ(0x721UL, GET_CAN_FRAME_ID(frame));
  EXPECT_EQ(0, frame.can_dlc);
}

int appl_main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

#include <string>
#include <memory>
#include <assert.h>

using namespace std;

#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "os/os.h"

#include "utils/gc_pipe.hxx"
#include "utils/pipe.hxx"
#include "nmranet_can.h"

using testing::StrEq;
using testing::_;
using testing::ElementsAre;
//using testing::_;

void ClearFrame(struct can_frame* frame) {
  memset(frame, 0, sizeof(*frame));
  SET_CAN_FRAME_EFF(*frame);
  CLR_CAN_FRAME_RTR(*frame);
  CLR_CAN_FRAME_ERR(*frame);
  frame->can_dlc = 0;
}

class MockPipeMember : public PipeMember {
 public:
  MOCK_METHOD2(write, void(const void* buf, size_t count));
};

class GcPipeTest : public testing::Test {
 public:
  GcPipeTest()
      : gc_side_(1), can_side_(sizeof(struct can_frame)) {}


  void SaveGcPacket(const void* buf, size_t count) {
    string s(static_cast<const char*>(buf), count);
    saved_gc_data_.push_back(s);
  }

  void SaveCanFrame(const void* buf, size_t count) {
    const struct can_frame *f = static_cast<const struct can_frame*>(
        buf);
    while (count >= sizeof(struct can_frame)) {
      saved_can_data_.push_back(*f);
      count -= sizeof(struct can_frame);
      f++;
    }
    assert(count == 0);
  }


 protected:
  void AddChannel() {
    channel_.reset(GCAdapterBase::CreateGridConnectAdapter(
        &gc_side_, &can_side_, false));
  }

  Pipe gc_side_;
  Pipe can_side_;
  std::unique_ptr<GCAdapterBase> channel_;

  vector<string> saved_gc_data_;
  vector<struct can_frame> saved_can_data_;
};


TEST_F(GcPipeTest, CreateDestroy) {
  EXPECT_EQ(0U, gc_side_.size());
  EXPECT_EQ(0U, can_side_.size());
  AddChannel();
  EXPECT_EQ(1U, gc_side_.size());
  EXPECT_EQ(1U, can_side_.size());
  channel_.reset();
  EXPECT_EQ(0U, gc_side_.size());
  EXPECT_EQ(0U, can_side_.size());
}

string void_to_string(const void* d) {
  return string((const char*)d);
}

TEST_F(GcPipeTest, SendCanPacket) {
  AddChannel();
  struct can_frame f;
  ClearFrame(&f);
  SET_CAN_FRAME_EFF(f);
  SET_CAN_FRAME_ID_EFF(f, 0x195b4672);
  f.can_dlc = 3;
  f.data[0] = 0xf0; f.data[1] = 0xf1; f.data[2] = 0xf2;
  MockPipeMember mock;
  gc_side_.RegisterMember(&mock);
  EXPECT_CALL(mock, write(_, _)).WillRepeatedly(Invoke(this, &GcPipeTest::SaveGcPacket));
  //ResultOf(&to_string, StrEq()), 18U
  can_side_.WriteToAll(NULL, &f, sizeof(f));
  EXPECT_THAT(saved_gc_data_, ElementsAre(":X195B4672NF0F1F2;"));
}

TEST_F(GcPipeTest, SendTwoCanPacket) {
  AddChannel();
  struct can_frame f;
  ClearFrame(&f);
  SET_CAN_FRAME_EFF(f);
  SET_CAN_FRAME_ID_EFF(f, 0x195b4672);
  f.can_dlc = 3;
  f.data[0] = 0xf0; f.data[1] = 0xf1; f.data[2] = 0xf2;
  MockPipeMember mock;
  gc_side_.RegisterMember(&mock);
  EXPECT_CALL(mock, write(_, _)).WillRepeatedly(Invoke(this, &GcPipeTest::SaveGcPacket));
  //ResultOf(&to_string, StrEq()), 18U
  can_side_.WriteToAll(NULL, &f, sizeof(f));
  SET_CAN_FRAME_ID_EFF(f, 0x195b2672);
  f.data[0] = 0xd0;
  can_side_.WriteToAll(NULL, &f, sizeof(f));
  EXPECT_THAT(saved_gc_data_, ElementsAre(
      ":X195B4672NF0F1F2;", ":X195B2672ND0F1F2;"));
}

TEST_F(GcPipeTest, SendGcPacket) {
  AddChannel();
  string s = ":X195B4672NF0F1F2;";
  MockPipeMember mock;
  can_side_.RegisterMember(&mock);
  EXPECT_CALL(mock, write(_, _)).WillRepeatedly(Invoke(this, &GcPipeTest::SaveCanFrame));
  gc_side_.WriteToAll(NULL, s.data(), s.size());
  ASSERT_EQ(1U, saved_can_data_.size());
  EXPECT_EQ(0x195b4672U, GET_CAN_FRAME_ID_EFF(saved_can_data_[0]));
  ASSERT_EQ(3, saved_can_data_[0].can_dlc);
  EXPECT_EQ(0xf0U, saved_can_data_[0].data[0]);
  EXPECT_EQ(0xf1U, saved_can_data_[0].data[1]);
  EXPECT_EQ(0xf2U, saved_can_data_[0].data[2]);
}

TEST_F(GcPipeTest, SendGcPacketWithGarbage) {
  AddChannel();
  string s = "garbage\n:X195B4672NF0F1F2;\n";
  MockPipeMember mock;
  can_side_.RegisterMember(&mock);
  EXPECT_CALL(mock, write(_, _)).WillRepeatedly(Invoke(this, &GcPipeTest::SaveCanFrame));
  gc_side_.WriteToAll(NULL, s.data(), s.size());
  ASSERT_EQ(1U, saved_can_data_.size());
  EXPECT_EQ(0x195b4672U, GET_CAN_FRAME_ID_EFF(saved_can_data_[0]));
  ASSERT_EQ(3, saved_can_data_[0].can_dlc);
  EXPECT_EQ(0xf0U, saved_can_data_[0].data[0]);
  EXPECT_EQ(0xf1U, saved_can_data_[0].data[1]);
  EXPECT_EQ(0xf2U, saved_can_data_[0].data[2]);
}



int appl_main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

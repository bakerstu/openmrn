#include <string>
#include <memory>
#include <assert.h>

using namespace std;

#include "utils/test_main.hxx"
#include "utils/gc_pipe.hxx"
#include "utils/PipeFlow.hxx"
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

class MockPipeMember : public HubPort {
 public:
    MockPipeMember() : HubPort(&g_service) {}

    MOCK_METHOD2(write, void(const void* buf, size_t count));

    virtual Action entry() {
        write(message()->data()->data(), message()->data()->size());
        return release_and_exit();
    }
};

class MockCanPipeMember : public CanHubPort {
 public:
    MockCanPipeMember() : CanHubPort(&g_service) {}

    MOCK_METHOD1(write, void(const struct can_frame* frame));

    virtual Action entry() {
        write(message()->data());
        return release_and_exit();
    }
};

class GcPipeTest : public testing::Test {
 public:
  GcPipeTest()
      : gc_side_(&g_service), can_side_(&g_service) {}

  ~GcPipeTest() {
    wait();
  }

  void wait() {
      wait_for_main_executor();
  }

  void SaveGcPacket(const void* buf, size_t count) {
    string s(static_cast<const char*>(buf), count);
    saved_gc_data_.push_back(s);
  }

  void SaveCanFrame(const struct can_frame* frame) {
      saved_can_data_.push_back(*frame);
  }

 protected:
  void add_channel() {
    channel_.reset(GCAdapterBase::CreateGridConnectAdapter(
        &gc_side_, &can_side_, false));
  }

    void send_can_frame(const struct can_frame* frame) {
        Buffer<CanHubData> *buffer;
        mainBufferPool->alloc(&buffer);
        struct can_frame* out_frame = buffer->data(); 
        *out_frame = *frame;
    }

    void send_gc_packet(const string& s) {
        Buffer<HubData> *buffer;
        mainBufferPool->alloc(&buffer);
        string* out_data = buffer->data(); 
        *out_data = s;
    }

  HubFlow gc_side_;
  CanHubFlow can_side_;
  std::unique_ptr<GCAdapterBase> channel_;

  vector<string> saved_gc_data_;
  vector<struct can_frame> saved_can_data_;
};

TEST(HubTest, PointerMaskTest) {
#ifdef __x86_64
    EXPECT_EQ(0xFFFFFFFFFFFFFFFFUL, POINTER_MASK);
#else
    EXPECT_EQ(0xFFFFFFFFU, POINTER_MASK);
#endif
}


TEST_F(GcPipeTest, CreateDestroy) {
  EXPECT_EQ(0U, gc_side_.size());
  EXPECT_EQ(0U, can_side_.size());
  add_channel();
  EXPECT_EQ(1U, gc_side_.size());
  EXPECT_EQ(1U, can_side_.size());
  wait();
  channel_.reset();
  EXPECT_EQ(0U, gc_side_.size());
  EXPECT_EQ(0U, can_side_.size());
}

string void_to_string(const void* d) {
  return string((const char*)d);
}

TEST_F(GcPipeTest, SendCanPacket) {
  add_channel();
  struct can_frame f;
  ClearFrame(&f);
  SET_CAN_FRAME_EFF(f);
  SET_CAN_FRAME_ID_EFF(f, 0x195b4672);
  f.can_dlc = 3;
  f.data[0] = 0xf0; f.data[1] = 0xf1; f.data[2] = 0xf2;
  MockPipeMember mock;
  gc_side_.register_port(&mock);
  EXPECT_CALL(mock, write(_, _)).WillRepeatedly(Invoke(this, &GcPipeTest::SaveGcPacket));
  //ResultOf(&to_string, StrEq()), 18U
  send_can_frame(&f);
  EXPECT_THAT(saved_gc_data_, ElementsAre(":X195B4672NF0F1F2;"));
}

TEST_F(GcPipeTest, SendTwoCanPacket) {
  add_channel();
  struct can_frame f;
  ClearFrame(&f);
  SET_CAN_FRAME_EFF(f);
  SET_CAN_FRAME_ID_EFF(f, 0x195b4672);
  f.can_dlc = 3;
  f.data[0] = 0xf0; f.data[1] = 0xf1; f.data[2] = 0xf2;
  MockPipeMember mock;
  gc_side_.register_port(&mock);
  EXPECT_CALL(mock, write(_, _)).WillRepeatedly(Invoke(this, &GcPipeTest::SaveGcPacket));
  //ResultOf(&to_string, StrEq()), 18U
  send_can_frame(&f);
  SET_CAN_FRAME_ID_EFF(f, 0x195b2672);
  f.data[0] = 0xd0;
  send_can_frame(&f);
  EXPECT_THAT(saved_gc_data_, ElementsAre(
      ":X195B4672NF0F1F2;", ":X195B2672ND0F1F2;"));
}

TEST_F(GcPipeTest, SendGcPacket) {
  add_channel();
  string s = ":X195B4672NF0F1F2;";
  MockCanPipeMember mock;
  can_side_.register_port(&mock);
  EXPECT_CALL(mock, write(_)).WillRepeatedly(Invoke(this, &GcPipeTest::SaveCanFrame));
  send_gc_packet(s);
  wait();
  ASSERT_EQ(1U, saved_can_data_.size());
  EXPECT_EQ(0x195b4672U, GET_CAN_FRAME_ID_EFF(saved_can_data_[0]));
  ASSERT_EQ(3, saved_can_data_[0].can_dlc);
  EXPECT_EQ(0xf0U, saved_can_data_[0].data[0]);
  EXPECT_EQ(0xf1U, saved_can_data_[0].data[1]);
  EXPECT_EQ(0xf2U, saved_can_data_[0].data[2]);
}

TEST_F(GcPipeTest, SendGcPacketWithGarbage) {
  add_channel();
  string s = "garbage\n:X195B4672NF0F1F2;\n";
  MockCanPipeMember mock;
  can_side_.register_port(&mock);
  EXPECT_CALL(mock, write(_)).WillRepeatedly(Invoke(this, &GcPipeTest::SaveCanFrame));
  send_gc_packet(s);
  wait();
  ASSERT_EQ(1U, saved_can_data_.size());
  EXPECT_EQ(0x195b4672U, GET_CAN_FRAME_ID_EFF(saved_can_data_[0]));
  ASSERT_EQ(3, saved_can_data_[0].can_dlc);
  EXPECT_EQ(0xf0U, saved_can_data_[0].data[0]);
  EXPECT_EQ(0xf1U, saved_can_data_[0].data[1]);
  EXPECT_EQ(0xf2U, saved_can_data_[0].data[2]);
}

TEST_F(GcPipeTest, PartialPacket) {
  add_channel();
  string s = "garbage\n:X195B";
  string s2 = "4672NF";
  string s3 = "0F1F2;\n";
  MockCanPipeMember mock;
  can_side_.register_port(&mock);
  EXPECT_CALL(mock, write(_)).WillRepeatedly(Invoke(this, &GcPipeTest::SaveCanFrame));
  send_gc_packet(s);
  send_gc_packet(s2);
  send_gc_packet(s3);
  wait();
  ASSERT_EQ(1U, saved_can_data_.size());
  EXPECT_EQ(0x195b4672U, GET_CAN_FRAME_ID_EFF(saved_can_data_[0]));
  ASSERT_EQ(3, saved_can_data_[0].can_dlc);
  EXPECT_EQ(0xf0U, saved_can_data_[0].data[0]);
  EXPECT_EQ(0xf1U, saved_can_data_[0].data[1]);
  EXPECT_EQ(0xf2U, saved_can_data_[0].data[2]);
}

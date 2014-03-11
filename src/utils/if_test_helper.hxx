// Helper classes for writing unittests testing the entire stack. Allows to
// create a static node, send incoming messages (in gridconnect format) and set
// expectations on messages produced.
//
// Only include this file in unittests.

#ifndef _UTILS_IF_TEST_HELPER_HXX_
#define _UTILS_IF_TEST_HELPER_HXX_

#include <queue>

#include "nmranet_config.h"
#include "utils/macros.h"
#include "utils/test_main.hxx"
#include "utils/gc_pipe.hxx"
#include "utils/pipe.hxx"

#include "os/OS.hxx"
#include "nmranet/GlobalEventHandler.hxx"
#include "nmranet/NMRAnetIfCan.hxx"
#include "executor/executor.hxx"
#include "nmranet/NMRAnetNode.hxx"

using ::testing::_;
using ::testing::Return;
using ::testing::StrictMock;
using ::testing::NiceMock;
using ::testing::StrCaseEq;
using ::testing::Mock;

DEFINE_PIPE(gc_pipe0, 1);
GCAdapterBase* g_gc_adapter = nullptr;

namespace NMRAnet {

const char *Node::MANUFACTURER = "Stuart W. Baker";
const char *Node::HARDWARE_REV = "N/A";
const char *Node::SOFTWARE_REV = "0.1";

const size_t Datagram::POOL_SIZE = 10;
const size_t Datagram::THREAD_STACK_SIZE = 512;
const size_t Stream::CHANNELS_PER_NODE = 10;
const uint16_t Stream::MAX_BUFFER_SIZE = 512;

class FakeRead : public PipeMember {
 public:
  FakeRead(Pipe* parent)
      : is_done_(true), parent_(parent) {
    parent->RegisterMember(this);
  }

  ~FakeRead() {
    parent_->UnregisterMember(this);
  }

  ssize_t read(void* buf, size_t count) {
    HASSERT(count == sizeof(struct can_frame));
    struct can_frame* f = static_cast<struct can_frame*>(buf);
    while (1) {
      {
        OSMutexLock l(&lock_);
        if (!frames_.empty()) {
          *f = frames_.front();
          frames_.pop();
          return count;
        } else {
          is_done_ = true;
          done_notify_.post();
        }
      }
      notify_.wait();
    }
  }

  void WaitForDone() {
    while(1) {
      {
        OSMutexLock l(&lock_);
        if (is_done_) return;
      }
      done_notify_.wait();
    }
  }

 private:
  virtual void write(const void* buf, size_t count) {
    HASSERT(!(count % sizeof(struct can_frame)));
    size_t f_count = count / sizeof(struct can_frame);
    const struct can_frame* f = static_cast<const struct can_frame*>(buf);
    OSMutexLock l(&lock_);
    while (f_count) {
      is_done_ = false;
      frames_.push(*f);
      f++;
      f_count--;
      notify_.post();
    }
  }

  /// Holds the pending frames.
  std::queue<struct can_frame> frames_;
  /// Notified for incoming frames. Waited in the read method.
  OSSem notify_;
  /// Protects frames_ and is_done_.
  OSMutex lock_;
  /// Set to true when all incoming frames have been processed and the reading
  /// thread returned into a blocked state.
  bool is_done_;
  /// Notified when is_done is flipped to true.
  OSSem done_notify_;
  /// Parent pipe where we are registered.
  Pipe* parent_;
};

FakeRead g_fake_read(&can_pipe0);

static ssize_t mock_write(int fd, const void* buf, size_t n) {
  can_pipe0.WriteToAll(&g_fake_read, buf, n);
  return n;
}

static ssize_t mock_read(int, void* buf, size_t count) {
  return g_fake_read.read(buf, count);
}

class MockSend : public PipeMember {
 public:
  virtual void write(const void* buf, size_t count) {
    string s(static_cast<const char*>(buf), count);
    MWrite(s);
  }

  MOCK_METHOD1(MWrite, void(const string& s));
  //MOCK_METHOD2(write, void(const void* buf, size_t count));
};

class TestNode : public Node {
 public:
    TestNode(NodeID node_id, If *nmranet_if, const char *name)
        : Node(node_id, nmranet_if, name)
    {
    }
private:
    /** Process a Buffered message at the application level.
     * @param buffer message buffer to process
     */
    void process(Buffer *buffer)
    {
    }  
};


class IfTest : public testing::Test {
 public:
  static void SetUpTestCase() {
    int fd[2] = {0, 0};
    can_pipe0.AddVirtualDeviceToPipe("can_pipe", 2048, fd);
    g_gc_adapter =
        GCAdapterBase::CreateGridConnectAdapter(&gc_pipe0, &can_pipe0, false);

    nmranet_if_.reset(new IfCan(0x02010d000000ULL,
                                "/dev/null", mock_read, mock_write));

    static_node_ = CreateNewNode();
  }

 protected:
  IfTest() {
    node_ = static_node_;
    WaitForEventThread();
    gc_pipe0.RegisterMember(&can_bus_);
  }

  ~IfTest() {
    WaitForReadThread();
    gc_pipe0.UnregisterMember(&can_bus_);
  }

  void WaitForEventThread() {
    WaitForReadThread();
#ifdef CPP_EVENT_HANDLER
    while (GlobalEventFlow::instance->EventProcessingPending()) usleep(1000);
#endif
  }

  void WaitForReadThread() {
    g_fake_read.WaitForDone();
  }

  void ExpectPacket(const string& gc_packet) {
    EXPECT_CALL(can_bus_, MWrite(StrCaseEq(gc_packet)));
  }

  void SendPacket(const string& gc_packet) {
    gc_pipe0.WriteToAll(&can_bus_, gc_packet.data(), gc_packet.size());
  }

  void SendPacketAndExpectResponse(const string& pkt, const string& resp) {
    ExpectPacket(resp);
    SendPacket(pkt);
    WaitForEventThread();
    Mock::VerifyAndClear(&can_bus_);
  }

  Node* node_;
  NiceMock<MockSend> can_bus_;

 private:
  // A loopback interace that reads/writes to can_pipe0.
  static unique_ptr<NMRAnet::If> nmranet_if_;
  // A node that talks to the loopback interface.
  static Node* static_node_;

  void SetupStaticNode() {
    static Node* static_node = CreateNewNode();
    static_node_ = static_node;
    node_ = static_node;
  }

  static Node* CreateNewNode() {
    Node* node =
        new TestNode(0x02010d000003ULL, nmranet_if_.get(), "Test Node1");
    fprintf(stderr, "node_=%p\n", node);
    //nmranet_node_user_description(node, "Test Node2");
    //nmranet_node_initialized(node);
    node->initialized();
    /*
    os_thread_t thread;
    os_thread_create(&thread, "event_process_thread", 0, 2048,
    &AutomataTests::DispatchThread, node);*/
    return node;
  }
};

unique_ptr<If> IfTest::nmranet_if_ = NULL;
Node* IfTest::static_node_ = 0;

#ifdef CPP_EVENT_HANDLER
// We use this to pull in an object from the nmranet library to link.
//void* ign_link1 = (void*)&nmranet_identify_consumers;

ThreadExecutor g_executor("global_event", 0, 2000);
GlobalEventFlow g_event_flow(&g_executor, 10);
#endif

} // namespace NMRAnet

#endif  //_UTILS_IF_TEST_HELPER_HXX_

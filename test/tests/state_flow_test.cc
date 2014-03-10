#include "utils/test_main.hxx"

#include "executor/StateFlow.hxx"

class StateFlowTest : public testing::Test
{
public:
    StateFlowTest()
    {
    }
    ~StateFlowTest()
    {
        wait();
    }

protected:
    SyncNotifiable done_notifier_;

    void wait()
    {
        wait_for_main_executor();
    }
};

class TrivialFlow : public StateFlowBase {
public:
    TrivialFlow(Service* s) : StateFlowBase(s) {}
};

TEST_F(StateFlowTest, CreateDestroyBase) {
  TrivialFlow f(&g_service);
}

class SimpleTestFlow : public StateFlowBase {
public:
  SimpleTestFlow()
    : StateFlowBase(&g_service) {}

  void DoSomethingSimple(int foo, int* bar) {
    foo_ = foo;
    bar_ = bar;
    start_flow(STATE(CopyData));
  }

private:
  Action CopyData() {
    *bar_ = foo_;
    return exit();
  }

  int foo_;
  int* bar_;
};

TEST_F(StateFlowTest, NotifyEmptyFlow) {
  TrivialFlow f(&g_service);
  f.notify();
  wait();
}

TEST_F(StateFlowTest, SimpleFlow) {
  int a = 5, b = 0;
  SimpleTestFlow f;
  f.DoSomethingSimple(a, &b);
  wait();
  EXPECT_EQ(5, b);
}

struct Id {
    typedef uint32_t id_type;
    uint32_t id_;
    id_type id() { return id_; }
};

class QueueTestFlow : public StateFlow<Buffer<Id>, QList<3> > {
public:
    QueueTestFlow(vector<uint32_t>* seen_ids)
        : StateFlow(&g_service),
          seenIds_(seen_ids) {}
protected:
    virtual Action entry() {
        seenIds_->push_back(message()->data()->id_);
        return release_and_exit();
    }

private:
    vector<uint32_t>* seenIds_;
};

DynamicPool g_message_pool(Bucket::init(4, 8, 16, 32, 0));

class QueueTest : public StateFlowTest {
public:
    QueueTest() : flow_(&seenIds_) {
    }
protected:
    vector<uint32_t> seenIds_;
    QueueTestFlow flow_;
};

TEST_F(QueueTest, Nothing) {
    wait();
    EXPECT_TRUE(seenIds_.empty());
}

TEST_F(QueueTest, OneItem) {
    Buffer<Id>* first;
    g_message_pool.alloc(&first);
    first->data()->id_ = 42;
    wait();
    EXPECT_TRUE(seenIds_.empty());
    flow_.send(first);
    wait();
    ASSERT_EQ(1U, seenIds_.size());
    EXPECT_EQ(42U, seenIds_[0]);
}

TEST_F(QueueTest, ThreeItems) {
    Buffer<Id>* first;
    g_message_pool.alloc(&first);
    first->data()->id_ = 42;
    EXPECT_TRUE(seenIds_.empty());

    flow_.send(first);
    wait();
    ASSERT_EQ(1U, seenIds_.size());
    EXPECT_EQ(42U, seenIds_[0]);

    Buffer<Id>* second;
    g_message_pool.alloc(&second);
    second->data()->id_ = 43;
    flow_.send(second);

    Buffer<Id>* third;
    g_message_pool.alloc(&third);
    third->data()->id_ = 44;
    flow_.send(third);
    wait();
    ASSERT_EQ(3U, seenIds_.size());
    EXPECT_EQ(42U, seenIds_[0]);
    EXPECT_EQ(43U, seenIds_[1]);
    EXPECT_EQ(44U, seenIds_[2]);
}

/* Utility class to block an executor for a while.
 *
 * Usage: add an instance of BlockExecutor to the executor you want to block,
 * then call wait_for_blocked() and later release_block().
 */
class BlockExecutor : public Executable
{
public:
    virtual void run()
    {
        n_.notify();
        m_.wait_for_notification();
    }

    /** Blocks the current thread until the BlockExecutor manages to block the
    executor it was scheduled on. */
    void wait_for_blocked()
    {
        n_.wait_for_notification();
    }

    /** Releases the executor that was blocked. */
    void release_block()
    {
        m_.notify();
    }

private:
    SyncNotifiable n_;
    SyncNotifiable m_;
};

TEST_F(QueueTest, Priorities) {
    Buffer<Id>* m[3];
    g_message_pool.alloc(m + 0);
    g_message_pool.alloc(m + 1);
    g_message_pool.alloc(m + 2);
    m[0]->data()->id_ = 42;
    m[1]->data()->id_ = 43;
    m[2]->data()->id_ = 44;
    // We block the executor before sending off the messages to avoid this test
    // being flakey on multi-core processors.
    BlockExecutor b;
    g_executor.add(&b, 0);
    b.wait_for_blocked();

    flow_.send(m[0]);
    flow_.send(m[1], 1);
    flow_.send(m[2], 0);
    b.release_block();

    wait();
    // The order of the arrived messages should be reversed.
    ASSERT_EQ(3U, seenIds_.size());
    EXPECT_EQ(44U, seenIds_[0]);
    EXPECT_EQ(43U, seenIds_[1]);
    EXPECT_EQ(42U, seenIds_[2]);
}

/*TEST_F(StateFlowTest, CallDone) {
  SimpleTestFlow f(&done_notifier_);
  int a = 5, b = 0;
  f.DoSomethingSimple(a, &b);
  done_notifier_.WaitForNotification();
  EXPECT_EQ(5, b);
  EXPECT_TRUE(f.IsDone());
  }*/

/*
class SleeperFlow : public StateFlow {
public:
  explicit SleeperFlow(Notifiable* done)
    : StateFlow(&global_executor, done), r_(NULL) {}

  void SleepOnce(bool* r) {
    r_ = r;
    StartFlowAt((MemberFunction)&SleeperFlow::MSleepOnce);
  }

  void SleepNTimes(int n) {
    count_ = n;
    StartFlowAt((MemberFunction)&SleeperFlow::MSleepCount);
  }

  void ReptSleepNTimes(int n) {
    count_ = n;
    WakeUpRepeatedly(&sleep_data_, MSEC_TO_NSEC(3));
    StartFlowAt(ST(StartReptSleep));
  }

private:
  bool* r_;
  StateFlowAction MSleepOnce() {
    return Sleep(&sleep_data_, MSEC_TO_NSEC(3),
                 (MemberFunction)&SleeperFlow::MSleepDone);
  }

  StateFlowAction MSleepCount() {
    if (count_--) {
      return Sleep(&sleep_data_, MSEC_TO_NSEC(3),
                   (MemberFunction)&SleeperFlow::MSleepCount);
    } else {
      return CallImmediately((MemberFunction)&SleeperFlow::MSleepDone);
    }
  }

  StateFlowAction StartReptSleep() {
    return WaitForTimerWakeUpAndCall(
        &sleep_data_,
        (MemberFunction)&SleeperFlow::MReptSleepCount);
  }

  StateFlowAction MReptSleepCount() {
    if (!--count_) {
      StopTimer(&sleep_data_);
      return CallImmediately(ST(MSleepDone));
    }
    return WaitForNotification();
  }

  StateFlowAction MSleepDone() {
    if (r_) *r_ = true;
    return Exit();
  }

  int count_;
  SleepData sleep_data_;
};

TEST_F(StateFlowTest, SleepOnceTest) {
  SleeperFlow f(&done_notifier_);
  bool r = false;
  f.SleepOnce(&r);
  done_notifier_.WaitForNotification();
  EXPECT_TRUE(r);
  EXPECT_TRUE(f.IsDone());
}

TEST_F(StateFlowTest, SleepNTest) {
  SleeperFlow f(&done_notifier_);
  f.SleepNTimes(3);
  done_notifier_.WaitForNotification();
}

TEST_F(StateFlowTest, SleepReptTest) {
  SleeperFlow f(&done_notifier_);
  f.ReptSleepNTimes(3);
  done_notifier_.WaitForNotification();
}


class SubFlowCaller : public StateFlow {
public:
  explicit SubFlowCaller(Notifiable* done)
    : StateFlow(&global_executor, done) {}

  void RunFlowAndSubFlow(bool *r) {
    r_ = r;
    StartFlowAt(ST(Start));
  }

private:
  bool* r_;
  SleeperFlow* sleeper_flow_;

  StateFlowAction Start() {
    sleeper_flow_ = new SleeperFlow(this);
    sleeper_flow_->SleepOnce(r_);
    return CallFlow(sleeper_flow_, ST(ChildDone));
  }

  StateFlowAction ChildDone() {
    delete sleeper_flow_;
    return Exit();
  }
};

TEST_F(StateFlowTest, FlowWithChildTest) {
  SubFlowCaller f(&done_notifier_);
  bool r = false;
  f.RunFlowAndSubFlow(&r);
  done_notifier_.WaitForNotification();
  EXPECT_TRUE(r);
}


TEST(StaticStateFlowTest, SizeSmall) {
  EXPECT_EQ(4U, sizeof(QueueMember));
  EXPECT_EQ(40U, sizeof(StateFlow));
  }*/

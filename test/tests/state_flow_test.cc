#include "utils/test_main.hxx"

#include "executor/StateFlow.hxx"

static Service g_service(&g_executor);

class StateFlowTest : public testing::Test {
public:
  StateFlowTest() {}

protected:
  SyncNotifiable done_notifier_;

  void wait() {
    while (!g_executor.empty()) {
      usleep(100);
    }  
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

#include <stdio.h>
#include <unistd.h>
#include "gtest/gtest.h"
#include "os/os.h"

#include "executor/control_flow.hxx"

static ThreadExecutor global_executor("ex_thread", 0, 1024);

class ControlFlowTest : public testing::Test {
public:
  ControlFlowTest() {}

protected:
  SyncNotifiable done_notifier_;

  void RunToCompletion() {
    while (!global_executor.empty()) {
      usleep(100);
    }  
  }
};

TEST_F(ControlFlowTest, CreateDestroy) {
  ControlFlow f(&global_executor, EmptyNotifiable::DefaultInstance());
}

class SimpleTestFlow : public ControlFlow {
public:
  SimpleTestFlow()
    : ControlFlow(&global_executor, EmptyNotifiable::DefaultInstance()) {}

  explicit SimpleTestFlow(Notifiable* done)
    : ControlFlow(&global_executor, done) {}

  void DoSomethingSimple(int foo, int* bar) {
    foo_ = foo;
    bar_ = bar;
    StartFlowAt((MemberFunction)&SimpleTestFlow::CopyData);
  }

private:
  ControlFlowAction CopyData() {
    *bar_ = foo_;
    return Exit();
  }

  int foo_;
  int* bar_;
};

TEST_F(ControlFlowTest, NotifyEmptyFlow) {
  ControlFlow f(&global_executor, EmptyNotifiable::DefaultInstance());
  f.Notify();
  RunToCompletion();
}

TEST_F(ControlFlowTest, SimpleFlow) {
  int a = 5, b = 0;
  SimpleTestFlow f;
  f.DoSomethingSimple(a, &b);
  RunToCompletion();
  EXPECT_EQ(5, b);
}

TEST_F(ControlFlowTest, CallDone) {
  SimpleTestFlow f(&done_notifier_);
  int a = 5, b = 0;
  f.DoSomethingSimple(a, &b);
  done_notifier_.WaitForNotification();
  EXPECT_EQ(5, b);
  EXPECT_TRUE(f.IsDone());
}

class SleeperFlow : public ControlFlow {
public:
  explicit SleeperFlow(Notifiable* done)
    : ControlFlow(&global_executor, done), r_(NULL) {}

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
    StartFlowAt(&SleeperFlow::StartReptSleep);
  }

private:
  bool* r_;
  ControlFlowAction MSleepOnce() {
    return Sleep(&sleep_data_, MSEC_TO_NSEC(3),
                 (MemberFunction)&SleeperFlow::MSleepDone);
  }

  ControlFlowAction MSleepCount() {
    if (count_--) {
      return Sleep(&sleep_data_, MSEC_TO_NSEC(3),
                   (MemberFunction)&SleeperFlow::MSleepCount);
    } else {
      return CallImmediately((MemberFunction)&SleeperFlow::MSleepDone);
    }
  }

  ControlFlowAction StartReptSleep() {
    return WaitForTimerWakeUpAndCall(
        &sleep_data_,
        (MemberFunction)&SleeperFlow::MReptSleepCount);
  }

  ControlFlowAction MReptSleepCount() {
    if (!--count_) {
      StopTimer(&sleep_data_);
      return CallImmediately(&SleeperFlow::MSleepDone);
    }
    return WaitForNotification();
  }

  ControlFlowAction MSleepDone() {
    if (r_) *r_ = true;
    return Exit();
  }

  int count_;
  SleepData sleep_data_;
};

TEST_F(ControlFlowTest, SleepOnceTest) {
  SleeperFlow f(&done_notifier_);
  bool r = false;
  f.SleepOnce(&r);
  done_notifier_.WaitForNotification();
  EXPECT_TRUE(r);
  EXPECT_TRUE(f.IsDone());
}

TEST_F(ControlFlowTest, SleepNTest) {
  SleeperFlow f(&done_notifier_);
  f.SleepNTimes(3);
  done_notifier_.WaitForNotification();
}

TEST_F(ControlFlowTest, SleepReptTest) {
  SleeperFlow f(&done_notifier_);
  f.ReptSleepNTimes(3);
  done_notifier_.WaitForNotification();
}


class SubFlowCaller : public ControlFlow {
public:
  explicit SubFlowCaller(Notifiable* done)
    : ControlFlow(&global_executor, done) {}

  void RunFlowAndSubFlow(bool *r) {
    r_ = r;
    StartFlowAt(&SubFlowCaller::Start);
  }

private:
  bool* r_;
  SleeperFlow* sleeper_flow_;

  ControlFlowAction Start() {
    sleeper_flow_ = new SleeperFlow(this);
    sleeper_flow_->SleepOnce(r_);
    return CallFlow(sleeper_flow_, &SubFlowCaller::ChildDone);
  }

  ControlFlowAction ChildDone() {
    delete sleeper_flow_;
    return Exit();
  }
};

TEST_F(ControlFlowTest, FlowWithChildTest) {
  SubFlowCaller f(&done_notifier_);
  bool r = false;
  f.RunFlowAndSubFlow(&r);
  done_notifier_.WaitForNotification();
  EXPECT_TRUE(r);
}

int appl_main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

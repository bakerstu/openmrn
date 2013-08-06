#include <stdio.h>
#include <unistd.h>
#include "gtest/gtest.h"
#include "os/os.h"


#include "executor/control_flow.hxx"

static ThreadExecutor global_executor("ex_thread", 0, 1024);

static void RunToCompletion() {
  while (!global_executor.empty()) {
    usleep(100);
  }
}

TEST(ControlFlowTest, CreateDestroy) {
  ControlFlow f(&global_executor);
}

class SimpleTestFlow : public ControlFlow {
public:
  SimpleTestFlow()
    : ControlFlow(&global_executor) {}

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

TEST(ControlFlowTest, NotifyEmptyFlow) {
  ControlFlow f(&global_executor);
  f.Notify();
  RunToCompletion();
}

TEST(ControlFlowTest, SimpleFlow) {
  int a = 5, b = 0;
  SimpleTestFlow f;
  f.DoSomethingSimple(a, &b);
  RunToCompletion();
  EXPECT_EQ(5, b);
}

int appl_main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

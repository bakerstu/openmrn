#include <stdio.h>

#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "os/os.h"

#include "executor/executor.hxx"

static ThreadExecutor global_executor("ex_thread", 0, 1024);

static void RunToCompletion() {
  while (!global_executor.empty()) {
    usleep(100);
  }
}


TEST(ExecutorTest, IsEmpty) {
  EXPECT_TRUE(global_executor.empty());
}


class MockRunnable : public Executable {
 public:
  MOCK_METHOD0(Run, void());
};


TEST(ExecutorTest, RunSingle) {
  EXPECT_TRUE(global_executor.empty());
  MockRunnable r;
  EXPECT_CALL(r, Run()).Times(1);
  global_executor.Add(&r);
  // This is a race condition -- it is possible that the executor thread gets
  // to this runnable before we validate the following expectation.
  EXPECT_FALSE(global_executor.empty());
  RunToCompletion();
  EXPECT_TRUE(global_executor.empty());
}



int appl_main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

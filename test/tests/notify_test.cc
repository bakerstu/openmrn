#include <stdio.h>
#include "gtest/gtest.h"
#include "os/os.h"

#include "executor/notifiable.hxx"

class TestNotifiable : public Notifiable {
 public:
  TestNotifiable() : done_(false) {}

  bool IsDone() { return done_; }

  virtual void Notify() {
    HASSERT(!done_);
    done_ = true;
  }

 private:
  bool done_;
};

class BarrierTests : public testing::Test {
 protected:
  BarrierTests()
      : barrier_(&parent_) {}

  TestNotifiable parent_;
  BarrierNotifiable barrier_;
};

TEST_F(BarrierTests, NoChild) {
  EXPECT_FALSE(parent_.IsDone());
  barrier_.MaybeDone();
  EXPECT_TRUE(parent_.IsDone());
}

TEST_F(BarrierTests, OneChild) {
  EXPECT_FALSE(parent_.IsDone());
  Notifiable* d1 = barrier_.NewChild();
  barrier_.MaybeDone();
  EXPECT_FALSE(parent_.IsDone());
  d1->Notify();
  EXPECT_TRUE(parent_.IsDone());
}

TEST_F(BarrierTests, TwoChild) {
  EXPECT_FALSE(parent_.IsDone());
  Notifiable* d1 = barrier_.NewChild();
  Notifiable* d2 = barrier_.NewChild();
  barrier_.MaybeDone();
  EXPECT_FALSE(parent_.IsDone());
  d1->Notify();
  EXPECT_FALSE(parent_.IsDone());
  d2->Notify();
  EXPECT_TRUE(parent_.IsDone());
}

TEST(BarrierDies, WhenFailedToCallMaybeDone) {
  EXPECT_DEATH({
      TestNotifiable parent;
      BarrierNotifiable barrier(&parent);
      Notifiable* c = barrier.NewChild();
      c->Notify();
    }, "!count_");
}

TEST(CrashDies, EveryTime) {
  EXPECT_DEATH({
      Notifiable* n = CrashNotifiable::DefaultInstance();
      n->Notify();
    }, "foo");
}

TEST(DummyTest, Success) {
  int i = 34;
  EXPECT_EQ(34, i);
}

int appl_main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

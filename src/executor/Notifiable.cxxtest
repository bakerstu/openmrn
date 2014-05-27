#include <stdio.h>
#include "gtest/gtest.h"
#include "os/os.h"

#include "executor/Notifiable.hxx"

class TestNotifiable : public Notifiable {
 public:
  TestNotifiable() : done_(false) {}

  bool is_done() { return done_; }

  virtual void notify() {
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
  EXPECT_FALSE(parent_.is_done());
  barrier_.maybe_done();
  EXPECT_TRUE(parent_.is_done());
}

TEST_F(BarrierTests, OneChild) {
  EXPECT_FALSE(parent_.is_done());
  Notifiable* d1 = barrier_.new_child();
  barrier_.maybe_done();
  EXPECT_FALSE(parent_.is_done());
  d1->notify();
  EXPECT_TRUE(parent_.is_done());
}

TEST_F(BarrierTests, TwoChild) {
  EXPECT_FALSE(parent_.is_done());
  Notifiable* d1 = barrier_.new_child();
  Notifiable* d2 = barrier_.new_child();
  barrier_.maybe_done();
  EXPECT_FALSE(parent_.is_done());
  d1->notify();
  EXPECT_FALSE(parent_.is_done());
  d2->notify();
  EXPECT_TRUE(parent_.is_done());
}

TEST(BarrierDies, WhenFailedToCallmaybe_done) {
  EXPECT_DEATH({
      TestNotifiable parent;
      BarrierNotifiable barrier(&parent);
      Notifiable* c = barrier.new_child();
      c->notify();
    }, "!count_");
}

TEST(CrashDies, EveryTime) {
  EXPECT_DEATH({
      Notifiable* n = CrashNotifiable::DefaultInstance();
      n->notify();
    }, "Called CrashNotifiable");
}

TEST(DummyTest, Success) {
  int i = 34;
  EXPECT_EQ(34, i);
}

TEST(AutoNotify, Call) {
    TestNotifiable n;
    {
        AutoNotify auton(&n);
        EXPECT_FALSE(n.is_done());
    }
    EXPECT_TRUE(n.is_done());
}

TEST(AutoNotify, Transfer) {
    TestNotifiable n;
    {
        AutoNotify auton(&n);
        EXPECT_FALSE(n.is_done());
        EXPECT_EQ(&n, auton.Transfer());
    }
    EXPECT_FALSE(n.is_done());
}

int appl_main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

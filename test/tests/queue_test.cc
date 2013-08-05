#include <stdio.h>
#include "gtest/gtest.h"
#include "os/os.h"


#include "executor/queue.hxx"


TEST(QueueTest, CreateDestroy) {
  Queue q;
  EXPECT_TRUE(q.empty());
  EXPECT_FALSE(q.Pop());
}

TEST(QueueTest, SingleElement) {
  Queue q;
  QueueMember foo;
  EXPECT_TRUE(q.empty());
  q.Push(&foo);
  EXPECT_FALSE(q.empty());
  EXPECT_EQ(&foo, q.Pop());
  EXPECT_TRUE(q.empty());

  // another run
  q.Push(&foo);
  EXPECT_FALSE(q.empty());
  EXPECT_EQ(&foo, q.Pop());
  EXPECT_TRUE(q.empty());
}

TEST(QueueTest, MixedWorkload) {
  Queue q;
  QueueMember foo, bar, baz;
  EXPECT_TRUE(q.empty());
  q.Push(&foo);
  q.Push(&bar);
  EXPECT_EQ(&foo, q.Pop());
  EXPECT_FALSE(q.empty());

  q.Push(&baz);
  EXPECT_FALSE(q.empty());
  EXPECT_EQ(&bar, q.Pop());
  EXPECT_FALSE(q.empty());
  q.Push(&foo);
  q.Push(&bar);
  EXPECT_EQ(&baz, q.Pop());
  EXPECT_EQ(&foo, q.Pop());
  EXPECT_FALSE(q.empty());
  EXPECT_EQ(&bar, q.Pop());
  EXPECT_TRUE(q.empty());
}

TEST(QueueTest, DieAddNull) {
  Queue q;
  EXPECT_DEATH({
      q.Push(NULL);
    }, "entry");
}

TEST(QueueTest, DieAddTwice) {
  Queue q;
  QueueMember foo;
  q.Push(&foo);
  EXPECT_DEATH({
      q.Push(&foo);
    }, "entry != tail_");
}

TEST(QueueTest, DieAddTwice2) {
  Queue q;
  QueueMember foo;
  QueueMember bar;
  q.Push(&foo);
  q.Push(&bar);
  EXPECT_DEATH({
      q.Push(&foo);
    }, "next_ ==");
}




int appl_main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

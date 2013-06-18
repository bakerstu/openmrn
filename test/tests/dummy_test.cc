#include <stdio.h>
#include "gtest/gtest.h"


TEST(DummyTest, Success) {
  int i = 34;
  EXPECT_EQ(34, i);
}

#include <stdio.h>
#include "gtest/gtest.h"
#include "os/os.h"


TEST(DummyTest, Success) {
  int i = 34;
  EXPECT_EQ(34, i);
}

int appl_main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

#define NDEBUG
#include <stdio.h>
#include "gtest/gtest.h"
#include "os/os.h"

#include "utils/macros.h"


TEST(AssertTest, AssertSuccess) {
    HASSERT(true);
}

TEST(AssertTest, AssertFailed) {
    EXPECT_DEATH({
            HASSERT(false);
    }, "false");
}

TEST(AssertTest, DAssertSuccessNonDebug) {
    DASSERT(true);
}

TEST(AssertTest, DAssertFailedNonDebug) {
    DASSERT(false);
}

int appl_main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

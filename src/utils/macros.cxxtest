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

#ifdef NDEBUG
#error This test requires NDEBUG to be unset.
#endif
TEST(AssertTest, DAssertSuccess) {
    DASSERT(true);
}

TEST(AssertTest, DAssertFailed) {
    EXPECT_DEATH({
            DASSERT(false);
    }, "false");
}

TEST(CrashTest, Crashed) {
    EXPECT_DEATH({
        DIE("deeeeeaad");
    }, "deeeeeaad");
}

int appl_main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

#include <stdio.h>
#include "gtest/gtest.h"
#include "os/os.h"


#include "executor/control_flow.hxx"



int appl_main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

#include "gtest/gtest.h"

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);

  // TODO: Have a different test runner that dynamically loads unit test objects and executes them

  int ret = RUN_ALL_TESTS();

  return ret;
}

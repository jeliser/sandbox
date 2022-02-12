// system includes
#include "gtest/gtest.h"

#include <sample.h>

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}

#include "aos/die.h"

#include "gtest/gtest.h"

namespace aos::testing {

TEST(DieDeathTest, Works) {
  EXPECT_EXIT(Die("str=%s num=%d\n", "hi", 5),
              ::testing::KilledBySignal(SIGABRT), ".*str=hi num=5\n");
}

}  // namespace aos::testing

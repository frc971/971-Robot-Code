#include "motors/seems_reasonable/spring.h"

#include "gtest/gtest.h"

namespace motors {
namespace seems_reasonable {
namespace testing {

// Tests that NextGoal always returns the next goal.
TEST(GoalTest, TestNextGoal) {
  EXPECT_NEAR(1.0, NextGoal(0.0, 1.0), 1e-6);

  EXPECT_NEAR(2.0 * M_PI + 1.0, NextGoal(1.1, 1.0), 1e-6);

  EXPECT_NEAR(6.0 * M_PI + 1.0, NextGoal(1.1 + 4.0 * M_PI, 1.0), 1e-6);

  EXPECT_NEAR(2.0 * M_PI + 1.0, NextGoal(1.0, 1.0), 1e-6);

  EXPECT_NEAR(2.0 * M_PI + 6.0, NextGoal(6.1, 6.0), 1e-6);
}

// Tests that PreviousGoal always returns the previous goal.
TEST(GoalTest, TestPreviousGoal) {
  EXPECT_NEAR(-2.0 * M_PI + 1.0, PreviousGoal(0.0, 1.0), 1e-6);

  EXPECT_NEAR(1.0, PreviousGoal(1.1, 1.0), 1e-6);

  EXPECT_NEAR(4.0 * M_PI + 1.0, PreviousGoal(1.1 + 4.0 * M_PI, 1.0), 1e-6);

  EXPECT_NEAR(-2.0 * M_PI + 1.0, PreviousGoal(1.0, 1.0), 1e-6);
}

}  // namespace testing
}  // namespace seems_reasonable
}  // namespace motors

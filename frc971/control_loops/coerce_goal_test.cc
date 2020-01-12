#include "frc971/control_loops/coerce_goal.h"

#include <unistd.h>

#include "gtest/gtest.h"

namespace frc971 {
namespace control_loops {
namespace {

::aos::controls::HVPolytope<2, 4, 4, double> poly() {
  return ::aos::controls::HVPolytope<2, 4, 4, double>(
      (Eigen::Matrix<double, 4, 2>() << /*[[*/ 1, 0 /*]*/,
       /*[*/ -1, 0 /*]*/,
       /*[*/ 0, 1 /*]*/,
       /*[*/ 0, -1 /*]]*/)
          .finished(),
      (Eigen::Matrix<double, 4, 1>() << /*[[*/ 12 /*]*/,
       /*[*/ 12 /*]*/,
       /*[*/ 12 /*]*/,
       /*[*/ 12 /*]*/)
          .finished(),
      (Eigen::Matrix<double, 2, 4>() << /*[[*/ 12, 12, -12, -12 /*]*/,
       /*[*/ -12, 12, 12, -12 /*]*/)
          .finished());
}

TEST(CoerceGoalTest, WithinRegion) {
  const auto upoly = poly();
  Eigen::Matrix<double, 1, 2> k;
  k << 2, 2;

  Eigen::Matrix<double, 2, 1> goal;
  goal << -2, 2;

  auto result = CoerceGoal<double>(upoly, k, 0, goal);

  EXPECT_EQ(result(0, 0), -2);
  EXPECT_EQ(result(1, 0), 2);
}

TEST(CoerceGoalTest, VerticalLine) {
  const auto upoly = poly();
  Eigen::Matrix<double, 1, 2> k;
  k << 2, 0;

  Eigen::Matrix<double, 2, 1> goal;
  goal << 0, 13;

  auto result = CoerceGoal<double>(upoly, k, 0, goal);

  EXPECT_EQ(result(0, 0), 0);
  EXPECT_EQ(result(1, 0), 12);
}

TEST(CoerceGoalTest, HorizontalLine) {
  const auto upoly = poly();
  Eigen::Matrix<double, 1, 2> k;
  k << 0, 2;

  Eigen::Matrix<double, 2, 1> goal;
  goal << 13, 2;

  auto result = CoerceGoal<double>(upoly, k, 0, goal);

  EXPECT_EQ(result(0, 0), 12);
  EXPECT_EQ(result(1, 0), 0);
}

}  // namespace
}  // namespace control_loops
}  // namespace frc971

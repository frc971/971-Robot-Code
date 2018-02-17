#include "y2018/control_loops/superstructure/arm/trajectory.h"

#include "gtest/gtest.h"

namespace y2018 {
namespace control_loops {
namespace superstructure {
namespace arm {
namespace testing {

// Tests that we can pull out values along the path.
TEST(TrajectoryTest, Theta) {
  Path p({{{0.0, 0.0, 1.0, 0.0, 0.0, 0.0}},
          {{1.0, 0.0, 1.0, 0.0, 0.0, 0.0}},
          {{2.0, 0.0, 1.0, 0.0, 0.0, 0.0}},
          {{3.0, 0.0, 1.0, 0.0, 0.0, 0.0}},
          {{4.0, 0.0, 1.0, 0.0, 0.0, 0.0}}});

  EXPECT_TRUE(p.Theta(0.5).isApprox(
      (::Eigen::Matrix<double, 2, 1>() << 0.5, 0.0).finished()));
  EXPECT_TRUE(p.Omega(0.5).isApprox(
      (::Eigen::Matrix<double, 2, 1>() << 1.0, 0.0).finished()));
  EXPECT_TRUE(p.Alpha(0.5).isApprox(
      (::Eigen::Matrix<double, 2, 1>() << 0.0, 0.0).finished()));

  EXPECT_TRUE(p.Theta(1.5).isApprox(
      (::Eigen::Matrix<double, 2, 1>() << 1.5, 0.0).finished()));

  for (double d = 0.0; d <= 4.0; d += 0.01) {
    EXPECT_TRUE(p.Theta(d).isApprox(
        (::Eigen::Matrix<double, 2, 1>() << d, 0.0).finished()));
  }
}

// Tests that a path with an even number of points works as expected.
TEST(TrajectoryTest, EvenPointNumbers) {
  Path p({{{0.0, 0.0, 1.0, 0.0, 0.0, 0.0}},
          {{1.0, 0.0, 1.0, 0.0, 0.0, 0.0}},
          {{2.0, 0.0, 1.0, 0.0, 0.0, 0.0}},
          {{3.0, 0.0, 1.0, 0.0, 0.0, 0.0}}});
  EXPECT_TRUE(p.Theta(1.5).isApprox(
      (::Eigen::Matrix<double, 2, 1>() << 1.5, 0.0).finished()));

  for (double d = 0.0; d <= 3.0; d += 0.01) {
    EXPECT_TRUE(p.Theta(d).isApprox(
        (::Eigen::Matrix<double, 2, 1>() << d, 0.0).finished()));
  }
}

// Tests that out of range points work as expected.
TEST(TrajectoryTest, OutOfBounds) {
  Path p({{{0.0, 0.0, 1.0, 0.0, 0.0, 0.0}},
          {{1.0, 0.0, 1.0, 0.0, 0.0, 0.0}},
          {{2.0, 0.0, 1.0, 0.0, 0.0, 0.0}},
          {{3.0, 0.0, 1.0, 0.0, 0.0, 0.0}}});
  EXPECT_TRUE(p.Theta(-1.0).isApprox(
      (::Eigen::Matrix<double, 2, 1>() << 0.0, 0.0).finished()));
  EXPECT_TRUE(p.Theta(10.0).isApprox(
      (::Eigen::Matrix<double, 2, 1>() << 3.0, 0.0).finished()));
}

}  // namespace testing
}  // namespace arm
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2018

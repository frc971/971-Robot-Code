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

// Tests a path where picking the right point to interpolate for matters.
TEST(TrajectoryTest, HarderPath) {
  Path p({{{0.0, 0.0, 1.0, 0.0, 0.0, 0.0}},
          {{1.0, 0.0, 1.0, 0.0, 0.0, 0.0}},
          {{1.0, 1.0, 0.0, 1.0, 0.0, 0.0}}});

  EXPECT_TRUE(p.Theta(0.5).isApprox(
      (::Eigen::Matrix<double, 2, 1>() << 0.5, 0.0).finished()));

  EXPECT_TRUE(p.Theta(1.5).isApprox(
      (::Eigen::Matrix<double, 2, 1>() << 1.0, 0.5).finished()));
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


// Tests that we can compute the indices of the plan for a given distance correctly.
TEST(TrajectoryTest, IndicesForDistanceTest) {
  // Start with a stupid simple plan.
  Path p({{{0.0, 0.0, 1.0, 0.0, 0.0, 0.0}},
          {{1.0, 0.0, 1.0, 0.0, 0.0, 0.0}},
          {{2.0, 0.0, 1.0, 0.0, 0.0, 0.0}},
          {{3.0, 0.0, 1.0, 0.0, 0.0, 0.0}}});
  Trajectory t(&p, 0.1);

  // 0 - 3.0 every 0.1 should be 31 points.
  EXPECT_EQ(t.num_plan_points(), 31);

  // Verify that something centered in a grid cell returns the points on either side.
  EXPECT_EQ(::std::make_pair(static_cast<size_t>(0), static_cast<size_t>(1)),
            t.IndicesForDistance(0.05));
  EXPECT_EQ(::std::make_pair(static_cast<size_t>(1), static_cast<size_t>(2)),
            t.IndicesForDistance(0.15));

  // Verify that something on an edge returns the expected result.
  EXPECT_EQ(::std::make_pair(static_cast<size_t>(1), static_cast<size_t>(2)),
            t.IndicesForDistance(0.1));

  // Verify what small deviations result.
  EXPECT_EQ(::std::make_pair(static_cast<size_t>(1), static_cast<size_t>(2)),
            t.IndicesForDistance(0.1001));
  EXPECT_EQ(::std::make_pair(static_cast<size_t>(0), static_cast<size_t>(1)),
            t.IndicesForDistance(0.0999));

  // Verify that blowing past the ends works.
  EXPECT_EQ(::std::make_pair(static_cast<size_t>(29), static_cast<size_t>(30)),
            t.IndicesForDistance(4.0));
  EXPECT_EQ(::std::make_pair(static_cast<size_t>(0), static_cast<size_t>(1)),
            t.IndicesForDistance(-0.1));

  // Verify that the index to distance calculation also works.
  EXPECT_EQ(0.0, t.DistanceForIndex(0));
  EXPECT_EQ(0.1, t.DistanceForIndex(1));
  EXPECT_EQ(3.0, t.DistanceForIndex(30));
}

}  // namespace testing
}  // namespace arm
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2018

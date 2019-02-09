#include "frc971/control_loops/pose.h"

#include "gtest/gtest.h"

namespace frc971 {
namespace control_loops {
namespace testing {

// Test that basic accessors on an individual Pose object work as expected.
TEST(PoseTest, BasicPoseTest) {
  // Provide a basic Pose with non-zero components for everything.
  Pose pose({1, 1, 0.5}, 0.5);
  // The xy_norm should just be based on the x/y positions, not the Z; hence
  // sqrt(2) rather than sqrt(1^2 + 1^2 + 0.5^2).
  EXPECT_DOUBLE_EQ(::std::sqrt(2.0), pose.xy_norm());
  // Similarly, heading should just be atan2(y, x).
  EXPECT_DOUBLE_EQ(M_PI / 4.0, pose.heading());
  // Global and relative poses should be the same since we did not construct
  // this off of a separate Pose.
  EXPECT_EQ(1.0, pose.rel_pos().x());
  EXPECT_EQ(1.0, pose.rel_pos().y());
  EXPECT_EQ(0.5, pose.rel_pos().z());

  EXPECT_EQ(1.0, pose.abs_pos().x());
  EXPECT_EQ(1.0, pose.abs_pos().y());
  EXPECT_EQ(1.0, pose.abs_xy().x());
  EXPECT_EQ(1.0, pose.abs_xy().y());
  EXPECT_EQ(0.5, pose.abs_pos().z());

  EXPECT_EQ(0.5, pose.rel_theta());
  EXPECT_EQ(0.5, pose.abs_theta());

  pose.set_theta(3.14);
  EXPECT_EQ(3.14, pose.rel_theta());
  pose.mutable_pos()->x() = 9.71;
  EXPECT_EQ(9.71, pose.rel_pos().x());

  EXPECT_EQ(nullptr, pose.base());
  Pose new_base;
  pose.set_base(&new_base);
  EXPECT_EQ(&new_base, pose.base());
}

// Check that Poses behave as expected when constructed relative to another
// POse.
TEST(PoseTest, BaseTest) {
  // Tolerance for the EXPECT_NEARs. Because we are doing enough trig operations
  // under the hood we actually start to lose some precision.
  constexpr double kEps = 1e-15;
  // The points we will construct have absolute positions at:
  // base1: (1, 1)
  // base2: (-1, 1)
  // rel1: (0, 2)
  // Where rel1 is expressed as compared to base1, noting that because base1
  // has a yaw of M_PI, the position of rel1 compared to base1 is (1, -1)
  // rather than (-1, 1).
  Pose base1({1, 1, 0}, M_PI);
  Pose base2({-1, 1, 0}, -M_PI / 2.0);
  Pose rel1(&base1, {1, -1, 0}, 0.0);
  EXPECT_NEAR(0.0, rel1.abs_pos().x(), kEps);
  EXPECT_NEAR(2.0, rel1.abs_pos().y(), kEps);
  EXPECT_NEAR(M_PI, rel1.abs_theta(), kEps);
  // Check that, when rebasing to base2, the absolute position does not change
  // and the relative POse changes to be relative to base2.
  Pose rel2 = rel1.Rebase(&base2);
  EXPECT_NEAR(rel1.abs_pos().x(), rel2.abs_pos().x(), kEps);
  EXPECT_NEAR(rel1.abs_pos().y(), rel2.abs_pos().y(), kEps);
  EXPECT_NEAR(rel1.abs_pos().z(), rel2.abs_pos().z(), kEps);
  EXPECT_NEAR(rel1.abs_theta(), rel2.abs_theta(), kEps);
  EXPECT_NEAR(-1.0, rel2.rel_pos().x(), kEps);
  EXPECT_NEAR(1.0, rel2.rel_pos().y(), kEps);
  EXPECT_NEAR(-M_PI / 2.0, rel2.rel_theta(), kEps);
  // Check that rebasing onto nullptr results in a Pose based in the global
  // frame.
  Pose abs = rel1.Rebase(nullptr);
  EXPECT_NEAR(rel1.abs_pos().x(), abs.abs_pos().x(), kEps);
  EXPECT_NEAR(rel1.abs_pos().y(), abs.abs_pos().y(), kEps);
  EXPECT_NEAR(rel1.abs_pos().z(), abs.abs_pos().z(), kEps);
  EXPECT_NEAR(rel1.abs_theta(), abs.abs_theta(), kEps);
  EXPECT_NEAR(rel1.abs_pos().x(), abs.rel_pos().x(), kEps);
  EXPECT_NEAR(rel1.abs_pos().y(), abs.rel_pos().y(), kEps);
  EXPECT_NEAR(rel1.abs_pos().z(), abs.rel_pos().z(), kEps);
  EXPECT_NEAR(rel1.abs_theta(), abs.rel_theta(), kEps);
}

// Tests that basic accessors for LineSegment behave as expected.
TEST(LineSegmentTest, BasicAccessorTest) {
  LineSegment l;
  EXPECT_EQ(0.0, l.pose1().rel_theta());
  l.mutable_pose1()->set_theta(1.234);
  EXPECT_EQ(1.234, l.pose1().rel_theta());
  EXPECT_EQ(0.0, l.pose2().rel_theta());
  l.mutable_pose2()->set_theta(5.678);
  EXPECT_EQ(5.678, l.pose2().rel_theta());

  const ::std::vector<Pose> plot_pts = l.PlotPoints();
  ASSERT_EQ(2u, plot_pts.size());
  EXPECT_EQ(l.pose1().rel_theta(), plot_pts[0].rel_theta());
  EXPECT_EQ(l.pose2().rel_theta(), plot_pts[1].rel_theta());
}

// Tests that basic checks for intersection function as expected.
TEST(LineSegmentTest, TrivialIntersectTest) {
  Pose p1({0, 0, 0}, 0.0), p2({2, 0, 0}, 0.0);
  // A line segment from (0, 0) to (0, 2).
  LineSegment l1(p1, p2);
  Pose q1({1, -1, 0}, 0.0), q2({1, 1, 0}, 0.0);
  // A line segment from (1, -1) to (1, 1).
  LineSegment l2(q1, q2);
  // The two line segments should intersect.
  EXPECT_TRUE(l1.Intersects(l2));
  EXPECT_TRUE(l2.Intersects(l1));

  // If we switch around the orderings such that the line segments are
  // (0, 0) -> (1, -1) and (2, 0)->(1, 1) then the line segments do not
  // intersect.
  LineSegment l3(p1, q1);
  LineSegment l4(p2, q2);
  EXPECT_FALSE(l3.Intersects(l4));
  EXPECT_FALSE(l4.Intersects(l3));
}

// Check that when we construct line segments that are collinear, both with
// overlapping bits and without overlapping bits, they register as not
// intersecting.
// We may want this behavior to change in the future, but for now check for
// consistency.
TEST(LineSegmentTest, CollinearIntersectTest) {
  Pose p1({0, 0, 0}, 0.0), p2({1, 0, 0}, 0.0), p3({2, 0, 0}, 0.0),
      p4({3, 0, 0}, 0.0);
  // These two line segments overlap and are collinear, one going from 0 to 2
  // and the other from 1 to 3 on the X-axis.
  LineSegment l1(p1, p3);
  LineSegment l2(p2, p4);
  EXPECT_FALSE(l1.Intersects(l2));
  EXPECT_FALSE(l2.Intersects(l1));

  // These two line segments do not overlap and are collinear, one going from 0
  // to 1 and the other from 2 to 3 on the X-axis.
  LineSegment l3(p1, p2);
  LineSegment l4(p3, p4);
  EXPECT_FALSE(l3.Intersects(l4));
  EXPECT_FALSE(l4.Intersects(l3));

  // Test when one line segment is completely contained within the other.
  LineSegment l5(p1, p4);
  LineSegment l6(p3, p2);
  EXPECT_FALSE(l5.Intersects(l6));
  EXPECT_FALSE(l6.Intersects(l5));
}

}  // namespace testing
}  // namespace control_loops
}  // namespace frc971

#include "y2019/control_loops/drivetrain/camera.h"

#include "gtest/gtest.h"

namespace y2019 {
namespace control_loops {
namespace testing {

// Check that a Target's basic operations work.
TEST(TargetTest, BasicTargetTest) {
  Target target({{1, 2, 3}, M_PI / 2.0}, 1.234,
                Target::TargetType::kFaceCargoBay, Target::GoalType::kHatches);

  EXPECT_EQ(1.0, target.pose().abs_pos().x());
  EXPECT_EQ(2.0, target.pose().abs_pos().y());
  EXPECT_EQ(3.0, target.pose().abs_pos().z());
  EXPECT_EQ(M_PI / 2.0, target.pose().abs_theta());
  EXPECT_EQ(1.234, target.radius());
  EXPECT_EQ(Target::GoalType::kHatches, target.goal_type());
  EXPECT_EQ(Target::TargetType::kFaceCargoBay, target.target_type());

  EXPECT_FALSE(target.occluded());
  target.set_occluded(true);
  EXPECT_TRUE(target.occluded());

  ::std::vector<Target::Pose> plot_pts = target.PlotPoints();
  ASSERT_EQ(4, plot_pts.size());
  for (const Target::Pose &pt : plot_pts) {
    EXPECT_EQ(3.0, pt.abs_pos().z());
    EXPECT_EQ(M_PI / 2.0, pt.abs_theta());
    // We don't particularly care about the plot point details, just check that
    // they are all roughly in the right vicinity:
    EXPECT_LT((pt.abs_pos() - target.pose().abs_pos()).norm(), 0.25);
  }
  EXPECT_EQ(plot_pts[0].abs_pos(), plot_pts[3].abs_pos());
}

typedef TypedCamera</*num_targets=*/3, /*num_obstacles=*/1, double> TestCamera;

class CameraTest : public ::testing::Test {
 public:
  // Set up three targets in a row, at (-1, 0), (0, 0), and (1, 0).
  // Make the right-most target (1, 0) be facing away from the camera, and give
  // the middle target some skew.
  // Place the camera at (0, -5) so the targets are a few meters away.
  // Place one obstacle in a place where it blocks the left-most target (-1, 0).
  CameraTest()
      : targets_{{Target(Target::Pose({-1.0, 0.0, 0.0}, M_PI_2)),
                  Target(Target::Pose({0.0, 0.0, kMiddleHeight},
                                      M_PI_2 + kMiddleSkew)),
                  Target(Target::Pose({1.0, 0.0, 0.0}, -M_PI_2))}},
        obstacles_{{TestCamera::LineSegment({{-2.0, -0.5, 0.0}, 0.0},
                                            {{-0.5, -0.5, 0.0}, 0.0})}},
        base_pose_({0.0, -5.0, 0.0}, M_PI_2),
        camera_({&base_pose_, {0.0, 0.0, 0.0}, 0.0}, M_PI_2, noise_parameters_,
                targets_, obstacles_) {}

 protected:
  static constexpr double kMiddleSkew = 0.1;
  static constexpr double kMiddleHeight = 0.5;
  ::std::array<Target, 3> targets_;
  ::std::array<TestCamera::LineSegment, 1> obstacles_;

  TestCamera::NoiseParameters noise_parameters_ = {
      .max_viewable_distance = 10.0,
      .heading_noise = 0.03,
      .nominal_distance_noise = 0.06,
      .nominal_skew_noise = 0.1,
      .nominal_height_noise = 0.01};

  // Provide base_pose_ as the base for the Pose used in the camera, to make it
  // so that we can easily move the camera around for testing.
  TestCamera::Pose base_pose_;
  TestCamera camera_;
};

constexpr double CameraTest::kMiddleSkew;
constexpr double CameraTest::kMiddleHeight;

constexpr double kEps = 1e-15;

// Check that, in the default setup we have, the correct targets are visible in
// the expected locations.
TEST_F(CameraTest, BasicCameraTest) {
  const auto views = camera_.target_views();
  // We should only be able to see one target (the middle one).
  ASSERT_EQ(1u, views.size());
  // And, check the actual result for correctness.
  EXPECT_NEAR(0.0, views[0].reading.heading, kEps);
  EXPECT_NEAR(5.0, views[0].reading.distance, kEps);
  EXPECT_NEAR(kMiddleSkew, views[0].reading.skew, kEps);
  EXPECT_NEAR(kMiddleHeight, views[0].reading.height, kEps);
  // Check that the noise outputs are sane; leave other tests to check the exact
  // values of the noise outputs.
  // All noise values should be strictly positive.
  EXPECT_GT(views[0].noise.heading, 0.0);
  EXPECT_GT(views[0].noise.distance, 0.0);
  EXPECT_GT(views[0].noise.skew, 0.0);
  EXPECT_GT(views[0].noise.height, 0.0);

  // Check that the PlotPoints for debugging are as expected (should be a single
  // line from the camera to the one visible target).
  const auto plot_pts = camera_.PlotPoints();
  ASSERT_EQ(1u, plot_pts.size());
  ASSERT_EQ(2u, plot_pts[0].size());
  EXPECT_EQ(camera_.pose().abs_pos(), plot_pts[0][0].abs_pos());
  EXPECT_EQ(views[0].target->pose().abs_pos(), plot_pts[0][1].abs_pos());
}

// Check that occluding the middle target makes it invisible.
TEST_F(CameraTest, OcclusionTest) {
  auto views = camera_.target_views();
  // We should only be able to see one target (the middle one).
  ASSERT_EQ(1u, views.size());
  targets_[1].set_occluded(true);
  TestCamera occluded_camera(camera_.pose(), camera_.fov(), noise_parameters_,
                             targets_, obstacles_);
  views = occluded_camera.target_views();
  // We should no longer see any targets.
  ASSERT_EQ(0u, views.size());
}

// Checks that targets outside of the field-of-view don't show up.
TEST_F(CameraTest, FovTest) {
  // Initially, we should still see just the middle target.
  EXPECT_EQ(1u, camera_.target_views().size());
  // Point camera so that the middle target is just barely in its field of view.
  base_pose_.set_theta(3.0 * M_PI / 4.0 - 0.01);
  EXPECT_EQ(1u, camera_.target_views().size());
  // Point camera so that the middle target is just outside of its FoV.
  base_pose_.set_theta(3.0 * M_PI / 4.0 + 0.01);
  EXPECT_EQ(0u, camera_.target_views().size());
  // Check the same things, but on the other edge of the FoV:
  base_pose_.set_theta(M_PI / 4.0 + 0.01);
  EXPECT_EQ(1u, camera_.target_views().size());
  base_pose_.set_theta(M_PI / 4.0 - 0.01);
  EXPECT_EQ(0u, camera_.target_views().size());
}

// Checks that targets don't show up when very far away.
TEST_F(CameraTest, FarAwayTest) {
  EXPECT_EQ(1u, camera_.target_views().size());
  // If we move the camera really far away we can't see it any more:
  base_pose_.mutable_pos()->y() = -1000.0;
  EXPECT_EQ(0u, camera_.target_views().size());
}

// Checks that targets which are highly skewed only show up if we are
// arbitrarily close.
TEST_F(CameraTest, HighlySkewedTest) {
  // Skew the target a bunch.
  targets_[1] = Target({{0.0, 0.0, 0.0}, 0.01});
  TestCamera occluded_camera(camera_.pose(), camera_.fov(), noise_parameters_,
                             targets_, obstacles_);
  EXPECT_EQ(0u, occluded_camera.target_views().size());
  // But if we get really close we should still see it...
  base_pose_.mutable_pos()->y() = -0.1;
  EXPECT_EQ(1u, camera_.target_views().size());
}

using Reading = TestCamera::TargetView::Reading;

// Checks that reading noises have the expected characteristics (mostly, going
// up linearly with distance):
TEST_F(CameraTest, DistanceNoiseTest) {
  const Reading normal_noise = camera_.target_views()[0].noise;
  // Get twice as close:
  base_pose_.mutable_pos()->y() /= 2.0;
  const Reading closer_noise = camera_.target_views()[0].noise;
  EXPECT_EQ(normal_noise.distance / 2.0, closer_noise.distance);
  EXPECT_EQ(normal_noise.skew / 2.0, closer_noise.skew);
  EXPECT_EQ(normal_noise.height / 2.0, closer_noise.height);
  // Heading reading should be equally good.
  EXPECT_EQ(normal_noise.heading, closer_noise.heading);
}

class CameraViewParamTest : public CameraTest,
                            public ::testing::WithParamInterface<Reading> {};

// Checks that invalid or absurd measurements result in large but finite noises
// and non-visible targets.
TEST_P(CameraViewParamTest, InvalidReading) {
  TestCamera::TargetView view;
  view.reading = GetParam();
  bool visible = true;
  camera_.PopulateNoise(&view, &visible);
  // Target should not be visible
  EXPECT_FALSE(visible);
  // We should end up with finite but very large noises when things are invalid.
  EXPECT_TRUE(::std::isfinite(view.noise.heading));
  EXPECT_TRUE(::std::isfinite(view.noise.distance));
  EXPECT_TRUE(::std::isfinite(view.noise.skew));
  EXPECT_TRUE(::std::isfinite(view.noise.height));
  // Don't check heading noise because it is always constant.
  EXPECT_LT(10, view.noise.distance);
  EXPECT_LT(10, view.noise.skew);
  EXPECT_LT(5, view.noise.height);
}

INSTANTIATE_TEST_CASE_P(
    InvalidMeasurements, CameraViewParamTest,
    ::testing::Values(
        // heading, distance, height, skew
        Reading({100.0, -10.0, -10.0, -3.0}), Reading({0.0, 1.0, 0.0, -3.0}),
        Reading({0.0, 1.0, 0.0, 3.0}), Reading({0.0, 1.0, 0.0, 9.0}),
        Reading({0.0, ::std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0}),
        Reading({0.0, ::std::numeric_limits<double>::infinity(), 0.0, 0.0}),
        Reading({0.0, 1.0, 0.0, ::std::numeric_limits<double>::infinity()}),
        Reading({0.0, 1.0, 0.0, ::std::numeric_limits<double>::quiet_NaN()})));

}  // namespace testing
}  // namespace control_loops
}  // namespace y2019

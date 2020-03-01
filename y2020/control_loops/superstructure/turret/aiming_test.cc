#include "y2020/control_loops/superstructure/turret/aiming.h"

#include "frc971/control_loops/pose.h"
#include "gtest/gtest.h"
#include "y2020/constants.h"
#include "y2020/control_loops/drivetrain/drivetrain_base.h"

namespace y2020 {
namespace control_loops {
namespace superstructure {
namespace turret {
namespace testing {

using frc971::control_loops::Pose;

class AimerTest : public ::testing::Test {
 public:
  typedef Aimer::Goal Goal;
  typedef Aimer::Status Status;
  struct StatusData {
    double x;
    double y;
    double theta;
    double linear;
    double angular;
  };
  aos::FlatbufferDetachedBuffer<Status> MakeStatus(const StatusData &data) {
    flatbuffers::FlatBufferBuilder fbb;
    frc971::control_loops::drivetrain::LocalizerState::Builder state_builder(
        fbb);
    state_builder.add_left_velocity(
        data.linear -
        data.angular * drivetrain::GetDrivetrainConfig().robot_radius);
    state_builder.add_right_velocity(
        data.linear +
        data.angular * drivetrain::GetDrivetrainConfig().robot_radius);
    const auto state_offset = state_builder.Finish();
    Status::Builder builder(fbb);
    builder.add_x(data.x);
    builder.add_y(data.y);
    builder.add_theta(data.theta);
    builder.add_localizer(state_offset);
    fbb.Finish(builder.Finish());
    return fbb.Release();
  }

  const Goal *Update(
      const StatusData &data, aos::Alliance alliance = aos::Alliance::kBlue,
      Aimer::WrapMode wrap_mode = Aimer::WrapMode::kAvoidEdges,
      Aimer::ShotMode shot_mode = Aimer::ShotMode::kShootOnTheFly) {
    const auto buffer = MakeStatus(data);
    aimer_.Update(&buffer.message(), alliance, wrap_mode, shot_mode);
    const Goal *goal = aimer_.TurretGoal();
    EXPECT_TRUE(goal->ignore_profile());
    return goal;
  }

 protected:
  Aimer aimer_;
};

TEST_F(AimerTest, StandingStill) {
  const Pose target = OuterPortPose(aos::Alliance::kBlue);
  const Goal *goal = Update({.x = target.abs_pos().x() + 1.0,
                             .y = target.abs_pos().y() + 0.0,
                             .theta = 0.0,
                             .linear = 0.0,
                             .angular = 0.0});
  EXPECT_EQ(0.0, goal->unsafe_goal());
  EXPECT_EQ(0.0, goal->goal_velocity());
  EXPECT_EQ(1.0, aimer_.DistanceToGoal());
  goal = Update({.x = target.abs_pos().x() + 1.0,
                 .y = target.abs_pos().y() + 0.0,
                 .theta = 1.0,
                 .linear = 0.0,
                 .angular = 0.0});
  EXPECT_EQ(-1.0, goal->unsafe_goal());
  EXPECT_EQ(0.0, goal->goal_velocity());
  goal = Update({.x = target.abs_pos().x() + 1.0,
                 .y = target.abs_pos().y() + 0.0,
                 .theta = -1.0,
                 .linear = 0.0,
                 .angular = 0.0});
  EXPECT_EQ(1.0, aos::math::NormalizeAngle(goal->unsafe_goal()));
  EXPECT_EQ(0.0, goal->goal_velocity());
  EXPECT_EQ(1.0, aimer_.DistanceToGoal());
  // Test that we handle the case that where we are right on top of the target.
  goal = Update({.x = target.abs_pos().x() + 0.0,
                 .y = target.abs_pos().y() + 0.0,
                 .theta = 0.0,
                 .linear = 0.0,
                 .angular = 0.0});
  EXPECT_EQ(M_PI, goal->unsafe_goal());
  EXPECT_EQ(0.0, goal->goal_velocity());
  EXPECT_EQ(0.0, aimer_.DistanceToGoal());
}

TEST_F(AimerTest, SpinningRobot) {
  const Pose target = OuterPortPose(aos::Alliance::kBlue);
  const Goal *goal = Update({.x = target.abs_pos().x() + 1.0,
                             .y = target.abs_pos().y() + 0.0,
                             .theta = 0.0,
                             .linear = 0.0,
                             .angular = 1.0});
  EXPECT_EQ(0.0, goal->unsafe_goal());
  EXPECT_FLOAT_EQ(-1.0, goal->goal_velocity());
}

// Tests that when we drive straight away from the target we don't have to spin
// the turret.
TEST_F(AimerTest, DrivingAwayFromTarget) {
  const Pose target = OuterPortPose(aos::Alliance::kBlue);
  // To keep the test simple, disable shooting on the fly so that the
  // goal distance comes out in an easy to calculate number.
  const Goal *goal = Update({.x = target.abs_pos().x() + 1.0,
                             .y = target.abs_pos().y() + 0.0,
                             .theta = 0.0,
                             .linear = 1.0,
                             .angular = 0.0},
                            aos::Alliance::kBlue, Aimer::WrapMode::kAvoidEdges,
                            Aimer::ShotMode::kStatic);
  EXPECT_EQ(0.0, goal->unsafe_goal());
  EXPECT_FLOAT_EQ(0.0, goal->goal_velocity());
  EXPECT_EQ(1.0, aimer_.DistanceToGoal());
  // Next, try with shooting-on-the-fly enabled--because we are driving straight
  // towards the target, only the goal distance should be impacted.
  goal = Update({.x = target.abs_pos().x() + 1.0,
                 .y = target.abs_pos().y() + 0.0,
                 .theta = 0.0,
                 .linear = 1.0,
                 .angular = 0.0},
                aos::Alliance::kBlue, Aimer::WrapMode::kAvoidEdges,
                Aimer::ShotMode::kShootOnTheFly);
  EXPECT_EQ(0.0, goal->unsafe_goal());
  EXPECT_FLOAT_EQ(0.0, goal->goal_velocity());
  EXPECT_LT(1.0001, aimer_.DistanceToGoal());
  EXPECT_GT(1.1, aimer_.DistanceToGoal());
}

// Tests that when we drive perpendicular to the target, we do have to spin.
TEST_F(AimerTest, DrivingLateralToTarget) {
  const Pose target = OuterPortPose(aos::Alliance::kBlue);
  // To keep the test simple, disable shooting on the fly so that the
  // goal_velocity comes out in an easy to calculate number.
  const Goal *goal = Update({.x = target.abs_pos().x() + 0.0,
                             .y = target.abs_pos().y() + 1.0,
                             .theta = 0.0,
                             .linear = 1.0,
                             .angular = 0.0},
                            aos::Alliance::kBlue, Aimer::WrapMode::kAvoidEdges,
                            Aimer::ShotMode::kStatic);
  EXPECT_EQ(M_PI_2, goal->unsafe_goal());
  EXPECT_FLOAT_EQ(-1.0, goal->goal_velocity());
  EXPECT_EQ(1.0, aimer_.DistanceToGoal());
  // Next, test with shooting-on-the-fly enabled, The goal numbers should all be
  // slightly offset due to the robot velocity.
  goal = Update({.x = target.abs_pos().x() + 0.0,
                 .y = target.abs_pos().y() + 1.0,
                 .theta = 0.0,
                 .linear = 1.0,
                 .angular = 0.0},
                aos::Alliance::kBlue, Aimer::WrapMode::kAvoidEdges,
                Aimer::ShotMode::kShootOnTheFly);
  // Confirm that the turret heading goal is less then -pi / 2, but not by too
  // much.
  EXPECT_GT(M_PI_2 - 0.001, goal->unsafe_goal());
  EXPECT_LT(M_PI_2 - 0.1, goal->unsafe_goal());
  // Similarly, the turret velocity goal should be a bit greater than -1.0,
  // since the turret is no longer at exactly a right angle.
  EXPECT_LT(-1.0, goal->goal_velocity());
  EXPECT_GT(-0.95, goal->goal_velocity());
  // And the distance to the goal should be a bit greater than 1.0.
  EXPECT_LT(1.0001, aimer_.DistanceToGoal());
  EXPECT_GT(1.1, aimer_.DistanceToGoal());
}

// Confirms that we will indeed shoot at the inner port when we have a good shot
// angle on it.
TEST_F(AimerTest, InnerPort) {
  const Pose target = InnerPortPose(aos::Alliance::kRed);
  const Goal *goal = Update({.x = target.abs_pos().x() + 1.0,
                             .y = target.abs_pos().y() + 0.0,
                             .theta = 0.0,
                             .linear = 0.0,
                             .angular = 0.0},
                            aos::Alliance::kRed);
  EXPECT_EQ(0.0, goal->unsafe_goal());
  EXPECT_EQ(0.0, goal->goal_velocity());
}

// Confirms that when we move the turret heading so that it would be entirely
// out of the normal range of motion that we send a valid (in-range) goal.
TEST_F(AimerTest, WrapWhenOutOfRange) {
  // Start ourselves needing a turret angle of M_PI.
  const Pose target = OuterPortPose(aos::Alliance::kBlue);
  StatusData status{.x = target.abs_pos().x() + 1.0,
                    .y = target.abs_pos().y() + 0.0,
                    .theta = 0.0,
                    .linear = 0.0,
                    .angular = 0.0};
  const Goal *goal = Update(status);
  EXPECT_EQ(0.0, goal->unsafe_goal());
  EXPECT_EQ(0.0, goal->goal_velocity());
  // Move the robot a small amount--we should go past pi and not wrap yet.
  status.theta = -0.1;
  goal = Update(status);
  EXPECT_FLOAT_EQ(0.1, goal->unsafe_goal());
  EXPECT_EQ(0.0, goal->goal_velocity());
  // Move the robot so that, if we had no hard-stops, we would go past it.
  status.theta = -2.0;
  goal = Update(status);
  EXPECT_FLOAT_EQ(2.0, goal->unsafe_goal());
  EXPECT_EQ(0.0, goal->goal_velocity());
}

// Confirms that the avoid edges turret mode doesn't let us get all the way to
// the turret hard-stops but that the avoid wrapping mode does.
TEST_F(AimerTest, WrappingModes) {
  // Start ourselves needing a turret angle of M_PI.
  const Pose target = OuterPortPose(aos::Alliance::kBlue);
  StatusData status{.x = target.abs_pos().x() - 1.0,
                    .y = target.abs_pos().y() + 0.0,
                    .theta = 0.0,
                    .linear = 0.0,
                    .angular = 0.0};
  const Goal *goal =
      Update(status, aos::Alliance::kBlue, Aimer::WrapMode::kAvoidWrapping);
  EXPECT_EQ(M_PI, goal->unsafe_goal());
  EXPECT_EQ(0.0, goal->goal_velocity());
  constexpr double kUpperLimit = constants::Values::kTurretRange().upper;
  // Move the robot to the upper limit with AvoidWrapping set--we should be at
  // the upper limit and not wrapped.
  status.theta = goal->unsafe_goal() - kUpperLimit;
  goal = Update(status, aos::Alliance::kBlue, Aimer::WrapMode::kAvoidWrapping);
  EXPECT_FLOAT_EQ(kUpperLimit, goal->unsafe_goal());
  EXPECT_EQ(0.0, goal->goal_velocity());
  // Enter kAvoidEdges mode--we should wrap around.
  goal = Update(status, aos::Alliance::kBlue, Aimer::WrapMode::kAvoidEdges);
  // confirm that this test is actually testing something...
  ASSERT_NE(aos::math::NormalizeAngle(kUpperLimit), kUpperLimit);
  EXPECT_FLOAT_EQ(aos::math::NormalizeAngle(kUpperLimit), goal->unsafe_goal());
  EXPECT_EQ(0.0, goal->goal_velocity());
}

}  // namespace testing
}  // namespace turret
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2020

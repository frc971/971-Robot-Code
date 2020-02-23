#include "y2020/control_loops/superstructure/turret/aiming.h"

#include "frc971/control_loops/pose.h"
#include "gtest/gtest.h"
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

  const Goal *Update(const StatusData &data,
                     aos::Alliance alliance = aos::Alliance::kBlue) {
    const auto buffer = MakeStatus(data);
    aimer_.Update(&buffer.message(), alliance);
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
  EXPECT_EQ(M_PI, goal->unsafe_goal());
  EXPECT_EQ(0.0, goal->goal_velocity());
  goal = Update({.x = target.abs_pos().x() + 1.0,
                 .y = target.abs_pos().y() + 0.0,
                 .theta = 1.0,
                 .linear = 0.0,
                 .angular = 0.0});
  EXPECT_EQ(M_PI - 1.0, goal->unsafe_goal());
  EXPECT_EQ(0.0, goal->goal_velocity());
  goal = Update({.x = target.abs_pos().x() + 1.0,
                 .y = target.abs_pos().y() + 0.0,
                 .theta = -1.0,
                 .linear = 0.0,
                 .angular = 0.0});
  EXPECT_EQ(-M_PI + 1.0, goal->unsafe_goal());
  EXPECT_EQ(0.0, goal->goal_velocity());
  // Test that we handle the case that where we are right on top of the target.
  goal = Update({.x = target.abs_pos().x() + 0.0,
                 .y = target.abs_pos().y() + 0.0,
                 .theta = 0.0,
                 .linear = 0.0,
                 .angular = 0.0});
  EXPECT_EQ(0.0, goal->unsafe_goal());
  EXPECT_EQ(0.0, goal->goal_velocity());
}

TEST_F(AimerTest, SpinningRobot) {
  const Pose target = OuterPortPose(aos::Alliance::kBlue);
  const Goal *goal = Update({.x = target.abs_pos().x() + 1.0,
                             .y = target.abs_pos().y() + 0.0,
                             .theta = 0.0,
                             .linear = 0.0,
                             .angular = 1.0});
  EXPECT_EQ(M_PI, goal->unsafe_goal());
  EXPECT_FLOAT_EQ(-1.0, goal->goal_velocity());
}

// Tests that when we drive straight away from the target we don't have to spin
// the turret.
TEST_F(AimerTest, DrivingAwayFromTarget) {
  const Pose target = OuterPortPose(aos::Alliance::kBlue);
  const Goal *goal = Update({.x = target.abs_pos().x() + 1.0,
                             .y = target.abs_pos().y() + 0.0,
                             .theta = 0.0,
                             .linear = 1.0,
                             .angular = 0.0});
  EXPECT_EQ(M_PI, goal->unsafe_goal());
  EXPECT_FLOAT_EQ(0.0, goal->goal_velocity());
}

// Tests that when we drive perpendicular to the target, we do have to spin.
TEST_F(AimerTest, DrivingLateralToTarget) {
  const Pose target = OuterPortPose(aos::Alliance::kBlue);
  const Goal *goal = Update({.x = target.abs_pos().x() + 0.0,
                             .y = target.abs_pos().y() + 1.0,
                             .theta = 0.0,
                             .linear = 1.0,
                             .angular = 0.0});
  EXPECT_EQ(-M_PI_2, goal->unsafe_goal());
  EXPECT_FLOAT_EQ(-1.0, goal->goal_velocity());
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
  EXPECT_EQ(M_PI, goal->unsafe_goal());
  EXPECT_EQ(0.0, goal->goal_velocity());
}

}  // namespace testing
}  // namespace turret
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2020

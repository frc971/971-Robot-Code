#include "frc971/control_loops/aiming/aiming.h"

#include "frc971/control_loops/pose.h"
#include "gtest/gtest.h"
#include "frc971/constants.h"

namespace frc971::control_loops::aiming::testing {

TEST(AimerTest, StandingStill) {
  const Pose target({0.0, 0.0, 0.0}, 0.0);
  Pose robot_pose({1.0, 0.0, 0.0}, 0.0);
  const constants::Range range{-4.5, 4.5, -4.0, 4.0};
  const double kBallSpeed = 10.0;
  // Robot is ahead of target, should have to turret to 180 deg to shoot.
  TurretGoal goal = AimerGoal(
      ShotConfig{target, ShotMode::kShootOnTheFly, range, kBallSpeed, 0.0, 0.0},
      RobotState{robot_pose, {0.0, 0.0}, 0.0, 0.0});
  EXPECT_FLOAT_EQ(M_PI, goal.position);
  EXPECT_FLOAT_EQ(0.0, goal.velocity);
  EXPECT_FLOAT_EQ(1.0, goal.virtual_shot_distance);
  EXPECT_FLOAT_EQ(1.0, goal.target_distance);

  // If there is a turret offset, it should get compensated out.
  goal = AimerGoal(ShotConfig{target, ShotMode::kShootOnTheFly, range,
                              kBallSpeed, 0.0, M_PI},
                   RobotState{robot_pose, {0.0, 0.0}, 0.0, 0.0});
  EXPECT_FLOAT_EQ(0.0, goal.position);

  robot_pose = Pose({-1.0, 0.0, 0.0}, 1.0);
  goal = AimerGoal(
      ShotConfig{target, ShotMode::kShootOnTheFly, range, kBallSpeed, 0.0, 0.0},
      RobotState{robot_pose, {0.0, 0.0}, 0.0, 0.0});
  EXPECT_FLOAT_EQ(-1.0, goal.position);
  EXPECT_FLOAT_EQ(0.0, goal.velocity);
  EXPECT_FLOAT_EQ(1.0, goal.virtual_shot_distance);
  EXPECT_FLOAT_EQ(1.0, goal.target_distance);

  // Test that we handle the case that where we are right on top of the target.
  robot_pose = Pose({0.0, 0.0, 0.0}, 0.0);
  goal = AimerGoal(
      ShotConfig{target, ShotMode::kShootOnTheFly, range, kBallSpeed, 0.0, 0.0},
      RobotState{robot_pose, {0.0, 0.0}, 0.0, 0.0});
  EXPECT_FLOAT_EQ(0.0, goal.position);
  EXPECT_FLOAT_EQ(0.0, goal.velocity);
  EXPECT_FLOAT_EQ(0.0, goal.virtual_shot_distance);
  EXPECT_FLOAT_EQ(0.0, goal.target_distance);
}

// Test that spinning in place results in correct velocity goals.
TEST(AimerTest, SpinningRobot) {
  const Pose target({0.0, 0.0, 0.0}, 0.0);
  Pose robot_pose({-1.0, 0.0, 0.0}, 0.0);
  const constants::Range range{-4.5, 4.5, -4.0, 4.0};
  const double kBallSpeed = 10.0;
  TurretGoal goal = AimerGoal(
      ShotConfig{target, ShotMode::kShootOnTheFly, range, kBallSpeed, 0.0, 0.0},
      RobotState{robot_pose, {0.0, 0.0}, 971.0, 0.0});
  EXPECT_FLOAT_EQ(0.0, goal.position);
  EXPECT_FLOAT_EQ(-971.0, goal.velocity);
}

// Tests that when we drive straight away from the target we don't have to spin
// the turret.
TEST(AimerTest, DrivingAwayFromTarget) {
  const Pose target({0.0, 0.0, 0.0}, 0.0);
  Pose robot_pose({-1.0, 0.0, 0.0}, 0.0);
  const constants::Range range{-4.5, 4.5, -4.0, 4.0};
  const double kBallSpeed = 10.0;
  TurretGoal goal = AimerGoal(
      ShotConfig{target, ShotMode::kStatic, range, kBallSpeed, 0.0, 0.0},
      RobotState{robot_pose, {-1.0, 0.0}, 0.0, 0.0});
  EXPECT_FLOAT_EQ(0.0, goal.position);
  EXPECT_FLOAT_EQ(0.0, goal.velocity);
  EXPECT_FLOAT_EQ(1.0, goal.virtual_shot_distance);
  EXPECT_FLOAT_EQ(1.0, goal.target_distance);
  // Next, try with shooting-on-the-fly enabled--because we are driving straight
  // away from the target, only the goal distance should be impacted.
  goal = AimerGoal(
      ShotConfig{target, ShotMode::kShootOnTheFly, range, kBallSpeed, 0.0, 0.0},
      RobotState{robot_pose, {-1.0, 0.0}, 0.0, 0.0});
  EXPECT_FLOAT_EQ(0.0, goal.position);
  EXPECT_FLOAT_EQ(0.0, goal.velocity);
  EXPECT_FLOAT_EQ(1.111, goal.virtual_shot_distance);
  EXPECT_FLOAT_EQ(1.0, goal.target_distance);
}

// Tests that when we drive perpendicular to the target, we do have to spin.
TEST(AimerTest, DrivingLateralToTarget) {
  const Pose target({0.0, 0.0, 0.0}, 0.0);
  Pose robot_pose({0.0, -1.0, 0.0}, 0.0);
  const constants::Range range{-4.5, 4.5, -4.0, 4.0};
  const double kBallSpeed = 10.0;
  TurretGoal goal = AimerGoal(
      ShotConfig{target, ShotMode::kStatic, range, kBallSpeed, 0.0, 0.0},
      RobotState{robot_pose, {1.0, 0.0}, 0.0, 0.0});
  EXPECT_FLOAT_EQ(M_PI_2, goal.position);
  EXPECT_FLOAT_EQ(1.0, goal.velocity);
  EXPECT_FLOAT_EQ(1.0, goal.virtual_shot_distance);
  EXPECT_FLOAT_EQ(1.0, goal.target_distance);
  // Next, test with shooting-on-the-fly enabled, The goal numbers should all be
  // slightly offset due to the robot velocity.
  goal = AimerGoal(
      ShotConfig{target, ShotMode::kShootOnTheFly, range, kBallSpeed, 0.0, 0.0},
      RobotState{robot_pose, {1.0, 0.0}, 0.0, 0.0});
  // Confirm that the turret heading goal is a bit more than pi / 2, but not by
  // too much.
  EXPECT_LT(M_PI_2 + 0.01, goal.position);
  EXPECT_GT(M_PI_2 + 0.5, goal.position);
  // Similarly, the turret velocity goal should be a bit less than 1.0,
  // since the turret is no longer at exactly a right angle.
  EXPECT_LT(0.9, goal.velocity);
  EXPECT_GT(0.999, goal.velocity);
  // And the distance to the goal should be a bit greater than 1.0.
  EXPECT_LT(1.00001, goal.virtual_shot_distance);
  EXPECT_GT(1.1, goal.virtual_shot_distance);
  EXPECT_FLOAT_EQ(1.0, goal.target_distance);
}

// Confirms that when we move the turret heading so that it would be entirely
// out of the normal range of motion that we send a valid (in-range) goal.
// I.e., test that we have some hysteresis, but that it doesn't take us
// out-of-range.
TEST(AimerTest, WrapWhenOutOfRange) {
  // Start ourselves needing a turret angle of 0.0.
  const Pose target({0.0, 0.0, 0.0}, 0.0);
  Pose robot_pose({-1.0, 0.0, 0.0}, 0.0);
  const constants::Range range{-5.5, 5.5, -5.0, 5.0};
  const double kBallSpeed = 10.0;
  TurretGoal goal = AimerGoal(
      ShotConfig{target, ShotMode::kShootOnTheFly, range, kBallSpeed, 0.0, 0.0},
      RobotState{robot_pose, {0.0, 0.0}, 0.0, 0.0});
  EXPECT_FLOAT_EQ(0.0, goal.position);
  EXPECT_FLOAT_EQ(0.0, goal.velocity);

  // Rotate a bit...
  robot_pose = Pose({-1.0, 0.0, 0.0}, 2.0);
  goal = AimerGoal(
      ShotConfig{target, ShotMode::kShootOnTheFly, range, kBallSpeed, 0.0, 0.0},
      RobotState{robot_pose, {0.0, 0.0}, 0.0, goal.position});
  EXPECT_FLOAT_EQ(-2.0, goal.position);
  EXPECT_FLOAT_EQ(0.0, goal.velocity);

  // Rotate to the soft stop.
  robot_pose = Pose({-1.0, 0.0, 0.0}, 4.0);
  goal = AimerGoal(
      ShotConfig{target, ShotMode::kShootOnTheFly, range, kBallSpeed, 0.0, 0.0},
      RobotState{robot_pose, {0.0, 0.0}, 0.0, goal.position});
  EXPECT_FLOAT_EQ(-4.0, goal.position);
  EXPECT_FLOAT_EQ(0.0, goal.velocity);

  // Rotate past the hard stop.
  robot_pose = Pose({-1.0, 0.0, 0.0}, 0.0);
  goal = AimerGoal(
      ShotConfig{target, ShotMode::kShootOnTheFly, range, kBallSpeed, 0.0, 0.0},
      RobotState{robot_pose, {0.0, 0.0}, 0.0, goal.position});
  EXPECT_FLOAT_EQ(0.0, goal.position);
  EXPECT_FLOAT_EQ(0.0, goal.velocity);
}

}  // namespace frc971::control_loops::aiming::testing

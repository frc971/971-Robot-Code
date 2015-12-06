#include <cmath>

#include "gtest/gtest.h"

#include "aos/common/logging/logging.h"
#include "aos/testing/test_logging.h"
#include "y2015/util/kinematics.h"
#include "frc971/control_loops/team_number_test_environment.h"

namespace aos {
namespace util {
namespace testing {

// For verifcation against Mr. Schuh's internal tests.
// Please do not comment on this in a code review. We can remove this after the
// season to satisfy any OCD.
bool k_internal_debug = false;

class KinematicsTest : public ::testing::Test {
 public:
  KinematicsTest()
      : lower_height_limit_(0.01),
        upper_height_limit_(0.65),
        lower_angle_limit_(-M_PI / 2.0),
        upper_angle_limit_(M_PI / 2.0) {}

  void SetUp() {
      ::aos::testing::EnableTestLogging();
      kinematics_ = ElevatorArmKinematics(arm_length_, upper_height_limit_, lower_height_limit_,
                    upper_angle_limit_, lower_angle_limit_);
  }

 protected:
  double lower_height_limit_;
  double upper_height_limit_;
  double lower_angle_limit_;
  double upper_angle_limit_;
  double arm_length_ = 0.7366;

  ElevatorArmKinematics kinematics_;
};

// Used for internal debugging and verification only not acctual testing.
// Please do not comment on this in a code review. We can remove this after
// the season to satisfy any OCD.
TEST_F(KinematicsTest,
       PrintPointsSoWeCanHandVerifyThemUntilABetterTestIsMadeAndBrianIsHappy) {
  if (k_internal_debug) {
    for (double y = -1.0; y <= 1.0; y += 0.2) {
      for (double x = -1.0; x <= 1.0; x += 0.2) {
        ElevatorArmKinematics::KinematicResult res;
        int region = kinematics_.InverseKinematic(x, y, 0.0, 0.0, &res);
        printf(
            " %12.3f %12.3f   %8.3f  %9.3f  %8.2f %12d   %12.4f      %10.4f "
            "%15.4f %16.4f\n",
            x, y, res.elevator_height, res.arm_angle,
            res.arm_angle * 180.0 / M_PI, region, res.fridge_x, res.fridge_h,
            res.fridge_x - x, res.fridge_h - y);
      }
    }

    // Make a set of calls to test the grabber arm intersection test code.
    printf(
        "#  ArmAngle (degrees) ElevatorHeight  ClawAngle (degrees) GrabberX "
        "GrabberH intersectReturn SafeClawAngle (degrees)\n");
    for (double elevator_height = kinematics_.get_elevator_min();
         elevator_height <= kinematics_.get_elevator_max();
         elevator_height += 0.10) {
      for (double arm_angle = kinematics_.get_lower_angle_limit();
           arm_angle <= kinematics_.get_upper_angle_limit() + 0.01;
           arm_angle += M_PI * 0.5 / 9.0) {
        double claw_angle = M_PI * 0.25;
        double safe_claw_angle;
        double intersectReturnValue = kinematics_.GrabberArmIntersectionCheck(
            elevator_height, arm_angle, claw_angle, &safe_claw_angle);
        Eigen::Vector2d grabber_location =
            kinematics_.ForwardKinematicNoChecking(elevator_height, arm_angle);

        printf(
            "  %8.4f %8.2f   %8.2f %14.4f %9.2f %9.2f %9.2f %10.3f  %13.4f "
            "%12.3f\n",
            arm_angle, arm_angle * 180.0 / M_PI, elevator_height, claw_angle,
            claw_angle * 180.0 / M_PI, grabber_location.x(),
            grabber_location.y(), intersectReturnValue, safe_claw_angle,
            safe_claw_angle * 180.0 / M_PI);
      }
      printf("\n");
    }
  }
}

TEST_F(KinematicsTest, ValidIntersectCheckPointAtBottomOfElevatorRange) {
  double safe_claw_angle;
  double elevator_height = 0.01;
  double arm_angle = 30.0 * M_PI / 180.0;
  double claw_angle = M_PI * 0.25;
  bool intersectReturnValue = kinematics_.GrabberArmIntersectionCheck(
      elevator_height, arm_angle, claw_angle, &safe_claw_angle);
  EXPECT_TRUE(intersectReturnValue);
  EXPECT_EQ(safe_claw_angle, claw_angle);
}

TEST_F(KinematicsTest, ValidIntersectCheckPointAtMiddleOfElevatorRange) {
  double safe_claw_angle;
  double elevator_height = 0.4;
  double arm_angle = 30.0 * M_PI / 180.0;
  double claw_angle = M_PI * 0.25;
  bool intersectReturnValue = kinematics_.GrabberArmIntersectionCheck(
      elevator_height, arm_angle, claw_angle, &safe_claw_angle);
  EXPECT_TRUE(intersectReturnValue);
  EXPECT_EQ(safe_claw_angle, claw_angle);
}

TEST_F(KinematicsTest,
       invalidIntersectCheckPointAtBottomOfElevatorRangeWithSafeClawAngle) {
  double safe_claw_angle;
  double elevator_height = 0.01;
  double arm_angle = -20.0 * M_PI / 180.0;
  double claw_angle = M_PI * 0.25;
  bool intersectReturnValue = kinematics_.GrabberArmIntersectionCheck(
      elevator_height, arm_angle, claw_angle, &safe_claw_angle);
  EXPECT_FALSE(intersectReturnValue);
  EXPECT_NEAR(safe_claw_angle, 0.0435733, 0.000001);
}

TEST_F(KinematicsTest,
       invalidIntersectCheckPointAtMiddleOfElevatorRangeWithSafeClawAngle) {
  double safe_claw_angle;
  double elevator_height = 0.41;
  double arm_angle = -60.0 * M_PI / 180.0;
  double claw_angle = M_PI * 0.25;
  bool intersectReturnValue = kinematics_.GrabberArmIntersectionCheck(
      elevator_height, arm_angle, claw_angle, &safe_claw_angle);
  EXPECT_FALSE(intersectReturnValue);
  EXPECT_NEAR(safe_claw_angle, 0.12655341, 0.000001);
}

TEST_F(KinematicsTest,
       invalidIntersectCheckPointAtBottomOfElevatorRangeNoSafeClawAngle) {
  double safe_claw_angle;
  double elevator_height = 0.01;
  double arm_angle = -30.0 * M_PI / 180.0;
  double claw_angle = M_PI * 0.25;
  bool intersectReturnValue = kinematics_.GrabberArmIntersectionCheck(
      elevator_height, arm_angle, claw_angle, &safe_claw_angle);
  EXPECT_FALSE(intersectReturnValue);
  EXPECT_EQ(safe_claw_angle, -1.0);
}

TEST_F(KinematicsTest,
       invalidIntersectCheckPointAtMiddleOfElevatorRangeNoSafeClawAngle) {
  double safe_claw_angle;
  double elevator_height = 0.41;
  double arm_angle = -70.0 * M_PI / 180.0;
  double claw_angle = M_PI * 0.25;
  bool intersectReturnValue = kinematics_.GrabberArmIntersectionCheck(
      elevator_height, arm_angle, claw_angle, &safe_claw_angle);
  EXPECT_FALSE(intersectReturnValue);
  EXPECT_EQ(safe_claw_angle, -1.0);
}

// Tests that velocity calulations are correct
TEST_F(KinematicsTest, InverseKinematicVelocity) {
  ElevatorArmKinematics::KinematicResult result;
  // move striaght up and verify that only hieght changes
  EXPECT_EQ(0, kinematics_.InverseKinematic(0.0, 0.2, 0.0, 0.7, &result));
  EXPECT_NEAR(0.0, result.arm_velocity, 0.00001);
  EXPECT_NEAR(0.7, result.elevator_velocity, 0.00001);
  // check the negative
  EXPECT_EQ(0, kinematics_.InverseKinematic(0.0, 0.2, 0.0, -0.7, &result));
  EXPECT_NEAR(0.0, result.arm_velocity, 0.00001);
  EXPECT_NEAR(-0.7, result.elevator_velocity, 0.00001);
  // even with the arm out we should still just move up
  EXPECT_EQ(0, kinematics_.InverseKinematic(M_PI / 6, 0.2, 0.0, 0.7, &result));
  EXPECT_NEAR(0.0, result.arm_velocity, 0.00001);
  EXPECT_NEAR(0.7, result.elevator_velocity, 0.00001);
  // even with the arm back we should still just move up
  EXPECT_EQ(0, kinematics_.InverseKinematic(-M_PI / 6, 0.2, 0.0, 0.7, &result));
  EXPECT_NEAR(0.0, result.arm_velocity, 0.00001);
  EXPECT_NEAR(0.7, result.elevator_velocity, 0.00001);

  // should move only angle forward
  EXPECT_EQ(0, kinematics_.InverseKinematic(0.0, 0.2, 1.0, 0.0, &result));
  EXPECT_NEAR(-1.35759, result.arm_velocity, 0.00001);
  EXPECT_NEAR(0.0, result.elevator_velocity, 0.00001);
  // check the negative
  EXPECT_EQ(0, kinematics_.InverseKinematic(0.0, 0.2, -1.0, 0.0, &result));
  EXPECT_NEAR(1.35759, result.arm_velocity, 0.00001);
  EXPECT_NEAR(0.0, result.elevator_velocity, 0.00001);
  // with the arm out a change in x should make arm angle greater and
  // bring the evevator down.
  EXPECT_EQ(0, kinematics_.InverseKinematic(0.2, 0.2, 1.0, 0.0, &result));
  EXPECT_GT(0.0, result.arm_velocity);
  EXPECT_LT(0.0, result.elevator_velocity);
  // with the arm out a change in x should make arm angle greater and
  // bring the evevator down.
  EXPECT_EQ(0, kinematics_.InverseKinematic(-0.2, 0.2, 1.0, 0.0, &result));
  EXPECT_GT(0.0, result.arm_velocity);
  EXPECT_GT(0.0, result.elevator_velocity);
}

// Tests that velocity calulations are correct
TEST_F(KinematicsTest, ForwardKinematicVelocity) {
  ElevatorArmKinematics::KinematicResult result;

  // moving the arm forward at zero should result in x velocity
  EXPECT_EQ(0, kinematics_.ForwardKinematic(0.2, 0.0, 0.0, 1.35759, &result));
  EXPECT_NEAR(-1.0, result.fridge_x_velocity, 0.00001);
  EXPECT_NEAR(0.0, result.fridge_h_velocity, 0.00001);
  // check the negative
  EXPECT_EQ(0, kinematics_.ForwardKinematic(0.2, 0.0, 0.0, -1.35759, &result));
  EXPECT_NEAR(1.0, result.fridge_x_velocity, 0.00001);
  EXPECT_NEAR(0.0, result.fridge_h_velocity, 0.00001);
  // moving the arm up at zero should result in h velocity
  EXPECT_EQ(0, kinematics_.ForwardKinematic(0.2, 0.0, 1.0, 0.0, &result));
  EXPECT_NEAR(0.0, result.fridge_x_velocity, 0.00001);
  EXPECT_NEAR(1.0, result.fridge_h_velocity, 0.00001);
  // check the negative
  EXPECT_EQ(0, kinematics_.ForwardKinematic(0.2, 0.0, -1.0, 0.0, &result));
  EXPECT_NEAR(0.0, result.fridge_x_velocity, 0.00001);
  EXPECT_NEAR(-1.0, result.fridge_h_velocity, 0.00001);
  // arm is forward a negative angle should make x head forward and y head down.
  EXPECT_EQ(0, kinematics_.ForwardKinematic(0.5, -0.2, 0.0, -1.0, &result));
  EXPECT_GT(result.fridge_x_velocity, 0.0);
  EXPECT_LT(result.fridge_h_velocity, 0.0);
  // arm is forward a positive angle should make x head backwardward and y head up.
  EXPECT_EQ(0, kinematics_.ForwardKinematic(0.5, -0.2, 0.0, 1.0, &result));
  EXPECT_LT(result.fridge_x_velocity, 0.0);
  EXPECT_GT(result.fridge_h_velocity, 0.0);
  // arm is backward a negative angle should make x head forward and y head down.
  EXPECT_EQ(0, kinematics_.ForwardKinematic(0.5, 0.2, 0.0, -1.0, &result));
  EXPECT_GT(result.fridge_x_velocity, 0.0);
  EXPECT_GT(result.fridge_h_velocity, 0.0);
  // arm is backward a negative angle should make x head forward and y head down.
  EXPECT_EQ(0, kinematics_.ForwardKinematic(0.5, 0.2, 0.0, 1.0, &result));
  EXPECT_LT(result.fridge_x_velocity, 0.0);
  EXPECT_LT(result.fridge_h_velocity, 0.0);
}

}  // namespace testing
}  // namespace util
}  // namespace aos

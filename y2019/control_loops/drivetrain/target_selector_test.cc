#include "y2019/control_loops/drivetrain/target_selector.h"

#include "gtest/gtest.h"

namespace y2019 {
namespace control_loops {
namespace testing {

typedef ::frc971::control_loops::TypedPose<double> Pose;
typedef ::Eigen::Matrix<double, 5, 1> State;

namespace {
// Accessors to get some useful particular targets on the field:
Pose HPSlotLeft() { return constants::Field().targets()[7].pose(); }
Pose CargoNearLeft() { return constants::Field().targets()[2].pose(); }
Pose RocketPortalLeft() { return constants::Field().targets()[4].pose(); }
}  // namespace

// Tests the target selector with:
// -The [x, y, theta, left_vel, right_vel] state to test at
// -Whether we expect to see a target.
// -If (1) is true, the pose we expect to get back.
struct TestParams {
  State state;
  bool expect_target;
  Pose expected_pose;
};
class TargetSelectorParamTest : public ::testing::TestWithParam<TestParams> {};

TEST_P(TargetSelectorParamTest, ExpectReturn) {
  TargetSelector selector;
  bool expect_target = GetParam().expect_target;
  const State state = GetParam().state;
  ASSERT_EQ(expect_target, selector.UpdateSelection(state))
      << "We expected a return of " << expect_target << " at state "
      << state.transpose();
  if (expect_target) {
    const Pose expected_pose = GetParam().expected_pose;
    const Pose actual_pose = selector.TargetPose();
    const ::Eigen::Vector3d expected_pos = expected_pose.abs_pos();
    const ::Eigen::Vector3d actual_pos = actual_pose.abs_pos();
    const double expected_angle = expected_pose.abs_theta();
    const double actual_angle = actual_pose.abs_theta();
    EXPECT_EQ(expected_pos, actual_pos)
        << "Expected the pose to be at " << expected_pos.transpose()
        << " but got " << actual_pos.transpose() << " with the robot at "
        << state.transpose();
    EXPECT_EQ(expected_angle, actual_angle);
  }
}

INSTANTIATE_TEST_CASE_P(
    TargetSelectorTest, TargetSelectorParamTest,
    ::testing::Values(
        // When we are far away from anything, we should not register any
        // targets:
        TestParams{(State() << 0.0, 0.0, 0.0, 1.0, 1.0).finished(), false, {}},
        // Aim for a human-player spot; at low speeds we should not register
        // anything.
        TestParams{
            (State() << 4.0, 2.0, M_PI, 0.05, 0.05).finished(), false, {}},
        TestParams{
            (State() << 4.0, 2.0, M_PI, -0.05, -0.05).finished(), false, {}},
        TestParams{(State() << 4.0, 2.0, M_PI, 0.5, 0.5).finished(), true,
                   HPSlotLeft()},
        // Put ourselves between the rocket and cargo ship; we should see the
        // portal driving one direction and the near cargo ship port the other.
        TestParams{(State() << 6.0, 2.0, -M_PI_2, 0.5, 0.5).finished(), true,
                   CargoNearLeft()},
        TestParams{(State() << 6.0, 2.0, M_PI_2, -0.5, -0.5).finished(), true,
                   CargoNearLeft()},
        TestParams{(State() << 6.0, 2.0, -M_PI_2, -0.5, -0.5).finished(), true,
                   RocketPortalLeft()},
        TestParams{(State() << 6.0, 2.0, M_PI_2, 0.5, 0.5).finished(), true,
                   RocketPortalLeft()},
        // And we shouldn't see anything spinning in place:
        TestParams{
            (State() << 6.0, 2.0, M_PI_2, -0.5, 0.5).finished(), false, {}},
        // Drive backwards off the field--we should not see anything.
        TestParams{
            (State() << -0.1, 0.0, 0.0, -0.5, -0.5).finished(), false, {}}));

}  // namespace testing
}  // namespace control_loops
}  // namespace y2019

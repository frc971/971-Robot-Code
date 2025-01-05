#include "frc971/control_loops/swerve/linear_velocity_controller.h"

#include "gtest/gtest.h"

#include "aos/realtime.h"

namespace frc971::control_loops::swerve::test {
class LinearVelocityControllerTest : public ::testing::Test {
 protected:
  typedef LinearVelocityController::States States;
  typedef LinearVelocityController::Inputs Inputs;
  typedef LinearVelocityController::State State;
  typedef LinearVelocityController::Input Input;
  LinearVelocityControllerTest()
      : controller_(LinearVelocityController::MakeParameters(
            LinearVelocityController::ControllerWeights{
                .thetas_q = 1.0,
                .omegas_q = 1e-4,
                .vel_q = 20.0,
                .theta_q = 10.0,
                .omega_q = 10.0,
                .steer_current_r = 1e-4,
                .drive_current_r = 1e-3})) {}
  LinearVelocityController controller_;
};

// Checks that we output zero currents when our goal velocities are zero and we
// are at zero.
TEST_F(LinearVelocityControllerTest, ZeroAtZero) {
  auto result =
      controller_.RunRawController(State::Zero(), State::Zero(), Input::Zero());
  EXPECT_EQ(State::Zero(), result.debug.goal);
  EXPECT_EQ(Input::Zero(), result.U);
  result =
      controller_.RunController(State::Zero(), {.vx = 0, .vy = 0, .omega = 0});
  EXPECT_EQ(State::Zero(), result.debug.goal);
  EXPECT_EQ(Input::Zero(), result.U);
}

// Check that driving in a straight line results in reasonable dynamics.
TEST_F(LinearVelocityControllerTest, StraightLine) {
  auto result = controller_.RunController(State::Zero(),
                                          {.vx = 1.0, .vy = 0, .omega = 0});
  const State next_goal = result.debug.goal;
  EXPECT_EQ(1.0, result.debug.goal(States::kVx));
  result.debug.goal(States::kVx) = 0.0;
  EXPECT_EQ(State::Zero(), result.debug.goal);
  EXPECT_EQ(Input::Zero(), result.debug.U_ff);
  // Check that we have positive drive currents and zero steer currents.
  for (int module_index = 0; module_index < 4; ++module_index) {
    SCOPED_TRACE(module_index);
    EXPECT_EQ(0.0, result.U(Inputs::kIs0 + 2 * module_index));
    EXPECT_LT(1.0, result.U(Inputs::kId0 + 2 * module_index));
  }

  // Now get the simulated state up to speed; we should end up with zero
  // feedback current.
  {
    aos::ScopedRealtime realtime;
    result =
        controller_.RunController(next_goal, {.vx = 1.0, .vy = 0, .omega = 0});
  }
  EXPECT_EQ(1.0, result.debug.goal(States::kVx));
  result.debug.goal(States::kVx) = 0.0;
  EXPECT_EQ(State::Zero(), result.debug.goal);
  EXPECT_EQ(Input::Zero(), result.U);
  EXPECT_EQ(Input::Zero(), result.debug.U_ff);
  EXPECT_EQ(Input::Zero(), result.debug.U_feedback);
}

// Check that attempting to spin in place results in reasonable dynamics.
TEST_F(LinearVelocityControllerTest, SpinInPlace) {
  auto result = controller_.RunController(State::Zero(),
                                          {.vx = 0.0, .vy = 0, .omega = 1});
  State expected = State::Zero();
  // Technically it is permissible to end up with every single wheel at either
  // the specified value or +/- PI from the specified value. In the future we
  // may want to have something that biases us towards the current values of X.
  expected(States::kThetas0) = -M_PI_4;
  expected(States::kThetas1) = M_PI_4;
  expected(States::kThetas2) = -M_PI_4;
  expected(States::kThetas3) = M_PI_4;
  expected(States::kOmega) = 1.0;
  EXPECT_LT((expected - result.debug.goal).norm(), 5e-3)
      << "Expected:\n"
      << expected.transpose() << "\ngot:\n"
      << result.debug.goal.transpose();
  EXPECT_EQ(result.U, result.debug.U_ff + result.debug.U_feedback);
  EXPECT_LT(result.debug.U_ff.norm(), 1e-4) << result.debug.U_ff.transpose();

  const double nominal_drive_current = std::abs(result.U(Inputs::kId0));
  // All drive motors should have ~the same current; however, some of them are
  // driving backwards and so need signs flipped.
  // The steer motors should be getting driven in the direction required to get
  // them to the goal.
  for (std::pair<int, double> module : std::vector<std::pair<int, double>>{
           {0, -1.0}, {1, -1.0}, {2, 1.0}, {3, 1.0}}) {
    SCOPED_TRACE(module.first);
    if (expected(States::kThetas0 + 2 * module.first) > 0) {
      EXPECT_LT(1.0, result.U(Inputs::kIs0 + 2 * module.first));
    } else {
      EXPECT_GT(-1.0, result.U(Inputs::kIs0 + 2 * module.first));
    }

    EXPECT_NEAR(nominal_drive_current * module.second,
                result.U(Inputs::kId0 + 2 * module.first), 1e-4);
  }

  {
    aos::ScopedRealtime realtime;
    // Test when the wheels are already rotated, hence the goal state from the
    // goal of having a goal omega of 1
    result = controller_.RunController(result.debug.goal,
                                       {.vx = 0.0, .vy = 0, .omega = 1});
  }
  EXPECT_LT((expected - result.debug.goal).norm(), 5e-3)
      << "Expected:\n"
      << expected.transpose() << "\ngot:\n"
      << result.debug.goal.transpose();
  EXPECT_LT(result.U.norm(), 1e-4) << result.U.transpose();
  EXPECT_LT(result.debug.U_ff.norm(), 1e-4) << result.debug.U_ff.transpose();
  EXPECT_EQ(Input::Zero(), result.debug.U_feedback);
}

}  // namespace frc971::control_loops::swerve::test

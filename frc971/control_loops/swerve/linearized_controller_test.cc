#include "frc971/control_loops/swerve/linearized_controller.h"

#include "gtest/gtest.h"

namespace frc971::control_loops::swerve::test {

class LinearizedControllerTest : public ::testing::Test {
 protected:
  typedef LinearizedController<2> Controller;
  typedef Controller::State State;
  typedef Controller::Input Input;
  struct LinearDynamics : public Controller::Dynamics {
    Eigen::Vector2d operator()(
        const Eigen::Vector2d &X,
        const Eigen::Matrix<double, 8, 1> &U) const override {
      return Eigen::Matrix2d{{0.0, 1.0}, {0.0, -0.01}} * X +
             Eigen::Matrix<double, 2, 8>{
                 {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                 {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}} *
                 U;
    }
  };
  static Eigen::Matrix<double, kNumInputs, kNumInputs> MakeR() {
    Eigen::Matrix<double, kNumInputs, kNumInputs> R;
    R.setIdentity();
    return R;
  }
  LinearizedControllerTest()
      : controller_({.Q = Eigen::Matrix<double, 2, 2>{{1.0, 0.0}, {0.0, 1.0}},
                     .R = MakeR(),
                     .dt = std::chrono::milliseconds(10),
                     .dynamics = std::make_unique<LinearDynamics>()}) {}

  Controller controller_;
};

// Sanity check that the dynamics linearization is working correctly.
TEST_F(LinearizedControllerTest, LinearizedDynamics) {
  auto dynamics = controller_.LinearizeDynamics(State::Zero(), Input::Zero());
  EXPECT_EQ(0.0, dynamics.A(0, 0));
  EXPECT_EQ(0.0, dynamics.A(1, 0));
  EXPECT_EQ(1.0, dynamics.A(0, 1));
  EXPECT_EQ(-0.01, dynamics.A(1, 1));
  // All elements of B except for (1, 0) should be exactly 0.
  EXPECT_EQ(1.0, dynamics.B(1, 0));
  EXPECT_EQ(1.0, dynamics.B.norm());
}

// Confirm that the generated LQR controller is able to generate correct
// inputs when state and goal are at zero.
TEST_F(LinearizedControllerTest, ControllerResultAtZero) {
  auto result =
      controller_.RunController(State::Zero(), State::Zero(), Input::Zero());
  EXPECT_EQ(0.0, result.U.norm());
  EXPECT_EQ(0.0, result.debug.U_ff.norm());
  EXPECT_EQ(0.0, result.debug.U_feedback.norm());
}

// Confirm that the generated LQR controller is able to generate correct
// inputs when state is zero and the goal is non-zero.
TEST_F(LinearizedControllerTest, ControlToZero) {
  auto result = controller_.RunController(State::Zero(), State{{1.0}, {0.0}},
                                          Input::Zero());
  EXPECT_LT(0.0, result.U(0, 0));
  // All other U inputs should be zero.
  EXPECT_EQ(0.0, result.U.bottomRows<7>().norm());
  EXPECT_EQ(0.0, result.debug.U_ff.norm());
  EXPECT_EQ(0.0,
            (result.U - (result.debug.U_ff + result.debug.U_feedback)).norm());
}

// Confirm that the generated LQR controller is able to pass through the
// feedforwards when we have no difference between the goal and the current
// state.
TEST_F(LinearizedControllerTest, ControlToNonzeroState) {
  const State state{{1.0}, {1.0}};
  auto result = controller_.RunController(
      state, state,
      Input{{1.0}, {0.0}, {0.0}, {0.0}, {0.0}, {0.0}, {0.0}, {0.0}});
  EXPECT_EQ(1.0, result.U(0, 0));
  // All other U inputs should be zero.
  EXPECT_EQ(0.0, result.U.bottomRows<7>().norm());
  EXPECT_EQ(0.0, result.debug.U_feedback.norm());
  EXPECT_EQ(0.0,
            (result.U - (result.debug.U_ff + result.debug.U_feedback)).norm());
}
}  // namespace frc971::control_loops::swerve::test

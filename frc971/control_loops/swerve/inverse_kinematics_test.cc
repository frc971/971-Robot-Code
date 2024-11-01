#include "frc971/control_loops/swerve/inverse_kinematics.h"

#include "gtest/gtest.h"

namespace frc971::control_loops::swerve::testing {
class InverseKinematicsTest : public ::testing::Test {
 protected:
  typedef double Scalar;
  using State = InverseKinematics<Scalar>::State;
  using States = InverseKinematics<Scalar>::States;
  using ModuleParams = InverseKinematics<Scalar>::ModuleParams;
  using Parameters = InverseKinematics<Scalar>::Parameters;
  static ModuleParams MakeModule(const Eigen::Matrix<Scalar, 2, 1> &position) {
    return ModuleParams{.position = position,
                        .slip_angle_coefficient = 0.0,
                        .slip_angle_alignment_coefficient = 0.0,
                        .steer_motor = KrakenFOC(),
                        .drive_motor = KrakenFOC(),
                        .steer_ratio = 1.0,
                        .drive_ratio = 1.0,
                        .extra_steer_inertia = 0.0};
  }
  static Parameters MakeParams() {
    return {.mass = 1.0,
            .moment_of_inertia = 1.0,
            .modules = {
                MakeModule({1.0, 1.0}),
                MakeModule({-1.0, 1.0}),
                MakeModule({-1.0, -1.0}),
                MakeModule({1.0, -1.0}),
            }};
  }

  InverseKinematicsTest() : inverse_kinematics_(MakeParams()) {}

  struct Goal {
    Scalar vx;
    Scalar vy;
    Scalar omega;
    Scalar theta;
  };

  void CheckState(
      const Goal &goal, const std::array<Scalar, 4> &expected_thetas,
      const std::optional<Eigen::Vector4d> &expected_omegas = std::nullopt) {
    State goal_state = State::Zero();
    goal_state(States::kVx) = goal.vx;
    goal_state(States::kVy) = goal.vy;
    goal_state(States::kOmega) = goal.omega;
    goal_state(States::kTheta) = goal.theta;
    SCOPED_TRACE(goal_state.bottomRows<4>().transpose());
    const State nominal_state = inverse_kinematics_.Solve(goal_state);
    // Now, calculate the numerical derivative of the state and validate that it
    // matches expectations.
    const Scalar kDt = 1e-5;
    const Scalar dtheta = kDt * goal.omega;
    goal_state(States::kTheta) += dtheta / 2.0;
    const State state_eps_pos = inverse_kinematics_.Solve(goal_state);
    goal_state(States::kTheta) -= dtheta;
    const State state_eps_neg = inverse_kinematics_.Solve(goal_state);
    const State state_derivative = (state_eps_pos - state_eps_neg) / kDt;
    for (size_t module_index = 0; module_index < 4; ++module_index) {
      SCOPED_TRACE(module_index);
      const int omega_idx = States::kOmegas0 + 2 * module_index;
      const int theta_idx = States::kThetas0 + 2 * module_index;
      EXPECT_NEAR(nominal_state(omega_idx), state_derivative(theta_idx), 1e-10);
      EXPECT_NEAR(nominal_state(theta_idx), expected_thetas[module_index],
                  1e-10);
      if (expected_omegas.has_value()) {
        EXPECT_NEAR(nominal_state(omega_idx),
                    expected_omegas.value()(module_index), 1e-10);
      }
    }
  }

  InverseKinematics<Scalar> inverse_kinematics_;
};

// Tests that if we are driving straight with no yaw that we get sane
// kinematics.
TEST_F(InverseKinematicsTest, StraightDrivingNoYaw) {
  // Sanity-check zero-speed operation.
  CheckState({.vx = 0.0, .vy = 0.0, .omega = 0.0, .theta = 0.0},
             {0.0, 0.0, 0.0, 0.0}, Eigen::Vector4d::Zero());

  CheckState({.vx = 1.0, .vy = 0.0, .omega = 0.0, .theta = 0.0},
             {0.0, 0.0, 0.0, 0.0}, Eigen::Vector4d::Zero());
  // Reverse should prefer to bias the modules towards [-pi/2, pi/2] due to
  // hysteresis from starting the modules at thetas of 0.
  CheckState({.vx = -1.0, .vy = 0.0, .omega = 0.0, .theta = 0.0},
             {0.0, 0.0, 0.0, 0.0}, Eigen::Vector4d::Zero());

  CheckState({.vx = 0.0, .vy = 1.0, .omega = 0.0, .theta = 0.0},
             {M_PI_2, M_PI_2, M_PI_2, M_PI_2}, Eigen::Vector4d::Zero());
  // For module hysteresis, this is a corner case where we are exactly 90 deg
  // from the current value; the exact result is unimportant.
  CheckState({.vx = 0.0, .vy = -1.0, .omega = 0.0, .theta = 0.0},
             {-M_PI_2, -M_PI_2, -M_PI_2, -M_PI_2}, Eigen::Vector4d::Zero());

  CheckState({.vx = 1.0, .vy = 1.0, .omega = 0.0, .theta = 0.0},
             {M_PI_4, M_PI_4, M_PI_4, M_PI_4}, Eigen::Vector4d::Zero());
  // Reverse should prefer to bias the modules towards [-pi/2, pi/2] due to
  // hysteresis from starting the modules at thetas of 0.
  CheckState({.vx = -1.0, .vy = -1.0, .omega = 0.0, .theta = 0.0},
             {M_PI_4, M_PI_4, M_PI_4, M_PI_4}, Eigen::Vector4d::Zero());
}

// Tests that if we are driving straight with non-zero yaw that we get sane
// kinematics.
TEST_F(InverseKinematicsTest, StraightDrivingYawed) {
  CheckState({.vx = 1.0, .vy = 0.0, .omega = 0.0, .theta = 0.1},
             {-0.1, -0.1, -0.1, -0.1}, Eigen::Vector4d::Zero());

  CheckState({.vx = 1.0, .vy = 0.0, .omega = 0.0, .theta = M_PI_2},
             {-M_PI_2, -M_PI_2, -M_PI_2, -M_PI_2}, Eigen::Vector4d::Zero());
  CheckState({.vx = 1.0, .vy = 0.0, .omega = 0.0, .theta = M_PI},
             {0.0, 0.0, 0.0, 0.0}, Eigen::Vector4d::Zero());

  CheckState({.vx = 0.0, .vy = 1.0, .omega = 0.0, .theta = M_PI_2},
             {0.0, 0.0, 0.0, 0.0}, Eigen::Vector4d::Zero());
  // Reverse should prefer to bias the modules towards [-pi/2, pi/2] due to
  // hysteresis from starting the modules at thetas of 0.
  CheckState({.vx = 0.0, .vy = -1.0, .omega = 0.0, .theta = M_PI_2},
             {0.0, 0.0, 0.0, 0.0}, Eigen::Vector4d::Zero());
}

// Tests that we can spin in place.
TEST_F(InverseKinematicsTest, SpinInPlace) {
  CheckState({.vx = 0.0, .vy = 0.0, .omega = 1.0, .theta = 0.0},
             {-M_PI_4, M_PI_4, -M_PI_4, M_PI_4}, Eigen::Vector4d::Zero());
  // And changing the current theta should not matter.
  CheckState({.vx = 0.0, .vy = 0.0, .omega = 1.0, .theta = 1.0},
             {-M_PI_4, M_PI_4, -M_PI_4, M_PI_4}, Eigen::Vector4d::Zero());
}

// Tests that if we are spinning while moving that we correctly calculate module
// yaw rates.
TEST_F(InverseKinematicsTest, SpinWhileMoving) {
  // Set up a situation where we are driving straight forwards, with a
  // yaw rate of 1 rad / sec.
  // The modules are all at radii of sqrt(2), so the contribution from both
  // the translational and rotational velocities should be equal; in this case
  // each module will have an angle that is an equal combination of straight
  // forwards (0 deg) and some 45 deg offset.
  CheckState({.vx = std::sqrt(static_cast<Scalar>(2)),
              .vy = 0.0,
              .omega = 1.0,
              .theta = 0.0},
             {3 * M_PI_4 / 2, -3 * M_PI_4 / 2, -M_PI_4 / 2, M_PI_4 / 2});
}
}  // namespace frc971::control_loops::swerve::testing

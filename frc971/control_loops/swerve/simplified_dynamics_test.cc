#include "frc971/control_loops/swerve/simplified_dynamics.h"

#include <functional>

#include "absl/log/log.h"
#include "gtest/gtest.h"

#include "aos/time/time.h"
#include "frc971/control_loops/jacobian.h"

namespace frc971::control_loops::swerve::testing {
class SimplifiedDynamicsTest : public ::testing::Test {
 protected:
  using Dynamics = SimplifiedDynamics<double>;
  using PositionState = Dynamics::PositionState<double>;
  using States = Dynamics::States;
  using Inputs = Dynamics::Inputs;
  using Input = Dynamics::Input<double>;
  using ModuleParams = Dynamics::ModuleParams;
  using Parameters = Dynamics::Parameters;
  static ModuleParams MakeModule(const Eigen::Vector2d &position,
                                 bool wheel_alignment) {
    return ModuleParams{
        .position = position,
        .slip_angle_coefficient = 200.0,
        .slip_angle_alignment_coefficient = wheel_alignment ? 1.0 : 0.0,
        .steer_motor = KrakenFOC(),
        .drive_motor = KrakenFOC(),
        .steer_ratio = 0.1,
        .drive_ratio = 0.01,
        .extra_steer_inertia = 0.0};
  }
  static Parameters MakeParams(bool wheel_alignment) {
    return {.mass = 60,
            .moment_of_inertia = 2,
            .modules =
                {
                    MakeModule({1.0, 1.0}, wheel_alignment),
                    MakeModule({-1.0, 1.0}, wheel_alignment),
                    MakeModule({-1.0, -1.0}, wheel_alignment),
                    MakeModule({1.0, -1.0}, wheel_alignment),
                },
            .accel_weight = 0.0};
  }
  SimplifiedDynamicsTest() : dynamics_(MakeParams(false)) {}

  PositionState ValidateDynamics(const PositionState &state,
                                 const Input &input) {
    const PositionState Xdot = dynamics_.Dynamics(state, input);
    // Sanity check simple invariants:
    EXPECT_EQ(Xdot(Dynamics::kX), state(Dynamics::kVx));
    EXPECT_EQ(Xdot(Dynamics::kY), state(Dynamics::kVy));
    EXPECT_EQ(Xdot(Dynamics::kTheta), state(Dynamics::kOmega));

    // Check that the dynamics linearization produces numbers that match numeric
    // differentiation of the dynamics.
    aos::monotonic_clock::time_point start_time = aos::monotonic_clock::now();
    const auto linearized_dynamics = dynamics_.LinearizedDynamics(state, input);
    const aos::monotonic_clock::duration auto_diff_time =
        aos::monotonic_clock::now() - start_time;

    start_time = aos::monotonic_clock::now();
    auto numerical_A = NumericalJacobianX(
        std::bind(&Dynamics::Dynamics<double>, &dynamics_,
                  std::placeholders::_1, std::placeholders::_2),
        state, input);
    auto numerical_B = NumericalJacobianU(
        std::bind(&Dynamics::Dynamics<double>, &dynamics_,
                  std::placeholders::_1, std::placeholders::_2),
        state, input);
    const aos::monotonic_clock::duration numerical_time =
        aos::monotonic_clock::now() - start_time;
    VLOG(1) << "Autodifferentiation took " << auto_diff_time
            << " while numerical differentiation took " << numerical_time;
    EXPECT_LT((numerical_A - linearized_dynamics.first).norm(), 1e-6)
        << "Numerical result:\n"
        << numerical_A << "\nAuto-diff result:\n"
        << linearized_dynamics.first;
    EXPECT_LT((numerical_B - linearized_dynamics.second).norm(), 1e-6)
        << "Numerical result:\n"
        << numerical_B << "\nAuto-diff result:\n"
        << linearized_dynamics.second;
    return Xdot;
  }

  Dynamics dynamics_;
};

// Test that if all states and inputs are at zero that the robot won't move.
TEST_F(SimplifiedDynamicsTest, ZeroIsZero) {
  EXPECT_EQ(PositionState::Zero(),
            ValidateDynamics(PositionState::Zero(), Input::Zero()));
}

// Test that if we are travelling straight forwards with zero inputs that we
// just coast.
TEST_F(SimplifiedDynamicsTest, CoastForwards) {
  PositionState state = PositionState::Zero();
  state(States::kVx) = 1.0;
  PositionState expected = PositionState::Zero();
  expected(States::kX) = 1.0;
  EXPECT_EQ(expected, ValidateDynamics(state, Input::Zero()));
}

// Tests that we can accelerate the robot.
TEST_F(SimplifiedDynamicsTest, AccelerateStraight) {
  // Check that the drive currents behave as anticipated and accelerate the
  // robot.
  Input input{{0.0}, {1.0}, {0.0}, {1.0}, {0.0}, {1.0}, {0.0}, {1.0}};
  PositionState state = PositionState::Zero();
  state(States::kVx) = 0.0;
  PositionState result = ValidateDynamics(state, input);
  EXPECT_EQ(result.norm(), result(States::kVx));
  EXPECT_LT(0.1, result(States::kVx));
}

// Test that if we are driving straight sideways (so our wheel are at 90
// degrees) that we experience a force slowing us down.
TEST_F(SimplifiedDynamicsTest, ForceWheelsSideways) {
  PositionState state = PositionState::Zero();
  state(States::kVy) = 1.0;
  PositionState result = ValidateDynamics(state, Input::Zero());
  EXPECT_LT(result.topRows<States::kVy>().norm(), 1e-10)
      << ": All derivatives prior to the vy state should be ~zero.\n"
      << result;
  EXPECT_LT(result(States::kVy), -1.0)
      << ": expected non-trivial deceleration.";
  EXPECT_EQ(result(States::kOmega), 0.0);
}

// Tests that we can make the robot spin in place by orienting all the wheels
TEST_F(SimplifiedDynamicsTest, SpinInPlaceNoSlip) {
  PositionState state = PositionState::Zero();
  state(States::kThetas0) = 3.0 * M_PI / 4.0;
  state(States::kThetas1) = 5.0 * M_PI / 4.0;
  state(States::kThetas2) = 7.0 * M_PI / 4.0;
  state(States::kThetas3) = 1.0 * M_PI / 4.0;
  state(States::kOmega) = 1.0;
  PositionState result = ValidateDynamics(state, Input::Zero());
  EXPECT_NEAR(result.norm(), 1.0, 1e-10)
      << ": Only non-zero state should be kTheta, which should be exactly 1.0.";
  EXPECT_EQ(result(States::kTheta), 1.0);

  // Sanity check that when we then apply drive torque to the wheels that that
  // apins the robot more.
  Input input{{0.0}, {1.0}, {0.0}, {1.0}, {0.0}, {1.0}, {0.0}, {1.0}};
  result = ValidateDynamics(state, input);
  EXPECT_EQ(result(States::kTheta), 1.0);
  EXPECT_LT(1.0, result(States::kOmega));
  // Everything else should be ~zero.
  result(States::kTheta) = 0.0;
  result(States::kOmega) = 0.0;
  EXPECT_LT(result.norm(), 1e-10);
}

// Tests that we can spin in place when skid-steering (i.e., all wheels stay
// pointed straight, but we still attempt to spint he robot).
TEST_F(SimplifiedDynamicsTest, SpinInPlaceSkidSteer) {
  PositionState state = PositionState::Zero();
  state(States::kThetas0) = -M_PI;
  state(States::kThetas1) = -M_PI;
  state(States::kThetas2) = 0.0;
  state(States::kThetas3) = 0.0;
  state(States::kOmega) = 1.0;
  PositionState result = ValidateDynamics(state, Input::Zero());
  EXPECT_EQ(result(States::kTheta), 1.0);
  EXPECT_LT(result(States::kOmega), -1.0)
      << "We should be aggressively decelerrating when slipping wheels.";
  // Everything else should be ~zero.
  result(States::kTheta) = 0.0;
  result(States::kOmega) = 0.0;
  EXPECT_LT(result.norm(), 1e-10);

  // Sanity check that when we then apply drive torque to the wheels that that
  // we can counteract the spin.
  Input input{{0.0}, {100.0}, {0.0}, {100.0}, {0.0}, {100.0}, {0.0}, {100.0}};
  result = ValidateDynamics(state, input);
  EXPECT_EQ(result(States::kTheta), 1.0);
  EXPECT_LT(1.0, result(States::kOmega));
  // Everything else should be ~zero.
  result(States::kTheta) = 0.0;
  result(States::kOmega) = 0.0;
  EXPECT_LT(result.norm(), 1e-10);
}

// Tests that we can spin in place when skid-steering backwards (ensures that
// slip angle calculations and the such handle the sign changes correctly).
TEST_F(SimplifiedDynamicsTest, SpinInPlaceSkidSteerBackwards) {
  PositionState state = PositionState::Zero();
  state(States::kThetas0) = 0.0;
  state(States::kThetas1) = 0.0;
  state(States::kThetas2) = M_PI;
  state(States::kThetas3) = M_PI;
  state(States::kOmega) = 1.0;
  PositionState result = ValidateDynamics(state, Input::Zero());
  EXPECT_EQ(result(States::kTheta), 1.0);
  EXPECT_LT(result(States::kOmega), -1.0)
      << "We should be aggressively decelerrating when slipping wheels.";
  // Everything else should be ~zero.
  result(States::kTheta) = 0.0;
  result(States::kOmega) = 0.0;
  EXPECT_LT(result.norm(), 1e-10);

  // Sanity check that when we then apply drive torque to the wheels that that
  // we can counteract the spin.
  Input input{{0.0}, {-100.0}, {0.0}, {-100.0},
              {0.0}, {-100.0}, {0.0}, {-100.0}};
  result = ValidateDynamics(state, input);
  EXPECT_EQ(result(States::kTheta), 1.0);
  EXPECT_LT(1.0, result(States::kOmega));
  // Everything else should be ~zero.
  result(States::kTheta) = 0.0;
  result(States::kOmega) = 0.0;
  EXPECT_LT(result.norm(), 1e-10);
}

// Test that if we turn on the wheel alignment forces that it results in forces
// that cause the wheels to align to straight over time.
TEST_F(SimplifiedDynamicsTest, WheelAlignmentForces) {
  dynamics_ = Dynamics(MakeParams(true));
  PositionState state = PositionState::Zero();
  state(States::kThetas0) = -0.1;
  state(States::kThetas1) = -0.1;
  state(States::kThetas2) = -0.1;
  state(States::kThetas3) = -0.1;
  state(States::kVx) = 1.0;
  PositionState result = ValidateDynamics(state, Input::Zero());
  EXPECT_LT(1e-2, result(States::kOmegas0));
  EXPECT_LT(1e-2, result(States::kOmegas1));
  EXPECT_LT(1e-2, result(States::kOmegas2));
  EXPECT_LT(1e-2, result(States::kOmegas3));
}

// Do some fuzz testing of the jacobian calculations.
TEST_F(SimplifiedDynamicsTest, Fuzz) {
  for (size_t state_index = 0; state_index < States::kNumPositionStates;
       ++state_index) {
    SCOPED_TRACE(state_index);
    PositionState state = PositionState::Zero();
    for (const double value : {-1.0, 0.0, 1.0}) {
      SCOPED_TRACE(value);
      state(state_index) = value;
      ValidateDynamics(state, Input::Zero());
    }
  }
}
}  // namespace frc971::control_loops::swerve::testing

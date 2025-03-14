#include "frc971/control_loops/swerve/linear_velocity_controller.h"

#include <memory>

#include "absl/flags/flag.h"

#include "linear_velocity_controller.h"
#include "nonlinear_cost.h"
#include "simplified_dynamics.h"
ABSL_FLAG(double, thetas_q, 1.0, "");
ABSL_FLAG(double, omegas_q, 1e-4, "");
ABSL_FLAG(double, is_r, 1e-5, "");
ABSL_FLAG(double, is_t, 1e-3, "");
ABSL_FLAG(double, vel_q, 20.0, "");
ABSL_FLAG(double, omega_q, 3.0, "");

namespace frc971::control_loops::swerve {

LinearVelocityController::Parameters LinearVelocityController::MakeParameters(
    const ControllerWeights weights,
    const LinearVelocityController::DynamicsParameters &params) {
  StateSquare Q = StateSquare::Zero();
  // We don't really care much about the actual angles of the swerve modules,
  // but make them non-zero to help guide things.
  Q.diagonal()(States::kThetas0) = weights.thetas_q;
  Q.diagonal()(States::kThetas1) = weights.thetas_q;
  Q.diagonal()(States::kThetas2) = weights.thetas_q;
  Q.diagonal()(States::kThetas3) = weights.thetas_q;
  Q.diagonal()(States::kOmegas0) = weights.omegas_q;
  Q.diagonal()(States::kOmegas1) = weights.omegas_q;
  Q.diagonal()(States::kOmegas2) = weights.omegas_q;
  Q.diagonal()(States::kOmegas3) = weights.omegas_q;
  Q.diagonal()(States::kOmegad0) = weights.omegad_q;
  Q.diagonal()(States::kOmegad1) = weights.omegad_q;
  Q.diagonal()(States::kOmegad2) = weights.omegad_q;
  Q.diagonal()(States::kOmegad3) = weights.omegad_q;
  Q.diagonal()(States::kTheta) = weights.theta_q;
  Q.diagonal()(States::kVx) = weights.vel_q;
  Q.diagonal()(States::kVy) = weights.vel_q;
  Q.diagonal()(States::kOmega) = weights.omega_q;

  InputSquare R = InputSquare::Zero();
  for (size_t index = 0; index < 4; ++index) {
    R.diagonal()(2 * index) = weights.steer_current_r;
    R.diagonal()(2 * index + 1) = weights.drive_current_r;
  }

  if (weights.k_slip != 0.0) {
    return Parameters{
        .Q = Q,
        .nonlinear_Q = std::optional{std::make_unique<
            NonLinearCost<Dynamics::States::kNumVelocityStates, Scalar>>(
            NonLinearCost<Dynamics::States::kNumVelocityStates, Scalar>(
                weights.k_slip, params))},
        .R = R,
        .dt = kDt,
        .dynamics = std::make_unique<VirtualDynamics>(params)};
  }

  return Parameters{.Q = Q,
                    .R = R,
                    .dt = kDt,
                    .dynamics = std::make_unique<VirtualDynamics>(params)};
}

namespace {
LinearVelocityController::Dynamics::ModuleParams MakeModule(
    const Eigen::Matrix<LinearVelocityController::Scalar, 2, 1> &position) {
  return {.position = position,
          .slip_angle_coefficient = 200.0,
          .slip_angle_alignment_coefficient = 10.0,
          .steer_motor = KrakenFOC(),
          .drive_motor = KrakenFOC(),
          .steer_ratio = 0.1,
          .drive_ratio = 0.01,
          .wheel_radius = 1.0,
          .extra_steer_inertia = 0.0};
}
}  // namespace

LinearVelocityController::DynamicsParameters
LinearVelocityController::MakeDynamicsParameters() {
  return {.mass = 60,
          .moment_of_inertia = 2,
          .modules =
              {
                  MakeModule({1.0, 1.0}),
                  MakeModule({-1.0, 1.0}),
                  MakeModule({-1.0, -1.0}),
                  MakeModule({1.0, -1.0}),
              },
          .accel_weight = 0.0};
}

LinearVelocityController::LinearVelocityController(
    Parameters params, const DynamicsParameters &dynamics_params)
    : inverse_kinematics_(dynamics_params), controller_(std::move(params)) {}

LinearVelocityController::State LinearVelocityController::MakeGoal(
    const Goal &goal) {
  State state = State::Zero();
  state(States::kVx) = goal.vx;
  state(States::kVy) = goal.vy;
  state(States::kOmega) = goal.omega;
  return state;
}

LinearVelocityController::ControllerResult
LinearVelocityController::RunController(const State &X, const Goal &goal) {
  return RunRawController(X, inverse_kinematics_.Solve(MakeGoal(goal)),
                          Input::Zero());
}

LinearVelocityController::ControllerResult
LinearVelocityController::RunRawController(const State &X, const State &goal,
                                           const Input &U_ff) {
  Controller::ControllerResult result =
      controller_.RunController(X, goal, U_ff);
  return {
      .U = result.U,
      .debug = {.U_ff = result.debug.U_ff,
                .U_feedback = result.debug.U_feedback,
                .feedback_contributions = result.debug.feedback_contributions,
                .goal = goal,
                .sb02od_exit_code = result.debug.sb02od_exit_code}};
}
}  // namespace frc971::control_loops::swerve

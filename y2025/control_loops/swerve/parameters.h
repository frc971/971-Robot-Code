#include "frc971/control_loops/swerve/simplified_dynamics.h"
#include "y2025/constants.h"

namespace y2025::control_loops {

using frc971::control_loops::swerve::LinearVelocityController;

LinearVelocityController::ControllerWeights CreateWeights(
    const y2025::VelocityControllerWeights *weights) {
  return LinearVelocityController::ControllerWeights{
      .thetas_q = weights->thetas_q(),
      .omegas_q = weights->omegas_q(),
      .vel_q = weights->vel_q(),
      .theta_q = weights->theta_q(),
      .omega_q = weights->omega_q(),
      .steer_current_r = weights->steer_current_r(),
      .drive_current_r = weights->drive_current_r()};
}

template <typename Scalar>
frc971::control_loops::swerve::SimplifiedDynamics<Scalar>::Parameters
MakeSwerveParameters() {
  auto make_module = [](const Eigen::Matrix<Scalar, 2, 1> &position) {
    return frc971::control_loops::swerve::LinearVelocityController::Dynamics::
        ModuleParams{
            .position = position,
            .slip_angle_coefficient = 0.0,
            .slip_angle_alignment_coefficient = 0.0,
            .steer_motor = frc971::control_loops::swerve::KrakenFOC(),
            .drive_motor = frc971::control_loops::swerve::KrakenFOC(),
            .steer_ratio = constants::Values::kRotationModuleRatio,
            .drive_ratio = constants::Values::kTranslationModuleRatio(),
            .extra_steer_inertia = 0.01};
  };

  constexpr Scalar kSideLength = 0.635;
  return {
      .mass = 35,
      .moment_of_inertia = 2.63,
      .modules =
          {
              // front left
              make_module({kSideLength / 2.0, kSideLength / 2.0}),
              // front right
              make_module({kSideLength / 2.0, -kSideLength / 2.0}),
              // back left
              make_module({-kSideLength / 2.0, kSideLength / 2.0}),
              // back right
              make_module({-kSideLength / 2.0, -kSideLength / 2.0}),
          },
      .accel_weight = 0.0,
  };
}
}  // namespace y2025::control_loops

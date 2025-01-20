#include "frc971/control_loops/swerve/simplified_dynamics.h"
#include "y2024_swerve/constants.h"
namespace y2024_swerve::control_loops {
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
            .steer_ratio = constants::Values::kRotationModuleRatio(),
            .drive_ratio = constants::Values::kTranslationModuleRatio(),
            .extra_steer_inertia = 0.01};
  };

  constexpr Scalar kSideLength = 0.635;
  return {
      .mass = 60,
      .moment_of_inertia = 2,
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
}  // namespace y2024_swerve::control_loops

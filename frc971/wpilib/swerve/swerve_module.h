#ifndef FRC971_WPILIB_SWERVE_SWERVE_MODULE_H_
#define FRC971_WPILIB_SWERVE_SWERVE_MODULE_H_

#include "frc971/wpilib/falcon.h"

namespace frc971 {
namespace wpilib {
namespace swerve {

struct SwerveModule {
  SwerveModule(FalconParams rotation_params, FalconParams translation_params,
               std::string canbus,
               std::vector<ctre::phoenix6::BaseStatusSignal *> *signals,
               double stator_current_limit, double supply_current_limit)
      : rotation(std::make_shared<Falcon>(rotation_params, canbus, signals,
                                          stator_current_limit,
                                          supply_current_limit)),
        translation(std::make_shared<Falcon>(translation_params, canbus,
                                             signals, stator_current_limit,
                                             supply_current_limit)) {}

  void WriteModule(
      const frc971::control_loops::drivetrain::swerve::SwerveModuleOutput
          *module_output,
      double max_voltage) {
    double rotation_current = 0.0;
    double translation_current = 0.0;

    if (module_output != nullptr) {
      rotation_current = module_output->rotation_current();
      translation_current = module_output->translation_current();
    }

    rotation->WriteCurrent(rotation_current, max_voltage);
    translation->WriteCurrent(translation_current, max_voltage);
  }

  std::shared_ptr<Falcon> rotation;
  std::shared_ptr<Falcon> translation;
};

}  // namespace swerve
}  // namespace wpilib
}  // namespace frc971
#endif  // FRC971_WPILIB_SWERVE_SWERVE_MODULE_H_

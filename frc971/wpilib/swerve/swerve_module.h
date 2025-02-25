#ifndef FRC971_WPILIB_SWERVE_SWERVE_MODULE_H_
#define FRC971_WPILIB_SWERVE_SWERVE_MODULE_H_

#include "frc971/control_loops/swerve/swerve_drivetrain_can_position_static.h"
#include "frc971/control_loops/swerve/swerve_drivetrain_output_generated.h"
#include "frc971/wpilib/talonfx.h"

namespace frc971::wpilib::swerve {

// Contains the objects for interacting with the hardware for a given swerve
// module, assuming that the module uses two TalonFX-based motor controllers.
struct SwerveModule {
  SwerveModule(TalonFXParams rotation_params, TalonFXParams translation_params,
               std::string canbus,
               std::vector<ctre::phoenix6::BaseStatusSignal *> *signals,
               double stator_current_limit, double supply_current_limit,
               double steer_stator_current_limit,
               double steer_supply_current_limit)
      : rotation(std::make_shared<TalonFX>(rotation_params, canbus, signals,
                                           steer_stator_current_limit,
                                           steer_supply_current_limit)),
        translation(std::make_shared<TalonFX>(translation_params, canbus,
                                              signals, stator_current_limit,
                                              supply_current_limit)) {}
  SwerveModule(TalonFXParams rotation_params, TalonFXParams translation_params,
               std::string canbus,
               std::vector<ctre::phoenix6::BaseStatusSignal *> *signals,
               double stator_current_limit, double supply_current_limit)
      : rotation(std::make_shared<TalonFX>(rotation_params, canbus, signals,
                                           stator_current_limit,
                                           supply_current_limit)),
        translation(std::make_shared<TalonFX>(translation_params, canbus,
                                              signals, stator_current_limit,
                                              supply_current_limit)) {}

  // Writes the requested torque currents from the module_output to the motors,
  // setting the maximum voltage of the motor outputs to the requested value.
  void WriteModule(
      const frc971::control_loops::swerve::SwerveModuleOutput *module_output,
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

  struct ModuleGearRatios {
    double rotation;
    double translation;
  };
  // Populates a CAN-position message with the CAN-based devices (currently,
  // just the motors themselves).
  // Scales the motors' position values by the provided gear ratios.
  void PopulateCanPosition(
      frc971::control_loops::swerve::SwerveModuleCanPositionStatic
          *can_position,
      const ModuleGearRatios &ratios) {
    rotation->SerializePosition(can_position->add_rotation(), ratios.rotation);
    translation->SerializePosition(can_position->add_translation(),
                                   ratios.translation);
  }

  std::shared_ptr<TalonFX> rotation;
  std::shared_ptr<TalonFX> translation;
};

// Represents all the modules in a swerve drivetrain.
struct SwerveModules {
  void PopulateFalconsVector(std::vector<std::shared_ptr<TalonFX>> *falcons) {
    CHECK(falcons != nullptr);
    falcons->push_back(front_left->rotation);
    falcons->push_back(front_left->translation);

    falcons->push_back(front_right->rotation);
    falcons->push_back(front_right->translation);

    falcons->push_back(back_left->rotation);
    falcons->push_back(back_left->translation);

    falcons->push_back(back_right->rotation);
    falcons->push_back(back_right->translation);
  }

  std::shared_ptr<SwerveModule> front_left;
  std::shared_ptr<SwerveModule> front_right;
  std::shared_ptr<SwerveModule> back_left;
  std::shared_ptr<SwerveModule> back_right;
};

}  // namespace frc971::wpilib::swerve
#endif  // FRC971_WPILIB_SWERVE_SWERVE_MODULE_H_

#ifndef y2020_CONSTANTS_H_
#define y2020_CONSTANTS_H_

#include <math.h>
#include <stdint.h>

#include <array>

#include "frc971/constants.h"
#include "frc971/control_loops/pose.h"
#include "frc971/control_loops/static_zeroing_single_dof_profiled_subsystem.h"
#include "y2020/control_loops/drivetrain/drivetrain_dog_motor_plant.h"
#include "y2020/control_loops/superstructure/accelerator/accelerator_plant.h"
#include "y2020/control_loops/superstructure/control_panel/control_panel_plant.h"
#include "y2020/control_loops/superstructure/finisher/finisher_plant.h"
#include "y2020/control_loops/superstructure/hood/hood_plant.h"
#include "y2020/control_loops/superstructure/intake/intake_plant.h"
#include "y2020/control_loops/superstructure/turret/turret_plant.h"

namespace y2020 {
namespace constants {

struct Values {
  static const uint16_t kCodingRobotTeamNumber = 7971;

  static const int kZeroingSampleSize = 200;

  static constexpr double kDrivetrainCyclesPerRevolution() { return 512.0; }
  static constexpr double kDrivetrainEncoderCountsPerRevolution() {
    return kDrivetrainCyclesPerRevolution() * 4;
  }
  static constexpr double kDrivetrainEncoderRatio() { return 1.0; }
  static constexpr double kMaxDrivetrainEncoderPulsesPerSecond() {
    return control_loops::drivetrain::kFreeSpeed / (2.0 * M_PI) *
           control_loops::drivetrain::kHighGearRatio /
           constants::Values::kDrivetrainEncoderRatio() *
           kDrivetrainEncoderCountsPerRevolution();
  }

  // Hood
  static constexpr double kHoodEncoderCountsPerRevolution() { return 4096.0; }

  static constexpr double kHoodEncoderRatio() { return 8.0 / 72.0; }

  static constexpr double kMaxHoodEncoderPulsesPerSecond() {
    return control_loops::superstructure::hood::kFreeSpeed / (2.0 * M_PI) *
           control_loops::superstructure::hood::kOutputRatio /
           kHoodEncoderRatio() * kHoodEncoderCountsPerRevolution();
  }

  static constexpr ::frc971::constants::Range kHoodRange() {
    return ::frc971::constants::Range{
        -0.01,  // Back Hard
        0.65,   // Front Hard
        0.0,    // Back Soft
        0.64    // Front Soft
    };
  }

  ::frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemParams<
      ::frc971::zeroing::AbsoluteEncoderZeroingEstimator>
      hood;

  // Intake
  static constexpr double kIntakeEncoderCountsPerRevolution() { return 4096.0; }

  static constexpr double kIntakeEncoderRatio() { return (16.0 / 32.0); }

  static constexpr double kMaxIntakeEncoderPulsesPerSecond() {
    return control_loops::superstructure::intake::kFreeSpeed / (2.0 * M_PI) *
           control_loops::superstructure::intake::kOutputRatio /
           kIntakeEncoderRatio() * kIntakeEncoderCountsPerRevolution();
  }

  // TODO(sabina): update range
  static constexpr ::frc971::constants::Range kIntakeRange() {
    return ::frc971::constants::Range{
        -1,     // Back Hard
        1,      // Front Hard
        -0.95,  // Back Soft
        0.95    // Front Soft
    };
  }

  ::frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemParams<
      ::frc971::zeroing::AbsoluteEncoderZeroingEstimator>
      intake;

  static constexpr double kIntakeRollerSupplyCurrentLimit() { return 30.0; }
  static constexpr double kIntakeRollerStatorCurrentLimit() { return 40.0; }

  // Turret
  static constexpr double kTurretEncoderCountsPerRevolution() { return 4096.0; }

  static constexpr double kTurretEncoderRatio() {
    return (26.0 / 150.0) * (130.0 / 40.0);
  }

  static constexpr double kMaxTurretEncoderPulsesPerSecond() {
    return control_loops::superstructure::turret::kFreeSpeed / (2.0 * M_PI) *
           control_loops::superstructure::turret::kOutputRatio /
           kTurretEncoderRatio() * kTurretEncoderCountsPerRevolution();
  }

  static constexpr double kTurretPotRatio() { return (26.0 / 150.0); }

  static constexpr ::frc971::constants::Range kTurretRange() {
    return ::frc971::constants::Range{
        // TODO (Kai): Placeholders right now.
        -3.2,   // Back Hard
        3.2,    // Front Hard
        -3.14,  // Back Soft
        3.14    // Front Soft
    };
  }

  struct PotAndAbsEncoderConstants {
    ::frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemParams<
        ::frc971::zeroing::PotAndAbsoluteEncoderZeroingEstimator>
        subsystem_params;
    double potentiometer_offset;
  };

  PotAndAbsEncoderConstants turret;

  // Control Panel

  // Mag encoder
  static constexpr double kControlPanelEncoderCountsPerRevolution() {
    return 4096.0;
  }

  // Ratio is encoder to output
  static constexpr double kControlPanelEncoderRatio() { return (56.0 / 28.0); }

  static constexpr double kMaxControlPanelEncoderPulsesPerSecond() {
    return control_loops::superstructure::control_panel::kFreeSpeed /
           (2.0 * M_PI) *
           control_loops::superstructure::control_panel::kOutputRatio /
           kControlPanelEncoderRatio() *
           kControlPanelEncoderCountsPerRevolution();
  }

  // Shooter
  static constexpr double kFinisherEncoderCountsPerRevolution() {
    return 4096.0;
  }
  static constexpr double kFinisherEncoderRatio() { return 30.0 / 40.0; }

  static constexpr double kMaxFinisherEncoderPulsesPerSecond() {
    return control_loops::superstructure::finisher::kFreeSpeed / (2.0 * M_PI) *
           control_loops::superstructure::finisher::kOutputRatio /
           kFinisherEncoderRatio() * kFinisherEncoderCountsPerRevolution();
  }


  static constexpr double kAcceleratorEncoderCountsPerRevolution() {
    return 4096.0;
  }
  static constexpr double kAcceleratorEncoderRatio() {
    return (1.2 * 1.2 * 1.2) * (30.0 / 40.0);
  }

  static constexpr double kMaxAcceleratorEncoderPulsesPerSecond() {
    return control_loops::superstructure::accelerator::kFreeSpeed /
           (2.0 * M_PI) *
           control_loops::superstructure::accelerator::kOutputRatio /
           kAcceleratorEncoderRatio() *
           kAcceleratorEncoderCountsPerRevolution();
  }

  // Climber
  static constexpr double kClimberSupplyCurrentLimit() { return 60.0; }
};

// Creates (once) a Values instance for ::aos::network::GetTeamNumber() and
// returns a reference to it.
const Values &GetValues();

// Creates Values instances for each team number it is called with and
// returns them.
const Values &GetValuesForTeam(uint16_t team_number);

}  // namespace constants
}  // namespace y2020

#endif  // y2020_CONSTANTS_H_

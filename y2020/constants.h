#ifndef y2020_CONSTANTS_H_
#define y2020_CONSTANTS_H_

#include <array>
#include <cmath>
#include <cstdint>

#include "frc971/constants.h"
#include "frc971/control_loops/static_zeroing_single_dof_profiled_subsystem.h"
#include "frc971/shooter_interpolation/interpolation.h"
#include "y2020/control_loops/drivetrain/drivetrain_dog_motor_plant.h"
#include "y2020/control_loops/superstructure/accelerator/accelerator_plant.h"
#include "y2020/control_loops/superstructure/control_panel/control_panel_plant.h"
#include "y2020/control_loops/superstructure/finisher/finisher_plant.h"
#include "y2020/control_loops/superstructure/hood/hood_plant.h"
#include "y2020/control_loops/superstructure/intake/intake_plant.h"
#include "y2020/control_loops/superstructure/turret/turret_plant.h"

using ::frc971::shooter_interpolation::InterpolationTable;

namespace y2020 {
namespace constants {

struct Values {
  static const uint16_t kCompTeamNumber = 971;
  static const uint16_t kPracticeTeamNumber = 9971;
  static const uint16_t kCodingRobotTeamNumber = 7971;
  static const uint16_t kSpareRoborioTeamNumber = 6971;

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

  static constexpr double kHoodEncoderRatio() {
    // TODO: This math is not quite right
    // 10.211 in of travel gets you 1 radian on the output
    const double radians_per_in_travel = 1.0 / 10.211;

    // one turn on the leadscrew gets you 0.5 in travel
    const double in_travel_per_radian = 0.5 / (2.0 * M_PI);

    // units reduce; radians on the encoder * this number = radians on the hood
    return in_travel_per_radian * radians_per_in_travel;
  }

  static constexpr double kHoodSingleTurnEncoderRatio() { return 8.0 / 72.0; }

  static constexpr double kMaxHoodEncoderPulsesPerSecond() {
    return control_loops::superstructure::hood::kFreeSpeed / (2.0 * M_PI) *
           control_loops::superstructure::hood::kOutputRatio /
           kHoodEncoderRatio() * kHoodEncoderCountsPerRevolution();
  }

  static constexpr ::frc971::constants::Range kHoodRange() {
    return ::frc971::constants::Range{
        -0.01,  // Back Hard
        0.65,   // Front Hard
        0.00,   // Back Soft
        0.63    // Front Soft
    };
  }

  ::frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemParams<
      ::frc971::zeroing::AbsoluteAndAbsoluteEncoderZeroingEstimator>
      hood;

  // Intake
  static constexpr double kIntakeEncoderCountsPerRevolution() { return 4096.0; }

  static constexpr double kIntakeEncoderRatio() { return (16.0 / 32.0); }

  static constexpr double kMaxIntakeEncoderPulsesPerSecond() {
    return control_loops::superstructure::intake::kFreeSpeed / (2.0 * M_PI) *
           control_loops::superstructure::intake::kOutputRatio /
           kIntakeEncoderRatio() * kIntakeEncoderCountsPerRevolution();
  }

  static constexpr ::frc971::constants::Range kIntakeRange() {
    return ::frc971::constants::Range{
        -1.05,  // Back Hard
        1.44,   // Front Hard
        -0.89,  // Back Soft
        1.26    // Front Soft
    };
  }

  static constexpr double kIntakeZero() { return -57 * M_PI / 180.0; }

  ::frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemParams<
      ::frc971::zeroing::AbsoluteEncoderZeroingEstimator>
      intake;

  static constexpr double kIntakeRollerSupplyCurrentLimit() { return 30.0; }
  static constexpr double kIntakeRollerStatorCurrentLimit() { return 40.0; }

  static constexpr double kFeederSupplyCurrentLimit() { return 30.0; }
  static constexpr double kFeederStatorCurrentLimit() { return 40.0; }

  // Turret
  static constexpr double kTurretEncoderCountsPerRevolution() { return 4096.0; }

  static constexpr double kTurretEncoderRatio() {
    return (26.0 / 150.0) * (130.0 / 26.0);
  }

  static constexpr double kMaxTurretEncoderPulsesPerSecond() {
    return control_loops::superstructure::turret::kFreeSpeed / (2.0 * M_PI) *
           control_loops::superstructure::turret::kOutputRatio /
           kTurretEncoderRatio() * kTurretEncoderCountsPerRevolution();
  }

  static constexpr double kTurretPotRatio() { return (26.0 / 150.0); }

  static constexpr ::frc971::constants::Range kTurretRange() {
    return ::frc971::constants::Range{
        -4.6,   // Back Hard
        4.85,   // Front Hard
        -4.3,   // Back Soft
        4.7123  // Front Soft
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
    return 2048.0;
  }
  static constexpr double kFinisherEncoderRatio() { return 36.0 / 40.0; }

  static constexpr double kMaxFinisherEncoderPulsesPerSecond() {
    return control_loops::superstructure::finisher::kFreeSpeed / (2.0 * M_PI) *
           control_loops::superstructure::finisher::kOutputRatio /
           kFinisherEncoderRatio() * kFinisherEncoderCountsPerRevolution();
  }

  static constexpr double kAcceleratorEncoderCountsPerRevolution() {
    return 2048.0;
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

  struct ShotParams {
    // Measured in radians
    double hood_angle;
    // Muzzle velocity (m/s) of the ball as it is shot out of the shooter.
    double velocity_ball;

    static ShotParams BlendY(double coefficient, ShotParams a1, ShotParams a2) {
      using ::frc971::shooter_interpolation::Blend;
      return ShotParams{Blend(coefficient, a1.hood_angle, a2.hood_angle),
                        Blend(coefficient, a1.velocity_ball, a2.velocity_ball)};
    }
  };

  struct FlywheelShotParams {
    // Angular velocity in radians per second of the slowest (lowest) wheel in
    // the kicker. Positive is shooting the ball.
    double velocity_accelerator;
    // Angular velocity in radians per seconds of the flywheel. Positive is
    // shooting.
    double velocity_finisher;

    static FlywheelShotParams BlendY(double coefficient, FlywheelShotParams a1,
                                     FlywheelShotParams a2) {
      using ::frc971::shooter_interpolation::Blend;
      return FlywheelShotParams{
          Blend(coefficient, a1.velocity_accelerator, a2.velocity_accelerator),
          Blend(coefficient, a1.velocity_finisher, a2.velocity_finisher)};
    }
  };

  // { distance_to_target, { hood_angle, velocity_ball }}
  InterpolationTable<ShotParams> shot_interpolation_table;
  // { velocity_ball, { velocity_accelerator, velocity_finisher }}
  InterpolationTable<FlywheelShotParams> flywheel_shot_interpolation_table;
};

// Creates (once) a Values instance for ::aos::network::GetTeamNumber(). Should
// be called before realtime because this allocates memory.
void InitValues();

// Returns a reference to the Values instance for
// ::aos::network::GetTeamNumber(). Values must be initialized through
// InitValues() before calling this.
const Values &GetValues();

}  // namespace constants
}  // namespace y2020

#endif  // y2020_CONSTANTS_H_

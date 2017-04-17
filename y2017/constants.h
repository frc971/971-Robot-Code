#ifndef Y2017_CONSTANTS_H_
#define Y2017_CONSTANTS_H_

#include <stdint.h>
#include <math.h>

#include "frc971/constants.h"
#include "frc971/shooter_interpolation/interpolation.h"

#include "y2017/control_loops/drivetrain/drivetrain_dog_motor_plant.h"
#include "y2017/control_loops/superstructure/column/column_plant.h"
#include "y2017/control_loops/superstructure/hood/hood_plant.h"
#include "y2017/control_loops/superstructure/intake/intake_plant.h"
#include "y2017/control_loops/superstructure/shooter/shooter_plant.h"

namespace y2017 {
namespace constants {

// Has all of our "constants", except the ones that come from other places. The
// ones which change between robots are put together with a workable way to
// retrieve the values for the current robot.

// Everything is in SI units (volts, radians, meters, seconds, etc).
// Some of these values are related to the conversion between raw values
// (encoder counts, voltage, etc) to scaled units (radians, meters, etc).
//
// All ratios are from the encoder shaft to the output units.

struct Values {
  struct Intake {
    double pot_offset;
    ::frc971::constants::PotAndAbsoluteEncoderZeroingConstants zeroing;
  };

  struct Hood {
    double pot_offset;
    ::frc971::constants::EncoderPlusIndexZeroingConstants zeroing;
  };

  struct Column {
    ::frc971::constants::HallEffectZeroingConstants indexer_zeroing;
    ::frc971::constants::HallEffectZeroingConstants turret_zeroing;
    // The max absolute value of the turret angle that we need to get to to be
    // classified as zeroed.  Otherwise, we may be ambiguous on which wrap we
    // are on.
    double turret_zeroed_distance;
  };

  static const int kZeroingSampleSize = 200;

  static constexpr double kDrivetrainCyclesPerRevolution = 256;
  static constexpr double kDrivetrainEncoderCountsPerRevolution =
      kDrivetrainCyclesPerRevolution * 4;
  static constexpr double kDrivetrainEncoderRatio =
      1.0 * control_loops::drivetrain::kWheelRadius;
  static constexpr double kMaxDrivetrainEncoderPulsesPerSecond =
      control_loops::drivetrain::kFreeSpeed *
      control_loops::drivetrain::kHighOutputRatio /
      constants::Values::kDrivetrainEncoderRatio *
      kDrivetrainEncoderCountsPerRevolution;

  static constexpr double kShooterEncoderCountsPerRevolution = 2048 * 4;
  static constexpr double kShooterEncoderRatio = 32.0 / 48.0;
  static constexpr double kMaxShooterEncoderPulsesPerSecond =
      control_loops::superstructure::shooter::kFreeSpeed *
      control_loops::superstructure::shooter::kOutputRatio /
      constants::Values::kShooterEncoderRatio *
      kShooterEncoderCountsPerRevolution;

  static constexpr double kIntakeEncoderCountsPerRevolution = 1024 * 4;
  static constexpr double kIntakeEncoderRatio =
      (16.0 * 0.25) * (20.0 / 40.0) / (2.0 * M_PI) * 0.0254;
  static constexpr double kIntakePotRatio = (16 * 0.25) / (2.0 * M_PI) * 0.0254;
  static constexpr double kIntakeEncoderIndexDifference =
      2.0 * M_PI * kIntakeEncoderRatio;
  static constexpr double kMaxIntakeEncoderPulsesPerSecond =
      control_loops::superstructure::intake::kFreeSpeed *
      control_loops::superstructure::intake::kOutputRatio /
      constants::Values::kIntakeEncoderRatio *
      kIntakeEncoderCountsPerRevolution;
  static constexpr ::frc971::constants::Range kIntakeRange{-0.01, 0.250, 0.01,
                                                           0.235};

  static constexpr double kHoodEncoderCountsPerRevolution = 2048 * 4;
  static constexpr double kHoodEncoderRatio = 20.0 / 345.0;
  static constexpr double kHoodEncoderIndexDifference =
      2.0 * M_PI * kHoodEncoderRatio;
  static constexpr double kMaxHoodEncoderPulsesPerSecond =
      control_loops::superstructure::hood::kFreeSpeed *
      control_loops::superstructure::hood::kOutputRatio /
      constants::Values::kHoodEncoderRatio * kHoodEncoderCountsPerRevolution;
  static constexpr ::frc971::constants::Range kHoodRange{
      -0.39 * M_PI / 180.0 - 0.01, 37.11 * M_PI / 180.0, (-0.39 + 1.0) * M_PI / 180.0,
      (37.11 - 1.0) * M_PI / 180.0};

  static constexpr double kTurretEncoderCountsPerRevolution = 256 * 4;
  static constexpr double kTurretEncoderRatio = 11.0 / 94.0;
  static constexpr double kMaxTurretEncoderPulsesPerSecond =
      control_loops::superstructure::column::kTurretFreeSpeed *
      control_loops::superstructure::column::kTurretOutputRatio /
      constants::Values::kTurretEncoderRatio *
      kTurretEncoderCountsPerRevolution;

  static constexpr double kIndexerEncoderCountsPerRevolution = 256 * 4;
  static constexpr double kIndexerEncoderRatio = (18.0 / 36.0) * (12.0 / 84.0);
  static constexpr double kIndexerEncoderIndexDifference =
      2.0 * M_PI * kIndexerEncoderRatio;
  static constexpr double kMaxIndexerEncoderPulsesPerSecond =
      control_loops::superstructure::column::kIndexerFreeSpeed *
      control_loops::superstructure::column::kIndexerOutputRatio /
      constants::Values::kIndexerEncoderRatio *
      kIndexerEncoderCountsPerRevolution;
  static constexpr ::frc971::constants::Range kTurretRange{
      -460.0 / 2.0 * M_PI / 180.0, 460.0 / 2.0 * M_PI / 180.0,
      -450.0 / 2.0 * M_PI / 180.0, 450.0 / 2.0 * M_PI / 180.0};

  double drivetrain_max_speed;

  Intake intake;

  Column column;

  Hood hood;

  double down_error;
  const char *vision_name;

  double vision_error;

  struct ShotParams {
    double angle;
    double power;
    double indexer_velocity;

    static ShotParams BlendY(double coefficient, ShotParams a1, ShotParams a2);
  };
};

// Creates (once) a Values instance for ::aos::network::GetTeamNumber() and
// returns a reference to it.
const Values &GetValues();

// Creates Values instances for each team number it is called with and returns
// them.
const Values &GetValuesForTeam(uint16_t team_number);

}  // namespace constants
}  // namespace y2017

#endif  // Y2017_CONSTANTS_H_

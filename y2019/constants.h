#ifndef Y2019_CONSTANTS_H_
#define Y2019_CONSTANTS_H_

#include <math.h>
#include <stdint.h>

#include "frc971/constants.h"
#include "y2019/control_loops/drivetrain/drivetrain_dog_motor_plant.h"
#include "y2019/control_loops/superstructure/intake/intake_plant.h"
#include "y2019/control_loops/superstructure/elevator/elevator_plant.h"
#include "y2019/control_loops/superstructure/stilts/stilts_plant.h"
#include "y2019/control_loops/superstructure/wrist/wrist_plant.h"

namespace y2019 {
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
  static const int kZeroingSampleSize = 200;

  static constexpr double kDrivetrainCyclesPerRevolution() { return 512.0; }
  static constexpr double kDrivetrainEncoderCountsPerRevolution() {
    return kDrivetrainCyclesPerRevolution() * 4;
  }
  static constexpr double kDrivetrainEncoderRatio() { return (24.0 / 52.0); }
  static constexpr double kMaxDrivetrainEncoderPulsesPerSecond() {
    return control_loops::drivetrain::kFreeSpeed / (2.0 * M_PI) *
           control_loops::drivetrain::kHighOutputRatio /
           constants::Values::kDrivetrainEncoderRatio() *
           kDrivetrainEncoderCountsPerRevolution();
  }

  // Elevator
  static constexpr double kElevatorEncoderCountsPerRevolution() {
    return 4096.0;
  }

  static constexpr double kElevatorEncoderRatio() {
    return (1.0) * control_loops::superstructure::elevator::kRadius;
  }

  static constexpr double kMaxElevatorEncoderPulsesPerSecond() {
    return control_loops::superstructure::elevator::kFreeSpeed *
           control_loops::superstructure::elevator::kOutputRatio /
           kElevatorEncoderRatio() / (2.0 * M_PI) *
           kElevatorEncoderCountsPerRevolution();
  }

  static constexpr double kElevatorPotRatio() {
    return (1.0) * control_loops::superstructure::elevator::kRadius;
  }

  // Intake
  static constexpr double kIntakeEncoderCountsPerRevolution() { return 4096.0; }

  static constexpr double kIntakeEncoderRatio() { return (18.0 / 38.0); }

  static constexpr double kMaxIntakeEncoderPulsesPerSecond() {
    return control_loops::superstructure::intake::kFreeSpeed *
           control_loops::superstructure::intake::kOutputRatio /
           kIntakeEncoderRatio() / (2.0 * M_PI) *
           kIntakeEncoderCountsPerRevolution();
  }

  // Wrist
  static constexpr double kWristEncoderCountsPerRevolution() { return 4096.0; }

  static constexpr double kWristEncoderRatio() {
    return (20.0 / 100.0) * (24.0 / 84.0);
  }

  static constexpr double kMaxWristEncoderPulsesPerSecond() {
    return control_loops::superstructure::wrist::kFreeSpeed *
           control_loops::superstructure::wrist::kOutputRatio /
           kWristEncoderRatio() / (2.0 * M_PI) *
           kWristEncoderCountsPerRevolution();
  }

  static constexpr double kWristPotRatio() { return (24.0) / (84.0); }

  // Stilts
  static constexpr double kStiltsEncoderCountsPerRevolution() { return 4096.0; }

  static constexpr double kStiltsEncoderRatio() {
    return (1.0 /* Gear ratio */) *
           control_loops::superstructure::stilts::kRadius;
  }

  static constexpr double kMaxStiltsEncoderPulsesPerSecond() {
    return control_loops::superstructure::stilts::kFreeSpeed *
           control_loops::superstructure::stilts::kOutputRatio /
           kStiltsEncoderRatio() / (2.0 * M_PI) *
           kStiltsEncoderCountsPerRevolution();
  }

  static constexpr double kStiltsPotRatio() {
    return (1.0 /* Gear ratio */) *
           control_loops::superstructure::stilts::kRadius;
  }

  struct PotAndAbsConstants {
    ::frc971::constants::PotAndAbsoluteEncoderZeroingConstants zeroing;
    double potentiometer_offset;
  };
  PotAndAbsConstants elevator;
  PotAndAbsConstants wrist;
  PotAndAbsConstants stilts;

  struct Intake {
    ::frc971::constants::AbsoluteEncoderZeroingConstants zeroing;
  };
  Intake intake;
};

// Creates (once) a Values instance for ::aos::network::GetTeamNumber() and
// returns a reference to it.
const Values &GetValues();

// Creates Values instances for each team number it is called with and returns
// them.
const Values &GetValuesForTeam(uint16_t team_number);

}  // namespace constants
}  // namespace y2019

#endif  // Y2019_CONSTANTS_H_

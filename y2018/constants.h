#ifndef Y2018_CONSTANTS_H_
#define Y2018_CONSTANTS_H_

#include <stdint.h>
#include <math.h>

#include "frc971/constants.h"

#include "y2018/control_loops/drivetrain/drivetrain_dog_motor_plant.h"
#include "y2018/control_loops/superstructure/arm/dynamics.h"
#include "y2018/control_loops/superstructure/intake/intake_plant.h"

namespace y2018 {
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
  static constexpr int kZeroingSampleSize() { return 200; }

  static constexpr double kDrivetrainCyclesPerRevolution() { return 512.0; }
  static constexpr double kDrivetrainEncoderCountsPerRevolution() {
    return kDrivetrainCyclesPerRevolution() * 4;
  }
  static constexpr double kDrivetrainEncoderRatio() {
    return (20.0 / 48.0) * (30.0 / 36.0);
  }
  static constexpr double kMaxDrivetrainEncoderPulsesPerSecond() {
    return control_loops::drivetrain::kFreeSpeed / (2.0 * M_PI) *
           control_loops::drivetrain::kHighOutputRatio /
           constants::Values::kDrivetrainEncoderRatio() *
           kDrivetrainEncoderCountsPerRevolution();
  }

  static constexpr double kDrivetrainShifterPotMaxVoltage() { return 2.0; }
  static constexpr double kDrivetrainShifterPotMinVoltage() { return 1.0; }

  static constexpr double kProximalEncoderCountsPerRevolution() { return 4096.0; }
  static constexpr double kProximalEncoderRatio() {
    return (12.0 / 60.0) * (18.0 / 84.0);
  }
  static constexpr double kMaxProximalEncoderPulsesPerSecond() {
    return control_loops::superstructure::arm::Dynamics::kFreeSpeed /
           (2.0 * M_PI) / control_loops::superstructure::arm::Dynamics::kG1 /
           kProximalEncoderRatio() * kProximalEncoderCountsPerRevolution();
  }
  static constexpr double kProximalPotRatio() { return (12.0 / 60.0); }

  static constexpr double kDistalEncoderCountsPerRevolution() { return 4096.0; }
  static constexpr double kDistalEncoderRatio() { return (12.0 / 60.0); }
  static constexpr double kMaxDistalEncoderPulsesPerSecond() {
    return control_loops::superstructure::arm::Dynamics::kFreeSpeed /
           (2.0 * M_PI) / control_loops::superstructure::arm::Dynamics::kG2 /
           kDistalEncoderRatio() * kProximalEncoderCountsPerRevolution();
  }
  static constexpr double kDistalPotRatio() {
    return (12.0 / 60.0) * (36.0 / 40.0);
  }

  static constexpr double kIntakeSpringRatio() {
    return (10.0 * 0.080) / (2.0 * 1.5 * M_PI);
  }
  static constexpr double kIntakeMotorEncoderCountsPerRevolution() { return 4096.0; }
  static constexpr double kIntakeMotorEncoderRatio() {
    return (18.0 / 68.0) * (18.0 / 50.0);
  }
  static constexpr double kIntakeMotorPotRatio() { return (14.0 / 68.0); }
  static constexpr double kMaxIntakeMotorEncoderPulsesPerSecond() {
    return control_loops::superstructure::intake::kFreeSpeed / (2.0 * M_PI) *
           control_loops::superstructure::intake::kGearRatio /
           kIntakeMotorEncoderRatio() *
           kIntakeMotorEncoderCountsPerRevolution();
  }

  struct Intake {
    double left_pot_offset;
    double right_pot_offset;
  };
  Intake intake;

  struct Proximal {
    double pot_offset;
  };
  Proximal proximal;

  struct Distal {
    double pot_offset;
  };
  Distal distal;

  double down_error;
  const char *vision_name;

  double vision_error;
};

// Creates (once) a Values instance for ::aos::network::GetTeamNumber() and
// returns a reference to it.
const Values &GetValues();

// Creates Values instances for each team number it is called with and returns
// them.
const Values &GetValuesForTeam(uint16_t team_number);

}  // namespace constants
}  // namespace y2018

#endif  // Y2018_CONSTANTS_H_

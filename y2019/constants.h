#ifndef Y2019_CONSTANTS_H_
#define Y2019_CONSTANTS_H_

#include <array>
#include <math.h>
#include <stdint.h>

#include "frc971/constants.h"
#include "frc971/control_loops/static_zeroing_single_dof_profiled_subsystem.h"
#include "y2019/control_loops/drivetrain/drivetrain_dog_motor_plant.h"
#include "y2019/control_loops/superstructure/elevator/elevator_plant.h"
#include "y2019/control_loops/superstructure/intake/intake_plant.h"
#include "y2019/control_loops/superstructure/stilts/stilts_plant.h"
#include "y2019/control_loops/superstructure/wrist/wrist_plant.h"
#include "y2019/control_loops/drivetrain/camera.h"
#include "frc971/control_loops/pose.h"

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


class Field {
 public:
  typedef ::frc971::control_loops::TypedPose<double> Pose;
  typedef ::y2019::control_loops::TypedTarget<double> Target;
  typedef ::frc971::control_loops::TypedLineSegment<double> Obstacle;

  static constexpr size_t kNumTargets = 32;
  static constexpr size_t kNumObstacles = 10;

  Field();

  ::std::array<Target, kNumTargets> targets() const { return targets_; }
  ::std::array<Obstacle, kNumObstacles> obstacles() const { return obstacles_; }

 private:
  // All target locations are defined as being at the center of the target,
  // except for the height, for which we use the top of the target.
  ::std::array<Target, kNumTargets> targets_;
  // Obstacle locations are approximate, as we are just trying to roughly
  // approximate what will block our view when on the field.
  // If anything, we should err on the side of making obstacles too small so
  // that if there is any error in our position, we don't assume that it must
  // be hidden behind a target when it really is not.
  ::std::array<Obstacle, kNumObstacles> obstacles_;
};

struct Values {
  static const int kZeroingSampleSize = 200;

  // Drivetrain Constants
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

  static constexpr ::frc971::constants::Range kElevatorRange() {
    return ::frc971::constants::Range{
        -0.02,  // Bottom Hard
        1.62,   // Top Hard
        0.01,   // Bottom Soft
        1.59    // Top Soft
    };
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

  static constexpr ::frc971::constants::Range kIntakeRange() {
    return ::frc971::constants::Range{
        -1.30,  // Back Hard
        1.35,   // Front Hard
        -1.25,  // Back Soft
        1.30    // Front Soft
    };
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

  static constexpr ::frc971::constants::Range kWristRange() {
    return ::frc971::constants::Range{
        -3.14,  // Back Hard
        3.14,   // Front Hard
        -2.97,  // Back Soft
        2.41    // Front Soft
    };
  }

  // Stilts
  static constexpr double kStiltsEncoderCountsPerRevolution() { return 4096.0; }

  // Stilts Constants
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

  static constexpr ::frc971::constants::Range kStiltsRange() {
    return ::frc971::constants::Range{
        -0.01,  // Top Hard
        0.72,   // Bottom Hard
        0.00,   // Top Soft
        0.71    // Bottom Soft
    };
  }

  struct PotAndAbsConstants {
    ::frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemParams<
        ::frc971::zeroing::PotAndAbsoluteEncoderZeroingEstimator>
        subsystem_params;
    double potentiometer_offset;
  };

  PotAndAbsConstants elevator;
  PotAndAbsConstants wrist;
  ::frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemParams<
      ::frc971::zeroing::AbsoluteEncoderZeroingEstimator>
      intake;

  PotAndAbsConstants stilts;

  struct CameraCalibration {
    // Pose of the camera relative to the robot.
    ::frc971::control_loops::TypedPose<double> pose;
    // Field of view, in radians. This is total horizontal FOV, from left
    // edge to right edge of the camera image.
    double fov;
  };

  static constexpr size_t kNumCameras = 5;
  ::std::array<CameraCalibration, kNumCameras> cameras;
  control_loops::TypedCamera<Field::kNumTargets, Field::kNumObstacles,
                             double>::NoiseParameters camera_noise_parameters;
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

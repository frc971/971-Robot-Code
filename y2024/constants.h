#ifndef Y2024_CONSTANTS_H_
#define Y2024_CONSTANTS_H_

#include <array>
#include <cmath>
#include <cstdint>

#include "frc971/constants.h"
#include "frc971/control_loops/pose.h"
#include "frc971/control_loops/static_zeroing_single_dof_profiled_subsystem.h"
#include "frc971/shooter_interpolation/interpolation.h"
#include "frc971/zeroing/absolute_encoder.h"
#include "frc971/zeroing/pot_and_absolute_encoder.h"
#include "y2024/constants/constants_generated.h"
#include "y2024/control_loops/drivetrain/drivetrain_dog_motor_plant.h"
#include "y2024/control_loops/superstructure/altitude/altitude_plant.h"
#include "y2024/control_loops/superstructure/catapult/catapult_plant.h"
#include "y2024/control_loops/superstructure/climber/climber_plant.h"
#include "y2024/control_loops/superstructure/extend/extend_plant.h"
#include "y2024/control_loops/superstructure/intake_pivot/intake_pivot_plant.h"
#include "y2024/control_loops/superstructure/turret/turret_plant.h"

namespace y2024::constants {

constexpr uint16_t kCompTeamNumber = 971;
constexpr uint16_t kPracticeTeamNumber = 9971;
constexpr uint16_t kCodingRobotTeamNumber = 7971;

struct Values {
  static const int kSuperstructureCANWriterPriority = 35;
  static const int kDrivetrainWriterPriority = 35;
  static const int kDrivetrainTxPriority = 36;
  static const int kDrivetrainRxPriority = 36;

  // TODO: These values will need to be changed for the 2024 robot.
  static constexpr double kDrivetrainCyclesPerRevolution() { return 512.0; }
  static constexpr double kDrivetrainEncoderCountsPerRevolution() {
    return kDrivetrainCyclesPerRevolution() * 4;
  }
  static constexpr double kDrivetrainEncoderRatio() { return 1.0; }
  static constexpr double kMaxDrivetrainEncoderPulsesPerSecond() {
    return control_loops::drivetrain::kFreeSpeed / (2.0 * M_PI) *
           control_loops::drivetrain::kHighOutputRatio /
           constants::Values::kDrivetrainEncoderRatio() *
           kDrivetrainEncoderCountsPerRevolution();
  }

  static double DrivetrainEncoderToMeters(int32_t in) {
    return ((static_cast<double>(in) /
             kDrivetrainEncoderCountsPerRevolution()) *
            (2.0 * M_PI)) *
           kDrivetrainEncoderRatio() * control_loops::drivetrain::kWheelRadius;
  }

  static double DrivetrainCANEncoderToMeters(double rotations) {
    return (rotations * (2.0 * M_PI)) *
           control_loops::drivetrain::kHighOutputRatio;
  }
  // TODO: (niko) add the gear ratios for the intake once we have them
  static constexpr double kIntakePivotEncoderCountsPerRevolution() {
    return 4096.0;
  }

  static constexpr double kIntakePivotEncoderRatio() { return (15.0 / 24.0); }

  static constexpr double kMaxIntakePivotEncoderPulsesPerSecond() {
    return control_loops::superstructure::intake_pivot::kFreeSpeed /
           (2.0 * M_PI) *
           control_loops::superstructure::intake_pivot::kOutputRatio /
           kIntakePivotEncoderRatio() *
           kIntakePivotEncoderCountsPerRevolution();
  }

  static constexpr double kClimberPotMetersPerRevolution() {
    return 16 * 0.25 * 0.0254;
  }

  static constexpr double kClimberPotMetersPerVolt() {
    return kClimberPotMetersPerRevolution() * (10.0 /*turns*/ / 5.0 /*volts*/);
  }

  static constexpr double kExtendEncoderCountsPerRevolution() { return 4096.0; }

  // TODO: (niko) add the gear ratios for the intake once we have them
  static constexpr double kCatapultEncoderCountsPerRevolution() {
    return 4096.0;
  }

  static constexpr double kCatapultEncoderRatio() { return 12.0 / 24.0; }

  static constexpr double kCatapultPotRatio() { return 12.0 / 24.0; }

  static constexpr double kCatapultPotRadiansPerVolt() {
    return kCatapultPotRatio() * (3.0 /*turns*/ / 5.0 /*volts*/) *
           (2 * M_PI /*radians*/);
  }

  static constexpr double kMaxCatapultEncoderPulsesPerSecond() {
    return control_loops::superstructure::catapult::kFreeSpeed / (2.0 * M_PI) *
           control_loops::superstructure::catapult::kOutputRatio /
           kCatapultEncoderRatio() * kCatapultEncoderCountsPerRevolution();
  }

  static constexpr double kExtendEncoderRatio() { return 1.0; }

  static constexpr double kExtendPotMetersPerRevolution() {
    return 36 * 0.005 * kExtendEncoderRatio();
  }
  static constexpr double kExtendEncoderMetersPerRadian() {
    return kExtendPotMetersPerRevolution() / 2.0 / M_PI;
  }
  static constexpr double kExtendPotMetersPerVolt() {
    return kExtendPotMetersPerRevolution() * (5.0 /*turns*/ / 5.0 /*volts*/);
  }
  static constexpr double kMaxExtendEncoderPulsesPerSecond() {
    return control_loops::superstructure::extend::kFreeSpeed / (2.0 * M_PI) *
           control_loops::superstructure::extend::kOutputRatio /
           kExtendEncoderRatio() * kExtendEncoderCountsPerRevolution();
  }

  static constexpr double kTurretEncoderCountsPerRevolution() { return 4096.0; }

  static constexpr double kTurretPotRatio() {
    return (22.0 / 100.0) * (28.0 / 48.0) * (36.0 / 24.0);
  }

  static constexpr double kTurretEncoderRatio() { return 22.0 / 100.0; }

  static constexpr double kTurretPotRadiansPerVolt() {
    return kTurretPotRatio() * (10.0 /*turns*/ / 5.0 /*volts*/) *
           (2 * M_PI /*radians*/);
  }
  static constexpr double kMaxTurretEncoderPulsesPerSecond() {
    return control_loops::superstructure::turret::kFreeSpeed / (2.0 * M_PI) *
           control_loops::superstructure::turret::kOutputRatio /
           kTurretEncoderRatio() * kTurretEncoderCountsPerRevolution();
  }

  static constexpr double kAltitudeEncoderCountsPerRevolution() {
    return 4096.0;
  }

  static constexpr double kAltitudeEncoderRatio() { return 16.0 / 162.0; }

  static constexpr double kAltitudePotRatio() { return 16.0 / 162.0; }

  static constexpr double kAltitudePotRadiansPerVolt() {
    return kAltitudePotRatio() * (10.0 /*turns*/ / 5.0 /*volts*/) *
           (2 * M_PI /*radians*/);
  }
  static constexpr double kMaxAltitudeEncoderPulsesPerSecond() {
    return control_loops::superstructure::altitude::kFreeSpeed / (2.0 * M_PI) *
           control_loops::superstructure::altitude::kOutputRatio /
           kAltitudeEncoderRatio() * kAltitudeEncoderCountsPerRevolution();
  }

  // 20 -> 28 reduction to a 0.5" radius roller
  static constexpr double kTransferRollerOutputRatio = (20.0 / 28.0) * 0.0127;
  // 20 -> 34 reduction, and the 34 is on a 0.625" radius roller
  static constexpr double kIntakeRollerOutputRatio = (20.0 / 34.0) * 0.015875;
  // 20 -> 28 reduction to a 0.5" radius roller
  static constexpr double kExtendRollerOutputRatio = (20.0 / 28.0) * 0.0127;

  struct ShotParams {
    // Measured in radians
    double shot_altitude_angle = 0.0;
    double shot_catapult_angle = 0.0;

    // Muzzle velocity (m/s) of the game piece as it is released from the
    // catapult.
    double shot_velocity = 0.0;

    // Speed over ground to use for shooting on the fly
    double shot_speed_over_ground = 0.0;

    static ShotParams BlendY(double coefficient, ShotParams a1, ShotParams a2) {
      using ::frc971::shooter_interpolation::Blend;
      return ShotParams{
          .shot_altitude_angle = Blend(coefficient, a1.shot_altitude_angle,
                                       a2.shot_altitude_angle),
          .shot_catapult_angle = Blend(coefficient, a1.shot_catapult_angle,
                                       a2.shot_catapult_angle),
          .shot_velocity =
              Blend(coefficient, a1.shot_velocity, a2.shot_velocity),
          .shot_speed_over_ground =
              Blend(coefficient, a1.shot_speed_over_ground,
                    a2.shot_speed_over_ground),
      };
    }

    static ShotParams FromFlatbuffer(const y2024::ShotParams *shot_params) {
      return ShotParams{
          .shot_altitude_angle = shot_params->shot_altitude_angle(),
          .shot_catapult_angle = shot_params->shot_catapult_angle(),
          .shot_velocity = shot_params->shot_velocity(),
          .shot_speed_over_ground = shot_params->shot_speed_over_ground()};
    }
  };

  static frc971::shooter_interpolation::InterpolationTable<ShotParams>
  InterpolationTableFromFlatbuffer(
      const flatbuffers::Vector<
          flatbuffers::Offset<y2024::InterpolationTablePoint>> *table) {
    std::vector<std::pair<double, ShotParams>> interpolation_table;

    for (const InterpolationTablePoint *point : *table) {
      interpolation_table.emplace_back(
          point->distance_from_goal(),
          ShotParams::FromFlatbuffer(point->shot_params()));
    }

    return frc971::shooter_interpolation::InterpolationTable<ShotParams>(
        interpolation_table);
  }

  struct PotAndAbsEncoderConstants {
    ::frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemParams<
        ::frc971::zeroing::PotAndAbsoluteEncoderZeroingEstimator>
        subsystem_params;
    double potentiometer_offset;
  };

  struct AbsoluteEncoderConstants {
    ::frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemParams<
        ::frc971::zeroing::AbsoluteEncoderZeroingEstimator>
        subsystem_params;
  };

  struct PotConstants {
    ::frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemParams<
        ::frc971::zeroing::RelativeEncoderZeroingEstimator>
        subsystem_params;
    double potentiometer_offset;
  };
};

// Creates and returns a Values instance for the constants.
// Should be called before realtime because this allocates memory.
// Only the first call to either of these will be used.
constants::Values MakeValues(uint16_t team);

// Calls MakeValues with aos::network::GetTeamNumber()
constants::Values MakeValues();

}  // namespace y2024::constants

#endif  // Y2024_CONSTANTS_H_

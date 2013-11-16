#include "frc971/constants.h"

#include <math.h>
#include <stdint.h>
#include <inttypes.h>

#include "aos/common/logging/logging.h"
#include "aos/common/once.h"
#include "aos/common/network/team_number.h"

#include "frc971/control_loops/drivetrain/polydrivetrain_dog_motor_plant.h"
#include "frc971/control_loops/drivetrain/polydrivetrain_clutch_motor_plant.h"
#include "frc971/control_loops/drivetrain/drivetrain_dog_motor_plant.h"
#include "frc971/control_loops/drivetrain/drivetrain_clutch_motor_plant.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace frc971 {
namespace constants {
namespace {

// It has about 0.029043 of gearbox slop.
// For purposes of moving the end up/down by a certain amount, the wrist is 18
// inches long.
const double kCompWristHallEffectStartAngle = 1.277;
const double kPracticeWristHallEffectStartAngle = 1.178;

const double kWristHallEffectStopAngle = 100 * M_PI / 180.0;

const double kPracticeWristUpperPhysicalLimit = 1.677562;
const double kCompWristUpperPhysicalLimit = 1.677562;

const double kPracticeWristLowerPhysicalLimit = -0.746128;
const double kCompWristLowerPhysicalLimit = -0.746128;

const double kPracticeWristUpperLimit = 1.615385;
const double kCompWristUpperLimit = 1.615385;

const double kPracticeWristLowerLimit = -0.746128;
const double kCompWristLowerLimit = -0.746128;

const double kWristZeroingSpeed = 0.125;
const double kWristZeroingOffSpeed = 0.35;

const int kAngleAdjustHallEffect = 2;

// Angle measured from CAD with the top of the angle adjust at the top of the
// wire guide is 0.773652098 radians.

const double kCompAngleAdjustHallEffectStartAngle[2] = {0.301170496, 1.5};
const double kPracticeAngleAdjustHallEffectStartAngle[2] = {0.297, 1.5};

const double kAngleAdjustHallEffectStopAngle[2] = {0.1, 1.0};

const double kPracticeAngleAdjustUpperPhysicalLimit = 0.904737;
const double kCompAngleAdjustUpperPhysicalLimit = 0.904737;

const double kPracticeAngleAdjustLowerPhysicalLimit = 0.270;
const double kCompAngleAdjustLowerPhysicalLimit = 0.302;

const double kPracticeAngleAdjustUpperLimit = 0.87;
const double kCompAngleAdjustUpperLimit = 0.87;

const double kPracticeAngleAdjustLowerLimit = 0.31;
const double kCompAngleAdjustLowerLimit = 0.28;

const double kAngleAdjustZeroingSpeed = -0.2;
const double kAngleAdjustZeroingOffSpeed = -0.5;

const double kPracticeAngleAdjustDeadband = 0.4;
const double kCompAngleAdjustDeadband = 0.0;

const int kCompCameraCenter = -2;
const int kPracticeCameraCenter = -5;

const double kCompDrivetrainEncoderRatio =
    (15.0 / 50.0) /*output reduction*/ * (36.0 / 24.0) /*encoder gears*/;
const double kCompLowGearRatio = 14.0 / 60.0 * 15.0 / 50.0;
const double kCompHighGearRatio = 30.0 / 44.0 * 15.0 / 50.0;

const double kPracticeDrivetrainEncoderRatio =
    (17.0 / 50.0) /*output reduction*/ * (64.0 / 24.0) /*encoder gears*/;
const double kPracticeLowGearRatio = 16.0 / 60.0 * 19.0 / 50.0;
const double kPracticeHighGearRatio = 28.0 / 48.0 * 19.0 / 50.0;

const ShifterHallEffect kCompLeftDriveShifter{0.83, 2.32, 1.2, 1.0};
const ShifterHallEffect kCompRightDriveShifter{0.865, 2.375, 1.2, 1.0};

const ShifterHallEffect kPracticeLeftDriveShifter{2.082283, 0.834433, 0.60,
                                                  0.47};
const ShifterHallEffect kPracticeRightDriveShifter{2.070124, 0.838993, 0.62,
                                                   0.55};

const Values *DoGetValues() {
  uint16_t team = ::aos::network::GetTeamNumber();
  LOG(INFO, "creating a Constants for team %" PRIu16 "\n", team);
  switch (team) {
    case kCompTeamNumber:
      return new Values{kCompWristHallEffectStartAngle,
                        kWristHallEffectStopAngle,
                        kCompWristUpperLimit,
                        kCompWristLowerLimit,
                        kCompWristUpperPhysicalLimit,
                        kCompWristLowerPhysicalLimit,
                        kWristZeroingSpeed,
                        kWristZeroingOffSpeed,
                        kCompAngleAdjustHallEffectStartAngle,
                        kAngleAdjustHallEffectStopAngle,
                        kCompAngleAdjustUpperLimit,
                        kCompAngleAdjustLowerLimit,
                        kCompAngleAdjustUpperPhysicalLimit,
                        kCompAngleAdjustLowerPhysicalLimit,
                        kAngleAdjustZeroingSpeed,
                        kAngleAdjustZeroingOffSpeed,
                        kCompAngleAdjustDeadband,
                        kCompDrivetrainEncoderRatio,
                        kCompLowGearRatio,
                        kCompHighGearRatio,
                        kCompLeftDriveShifter,
                        kCompRightDriveShifter,
                        true,
                        control_loops::MakeVClutchDrivetrainLoop,
                        control_loops::MakeClutchDrivetrainLoop,
                        kCompCameraCenter};
      break;
    case kPracticeTeamNumber:
      return new Values{kPracticeWristHallEffectStartAngle,
                        kWristHallEffectStopAngle,
                        kPracticeWristUpperLimit,
                        kPracticeWristLowerLimit,
                        kPracticeWristUpperPhysicalLimit,
                        kPracticeWristLowerPhysicalLimit,
                        kWristZeroingSpeed,
                        kWristZeroingOffSpeed,
                        kPracticeAngleAdjustHallEffectStartAngle,
                        kAngleAdjustHallEffectStopAngle,
                        kPracticeAngleAdjustUpperLimit,
                        kPracticeAngleAdjustLowerLimit,
                        kPracticeAngleAdjustUpperPhysicalLimit,
                        kPracticeAngleAdjustLowerPhysicalLimit,
                        kAngleAdjustZeroingSpeed,
                        kAngleAdjustZeroingOffSpeed,
                        kPracticeAngleAdjustDeadband,
                        kPracticeDrivetrainEncoderRatio,
                        kPracticeLowGearRatio,
                        kPracticeHighGearRatio,
                        kPracticeLeftDriveShifter,
                        kPracticeRightDriveShifter,
                        false,
                        control_loops::MakeVDogDrivetrainLoop,
                        control_loops::MakeDogDrivetrainLoop,
                        kPracticeCameraCenter};
      break;
    default:
      LOG(FATAL, "unknown team #%" PRIu16 "\n", team);
  }
}

}  // namespace

const Values &GetValues() {
  static ::aos::Once<const Values> once(DoGetValues);
  return *once.Get();
}

}  // namespace constants
}  // namespace frc971

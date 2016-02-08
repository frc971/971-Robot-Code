#include "y2016/constants.h"

#include <math.h>
#include <stdint.h>
#include <inttypes.h>

#include <map>

#if __has_feature(address_sanitizer)
#include "sanitizer/lsan_interface.h"
#endif

#include "aos/common/logging/logging.h"
#include "aos/common/once.h"
#include "aos/common/network/team_number.h"
#include "aos/common/mutex.h"

#include "y2016/control_loops/drivetrain/polydrivetrain_dog_motor_plant.h"
#include "y2016/control_loops/drivetrain/drivetrain_dog_motor_plant.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace y2016 {
namespace constants {
namespace {

const uint16_t kCompTeamNumber = 971;
const uint16_t kPracticeTeamNumber = 9971;

// TODO(constants): Update these to what we're using this year.
const double kCompDrivetrainEncoderRatio =
    (18.0 / 50.0) /*output reduction*/ * (56.0 / 30.0) /*encoder gears*/;
const double kCompLowGearRatio = 18.0 / 60.0 * 18.0 / 50.0;
const double kCompHighGearRatio = 28.0 / 50.0 * 18.0 / 50.0;

const double kPracticeDrivetrainEncoderRatio = kCompDrivetrainEncoderRatio;
const double kPracticeLowGearRatio = kCompLowGearRatio;
const double kPracticeHighGearRatio = kCompHighGearRatio;

const ShifterHallEffect kCompLeftDriveShifter{2.61, 2.33, 4.25, 3.28, 0.2, 0.7};
const ShifterHallEffect kCompRightDriveShifter{2.94, 4.31, 4.32,
                                               3.25, 0.2,  0.7};

const ShifterHallEffect kPracticeLeftDriveShifter{2.80, 3.05, 4.15,
                                                  3.2,  0.2,  0.7};
const ShifterHallEffect kPracticeRightDriveShifter{2.90, 3.75, 3.80,
                                                   2.98, 0.2,  0.7};

const double kRobotWidth = 25.0 / 100.0 * 2.54;

// TODO(comran): Remove comments from ratios for pots when they are actually
// used.
const double kIntakeEncoderRatio = 18.0 / 48.0 * 16.0 / 72.0;
//const double kIntakePotRatio = 48.0 / 48.0 * 16.0 / 72.0;
const double kShoulderEncoderRatio = 18.0 / 48.0 * 16.0 / 72.0;
//const double kShoulderPotRatio = 48.0 / 48.0 * 16.0 / 72.0;
const double kWristEncoderRatio = 18.0 / 48.0 * 16.0 / 72.0;
//const double kWristPotRatio = 48.0 / 48.0 * 16.0 / 72.0;

const double kIntakeEncoderIndexDifference = 2.0 * M_PI * kIntakeEncoderRatio;
const double kShoulderEncoderIndexDifference = 2.0 * M_PI * kShoulderEncoderRatio;
const double kWristEncoderIndexDifference = 2.0 * M_PI * kWristEncoderRatio;

const int kZeroingSampleSize = 200;

const Values *DoGetValuesForTeam(uint16_t team) {
  switch (team) {
    case 1:  // for tests
      return new Values{
          kCompDrivetrainEncoderRatio,
          kCompLowGearRatio,
          kCompHighGearRatio,
          kCompLeftDriveShifter,
          kCompRightDriveShifter,
          0.5,
          ::y2016::control_loops::drivetrain::MakeVelocityDrivetrainLoop,
          ::y2016::control_loops::drivetrain::MakeDrivetrainLoop,
          5.0,  // drivetrain max speed

          // Intake
          {
            {-M_PI - 0.05, M_PI + 0.05, -M_PI, M_PI},
            {kZeroingSampleSize, kIntakeEncoderIndexDifference, 0.9, 0.3},
          },

          // Shoulder
          {
            {-M_PI - 0.05, M_PI + 0.05, -M_PI, M_PI},
            {kZeroingSampleSize, kShoulderEncoderIndexDifference, 0.9, 0.3},
          },

          // Wrist
          {
            {-M_PI - 0.05, M_PI + 0.05, -M_PI, M_PI},
            {kZeroingSampleSize, kWristEncoderIndexDifference, 0.9, 0.3},
          },
      };
      break;
    case kCompTeamNumber:
      return new Values{
          kCompDrivetrainEncoderRatio,
          kCompLowGearRatio,
          kCompHighGearRatio,
          kCompLeftDriveShifter,
          kCompRightDriveShifter,
          kRobotWidth,
          ::y2016::control_loops::drivetrain::MakeVelocityDrivetrainLoop,
          ::y2016::control_loops::drivetrain::MakeDrivetrainLoop,
          5.0,  // drivetrain max speed

          // Intake
          {
            {-M_PI - 0.05, M_PI + 0.05, -M_PI, M_PI},
            {kZeroingSampleSize, kIntakeEncoderIndexDifference, 0.9, 0.3},
          },

          // Shoulder
          {
            {-M_PI - 0.05, M_PI + 0.05, -M_PI, M_PI},
            {kZeroingSampleSize, kShoulderEncoderIndexDifference, 0.9, 0.3},
          },

          // Wrist
          {
            {-M_PI - 0.05, M_PI + 0.05, -M_PI, M_PI},
            {kZeroingSampleSize, kWristEncoderIndexDifference, 0.9, 0.3},
          },
      };
      break;
    case kPracticeTeamNumber:
      return new Values{
          kPracticeDrivetrainEncoderRatio,
          kPracticeLowGearRatio,
          kPracticeHighGearRatio,
          kPracticeLeftDriveShifter,
          kPracticeRightDriveShifter,
          kRobotWidth,
          ::y2016::control_loops::drivetrain::MakeVelocityDrivetrainLoop,
          ::y2016::control_loops::drivetrain::MakeDrivetrainLoop,
          5.0,  // drivetrain max speed

          // Intake
          {
            {-M_PI - 0.05, M_PI + 0.05, -M_PI, M_PI},
            {kZeroingSampleSize, kIntakeEncoderIndexDifference, 0.9, 0.3},
          },

          // Shoulder
          {
            {-M_PI - 0.05, M_PI + 0.05, -M_PI, M_PI},
            {kZeroingSampleSize, kShoulderEncoderIndexDifference, 0.9, 0.3},
          },

          // Wrist
          {
            {-M_PI - 0.05, M_PI + 0.05, -M_PI, M_PI},
            {kZeroingSampleSize, kWristEncoderIndexDifference, 0.9, 0.3},
          },
      };
      break;
    default:
      LOG(FATAL, "unknown team #%" PRIu16 "\n", team);
  }
}

const Values *DoGetValues() {
  uint16_t team = ::aos::network::GetTeamNumber();
  LOG(INFO, "creating a Constants for team %" PRIu16 "\n", team);
  return DoGetValuesForTeam(team);
}

}  // namespace

const Values &GetValues() {
  static ::aos::Once<const Values> once(DoGetValues);
  return *once.Get();
}

const Values &GetValuesForTeam(uint16_t team_number) {
  static ::aos::Mutex mutex;
  ::aos::MutexLocker locker(&mutex);

  // IMPORTANT: This declaration has to stay after the mutex is locked to avoid
  // race conditions.
  static ::std::map<uint16_t, const Values *> values;

  if (values.count(team_number) == 0) {
    values[team_number] = DoGetValuesForTeam(team_number);
#if __has_feature(address_sanitizer)
    __lsan_ignore_object(values[team_number]);
#endif
  }
  return *values[team_number];
}

}  // namespace constants
}  // namespace y2016

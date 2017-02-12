#include "y2017/constants.h"

#include <inttypes.h>
#include <math.h>
#include <stdint.h>

#include <map>

#if __has_feature(address_sanitizer)
#include "sanitizer/lsan_interface.h"
#endif

#include "aos/common/logging/logging.h"
#include "aos/common/mutex.h"
#include "aos/common/network/team_number.h"
#include "aos/common/once.h"

#include "y2017/control_loops/drivetrain/drivetrain_dog_motor_plant.h"
#include "y2017/control_loops/drivetrain/polydrivetrain_dog_motor_plant.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace y2017 {
namespace constants {

// ///// Mutual constants between robots. /////
const int Values::kZeroingSampleSize;

constexpr double Values::kDrivetrainEncoderRatio, Values::kShooterEncoderRatio,
    Values::kIntakeEncoderRatio, Values::kIntakePotRatio,
    Values::kHoodEncoderRatio, Values::kHoodPotRatio,
    Values::kTurretEncoderRatio, Values::kTurretPotRatio,
    Values::kIndexerEncoderRatio, Values::kIntakeEncoderIndexDifference,
    Values::kIndexerEncoderIndexDifference,
    Values::kTurretEncoderIndexDifference, Values::kHoodEncoderIndexDifference;
constexpr ::frc971::constants::Range Values::kIntakeRange, Values::kHoodRange,
    Values::kTurretRange;

namespace {
const uint16_t kCompTeamNumber = 971;
const uint16_t kPracticeTeamNumber = 9971;

// ///// Dynamic constants. /////

const Values *DoGetValuesForTeam(uint16_t team) {
  switch (team) {
    case 1:  // for tests
      return new Values{
          5.0,  // drivetrain max speed

          // Intake
          {
              // Pot Offset
              0.0,
              {Values::kZeroingSampleSize,
               Values::kIntakeEncoderIndexDifference, 0.0, 0.3},
          },

          // Turret
          {
              // Pot Offset
              0.0,
              {Values::kZeroingSampleSize,
               Values::kTurretEncoderIndexDifference, 0.0, 0.3},
          },

          // Hood
          {
              // Pot Offset
              0.1,
              {Values::kZeroingSampleSize, Values::kHoodEncoderIndexDifference,
               0.1, 0.3},
          },
          0.0,  // down error
          "practice",
      };
      break;

    case kCompTeamNumber:
      return new Values{
          5.0,  // drivetrain max speed

          // Intake
          {
              // Pot Offset
              0.0,
              {Values::kZeroingSampleSize,
               Values::kIntakeEncoderIndexDifference, 0.0, 0.3},
          },

          // Turret
          {
              // Pot Offset
              0.0,
              {Values::kZeroingSampleSize,
               Values::kTurretEncoderIndexDifference, 0.0, 0.3},
          },

          // Hood
          {
              // Pot Offset
              0.0,
              {Values::kZeroingSampleSize, Values::kHoodEncoderIndexDifference,
               0.0, 0.3},
          },
          0.0,  // down error
          "competition",
      };
      break;

    case kPracticeTeamNumber:
      return new Values{
          5.0,  // drivetrain max speed

          // Intake
          {
              // Pot Offset
              0.0,
              {Values::kZeroingSampleSize,
               Values::kIntakeEncoderIndexDifference, 0.0, 0.3},
          },

          // Turret
          {
              // Pot Offset
              0.0,
              {Values::kZeroingSampleSize,
               Values::kTurretEncoderIndexDifference, 0.0, 0.3},
          },

          // Hood
          {
              // Pot Offset
              0.0,
              {Values::kZeroingSampleSize, Values::kHoodEncoderIndexDifference,
               0.0, 0.3},
          },
          0.0,  // down error
          "practice",
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
}  // namespace y2017

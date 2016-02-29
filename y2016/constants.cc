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

// ///// Mutual constants between robots. /////
const int Values::kZeroingSampleSize;

constexpr double Values::kDrivetrainEncoderRatio, Values::kShooterEncoderRatio,
    Values::kIntakeEncoderRatio, Values::kShoulderEncoderRatio,
    Values::kWristEncoderRatio, Values::kIntakePotRatio,
    Values::kShoulderPotRatio, Values::kWristPotRatio,
    Values::kIntakeEncoderIndexDifference,
    Values::kShoulderEncoderIndexDifference,
    Values::kWristEncoderIndexDifference;
constexpr ::frc971::constants::Range Values::kIntakeRange,
    Values::kShoulderRange, Values::kWristRange;

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
           0.0,
           {Values::kZeroingSampleSize, Values::kIntakeEncoderIndexDifference,
            0.0, 0.3},
          },

          // Shoulder
          {
           0.0,
           {Values::kZeroingSampleSize, Values::kShoulderEncoderIndexDifference,
            0.0, 0.3},
          },

          // Wrist
          {
           0.0,
           {Values::kZeroingSampleSize, Values::kWristEncoderIndexDifference,
            0.0, 0.3},
          },
      };
      break;

    case kCompTeamNumber:
      return new Values{
          5.0,  // drivetrain max speed

          // Intake
          {
           // Value to add to the pot reading for the intake.
           -4.550531 + 150.40906362 * M_PI / 180.0,
           {Values::kZeroingSampleSize, Values::kIntakeEncoderIndexDifference,
            // Location of an index pulse.
            -0.087413 + 150.40906362 * M_PI / 180.0, 0.3},
          },

          // Shoulder
          {
           // Value to add to the pot reading for the shoulder.
           -1.0 - 0.0822 + 0.06138835 * M_PI / 180.0,
           {Values::kZeroingSampleSize, Values::kShoulderEncoderIndexDifference,
            0.06138835 * M_PI / 180.0 + 0.353794, 0.3},
          },

          // Wrist
          {
           // Value to add to the pot reading for the wrist.
           3.2390714288298668 + -0.06138835 * M_PI / 180.0,
           {Values::kZeroingSampleSize, Values::kWristEncoderIndexDifference,
            -0.06138835 * M_PI / 180.0 - 0.260542, 0.3},
          },
      };
      break;
    case kPracticeTeamNumber:
      return new Values{
          5.0,  // drivetrain max speed

          // Intake
          {
           -4.654 + 150.40906362 * M_PI / 180.0,
           {Values::kZeroingSampleSize, Values::kIntakeEncoderIndexDifference,
            -0.076004 + 150.40906362 * M_PI / 180.0, 0.3},
          },

          // Shoulder
          {
           -1.0016 + 0.06138835 * M_PI / 180.0,
           {Values::kZeroingSampleSize, Values::kShoulderEncoderIndexDifference,
            0.706940 + 0.06138835 * M_PI / 180.0, 0.3},
          },

          // Wrist
          {
           3.326328571170133 - 0.06138835 * M_PI / 180.0,
           {Values::kZeroingSampleSize, Values::kWristEncoderIndexDifference,
            -0.634131 - 0.06138835 * M_PI / 180.0, 0.3},
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

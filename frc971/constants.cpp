#include "frc971/constants.h"

#include <stddef.h>
#include <inttypes.h>

#include "aos/common/messages/RobotState.q.h"
#include "aos/atom_code/output/MotorOutput.h"

namespace frc971 {
namespace constants {

namespace {

constexpr double kCompHorizontalHallEffectStartAngle = 72 * M_PI / 180.0;
constexpr double kPracticeHorizontalHallEffectStartAngle = 72 * M_PI / 180.0;

constexpr double kCompHorizontalHallEffectStopAngle = 100 * M_PI / 180.0;
constexpr double kPracticeHorizontalHallEffectStopAngle = 100 * M_PI / 180.0;

constexpr double kPracticeHorizontalUpperPhysicalLimit = 95 * M_PI / 180.0;
constexpr double kCompHorizontalUpperPhysicalLimit = 95 * M_PI / 180.0;

constexpr double kPracticeHorizontalLowerPhysicalLimit = -37.5 * M_PI / 180.0;
constexpr double kCompHorizontalLowerPhysicalLimit = -37.5 * M_PI / 180.0;

constexpr double kPracticeHorizontalUpperLimit = 93 * M_PI / 180.0;
constexpr double kCompHorizontalUpperLimit = 93 * M_PI / 180.0;

constexpr double kPracticeHorizontalLowerLimit = -36 * M_PI / 180.0;
constexpr double kCompHorizontalLowerLimit = -36 * M_PI / 180.0;

constexpr double kHorizontalZeroingSpeed = 1.0;

const int kCompCameraCenter = -2;
const int kPracticeCameraCenter = -5;

struct Values {
  // Wrist hall effect positive and negative edges.
  double horizontal_hall_effect_start_angle;
  double horizontal_hall_effect_stop_angle;

  // Upper and lower extreme limits of travel for the wrist.
  double horizontal_upper_limit;
  double horizontal_lower_limit;
  // Physical limits.  These are here for testing.
  double horizontal_upper_physical_limit;
  double horizontal_lower_physical_limit;

  // Zeroing speed.
  double horizontal_zeroing_speed;

  // what camera_center returns
  int camera_center;
};

Values *values = NULL;
// Attempts to retrieve a new Values instance and stores it in values if
// necessary.
// Returns a valid Values instance or NULL.
const Values *GetValues() {
  // TODO(brians): Make this use the new Once construct.
  if (values == NULL) {
    LOG(INFO, "creating a Constants for team %"PRIu16"\n",
        ::aos::robot_state->team_id);
    switch (::aos::robot_state->team_id) {
      case kCompTeamNumber:
        values = new Values{kCompHorizontalHallEffectStartAngle,
                            kCompHorizontalHallEffectStopAngle,
                            kCompHorizontalUpperLimit,
                            kCompHorizontalLowerLimit,
                            kCompHorizontalUpperPhysicalLimit,
                            kCompHorizontalLowerPhysicalLimit,
                            kHorizontalZeroingSpeed,
                            kCompCameraCenter};
        break;
      case kPracticeTeamNumber:
        values = new Values{kPracticeHorizontalHallEffectStartAngle,
                            kPracticeHorizontalHallEffectStopAngle,
                            kPracticeHorizontalUpperLimit,
                            kPracticeHorizontalLowerLimit,
                            kPracticeHorizontalUpperPhysicalLimit,
                            kPracticeHorizontalLowerPhysicalLimit,
                            kHorizontalZeroingSpeed,
                            kPracticeCameraCenter};
        break;
      default:
        LOG(ERROR, "unknown team #%"PRIu16"\n",
            aos::robot_state->team_id);
        return NULL;
    }
  }
  return values;
}

}  // namespace

bool horizontal_hall_effect_start_angle(double *angle) {
  const Values *const values = GetValues();
  if (values == NULL) return false;
  *angle = values->horizontal_hall_effect_start_angle;
  return true;
}
bool horizontal_hall_effect_stop_angle(double *angle) {
  const Values *const values = GetValues();
  if (values == NULL) return false;
  *angle = values->horizontal_hall_effect_stop_angle;
  return true;
}
bool horizontal_upper_limit(double *angle) {
  const Values *const values = GetValues();
  if (values == NULL) return false;
  *angle = values->horizontal_upper_limit;
  return true;
}

bool horizontal_lower_limit(double *angle) {
  const Values *const values = GetValues();
  if (values == NULL) return false;
  *angle = values->horizontal_lower_limit;
  return true;
}

bool horizontal_upper_physical_limit(double *angle) {
  const Values *const values = GetValues();
  if (values == NULL) return false;
  *angle = values->horizontal_upper_physical_limit;
  return true;
}

bool horizontal_lower_physical_limit(double *angle) {
  const Values *const values = GetValues();
  if (values == NULL) return false;
  *angle = values->horizontal_lower_physical_limit;
  return true;
}

bool horizontal_zeroing_speed(double *speed) {
  const Values *const values = GetValues();
  if (values == NULL) return false;
  *speed = values->horizontal_zeroing_speed;
  return true;
}

bool camera_center(int *center) {
  const Values *const values = GetValues();
  if (values == NULL) return false;
  *center = values->camera_center;
  return true;
}

}  // namespace constants
}  // namespace frc971

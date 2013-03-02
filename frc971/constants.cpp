#include "frc971/constants.h"

#include <stddef.h>
#include <inttypes.h>
#include <math.h>

#include "aos/common/messages/RobotState.q.h"
#include "aos/atom_code/output/MotorOutput.h"

namespace frc971 {
namespace constants {

namespace {

const double kCompWristHallEffectStartAngle = 72 * M_PI / 180.0;
const double kPracticeWristHallEffectStartAngle = 72 * M_PI / 180.0;

const double kCompWristHallEffectStopAngle = 100 * M_PI / 180.0;
const double kPracticeWristHallEffectStopAngle = 100 * M_PI / 180.0;

const double kPracticeWristUpperPhysicalLimit = 95 * M_PI / 180.0;
const double kCompWristUpperPhysicalLimit = 95 * M_PI / 180.0;

const double kPracticeWristLowerPhysicalLimit = -37.5 * M_PI / 180.0;
const double kCompWristLowerPhysicalLimit = -37.5 * M_PI / 180.0;

const double kPracticeWristUpperLimit = 93 * M_PI / 180.0;
const double kCompWristUpperLimit = 93 * M_PI / 180.0;

const double kPracticeWristLowerLimit = -36 * M_PI / 180.0;
const double kCompWristLowerLimit = -36 * M_PI / 180.0;

const double kWristZeroingSpeed = 1.0;

const int kCompCameraCenter = -2;
const int kPracticeCameraCenter = -5;

struct Values {
  // Wrist hall effect positive and negative edges.
  double wrist_hall_effect_start_angle;
  double wrist_hall_effect_stop_angle;

  // Upper and lower extreme limits of travel for the wrist.
  double wrist_upper_limit;
  double wrist_lower_limit;

  // Physical limits.  These are here for testing.
  double wrist_upper_physical_limit;
  double wrist_lower_physical_limit;

  // Zeroing speed.
  double wrist_zeroing_speed;

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
        values = new Values{kCompWristHallEffectStartAngle,
                            kCompWristHallEffectStopAngle,
                            kCompWristUpperLimit,
                            kCompWristLowerLimit,
                            kCompWristUpperPhysicalLimit,
                            kCompWristLowerPhysicalLimit,
                            kWristZeroingSpeed,
                            kCompCameraCenter};
        break;
      case kPracticeTeamNumber:
        values = new Values{kPracticeWristHallEffectStartAngle,
                            kPracticeWristHallEffectStopAngle,
                            kPracticeWristUpperLimit,
                            kPracticeWristLowerLimit,
                            kPracticeWristUpperPhysicalLimit,
                            kPracticeWristLowerPhysicalLimit,
                            kWristZeroingSpeed,
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

bool wrist_hall_effect_start_angle(double *angle) {
  const Values *const values = GetValues();
  if (values == NULL) return false;
  *angle = values->wrist_hall_effect_start_angle;
  return true;
}
bool wrist_hall_effect_stop_angle(double *angle) {
  const Values *const values = GetValues();
  if (values == NULL) return false;
  *angle = values->wrist_hall_effect_stop_angle;
  return true;
}
bool wrist_upper_limit(double *angle) {
  const Values *const values = GetValues();
  if (values == NULL) return false;
  *angle = values->wrist_upper_limit;
  return true;
}

bool wrist_lower_limit(double *angle) {
  const Values *const values = GetValues();
  if (values == NULL) return false;
  *angle = values->wrist_lower_limit;
  return true;
}

bool wrist_upper_physical_limit(double *angle) {
  const Values *const values = GetValues();
  if (values == NULL) return false;
  *angle = values->wrist_upper_physical_limit;
  return true;
}

bool wrist_lower_physical_limit(double *angle) {
  const Values *const values = GetValues();
  if (values == NULL) return false;
  *angle = values->wrist_lower_physical_limit;
  return true;
}

bool wrist_zeroing_speed(double *speed) {
  const Values *const values = GetValues();
  if (values == NULL) return false;
  *speed = values->wrist_zeroing_speed;
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

#include "frc971/constants.h"

#include <stddef.h>
#include <math.h>

#include "aos/common/inttypes.h"
#include "aos/common/messages/RobotState.q.h"
#include "aos/atom_code/output/MotorOutput.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Note: So far, none of the Angle Adjust numbers have been measured.
// Do not rely on them for real life.

namespace frc971 {
namespace constants {

namespace {

// It has about 0.029043 of gearbox slop.
const double kCompWristHallEffectStartAngle = 1.0872860614359172;
const double kPracticeWristHallEffectStartAngle = 1.0872860614359172;

const double kCompWristHallEffectStopAngle = 100 * M_PI / 180.0;
const double kPracticeWristHallEffectStopAngle = 100 * M_PI / 180.0;

const double kPracticeWristUpperPhysicalLimit = 1.677562;
const double kCompWristUpperPhysicalLimit = 1.677562;

const double kPracticeWristLowerPhysicalLimit = -0.746128;
const double kCompWristLowerPhysicalLimit = -0.746128;

const double kPracticeWristUpperLimit = 1.615385;
const double kCompWristUpperLimit = 1.615385;

const double kPracticeWristLowerLimit = -0.746128;
const double kCompWristLowerLimit = -0.746128;

const double kWristZeroingSpeed = 0.25;
const double kWristZeroingOffSpeed = 1.0;

const int kAngleAdjustHallEffect = 2;

const double kCompAngleAdjustHallEffectStartAngle[2] = {0.305432, 1.5};
const double kPracticeAngleAdjustHallEffectStartAngle[2] = {0.305432, 1.5};

const double kCompAngleAdjustHallEffectStopAngle[2] = {0.1, 1.0};
const double kPracticeAngleAdjustHallEffectStopAngle[2] = {0.1, 1.0};

const double kPracticeAngleAdjustUpperPhysicalLimit = 0.894481;
const double kCompAngleAdjustUpperPhysicalLimit = 0.894481;

const double kPracticeAngleAdjustLowerPhysicalLimit = 0.283616;
const double kCompAngleAdjustLowerPhysicalLimit = 0.283616;

const double kPracticeAngleAdjustUpperLimit = 0.85;
const double kCompAngleAdjustUpperLimit = 0.85;

const double kPracticeAngleAdjustLowerLimit = 0.32;
const double kCompAngleAdjustLowerLimit = 0.32;

const double kAngleAdjustZeroingSpeed = -0.2;
const double kAngleAdjustZeroingOffSpeed = -0.5;

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
  // Zeroing off speed.
  double wrist_zeroing_off_speed;

  // AngleAdjust hall effect positive and negative edges.
  const double *angle_adjust_hall_effect_start_angle;
  const double *angle_adjust_hall_effect_stop_angle;

  // Upper and lower extreme limits of travel for the angle adjust.
  double angle_adjust_upper_limit;
  double angle_adjust_lower_limit;
  // Physical limits.  These are here for testing.
  double angle_adjust_upper_physical_limit;
  double angle_adjust_lower_physical_limit;

  // Zeroing speed.
  double angle_adjust_zeroing_speed;
  // Zeroing off speed.
  double angle_adjust_zeroing_off_speed;

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
                            kWristZeroingOffSpeed,
                            kCompAngleAdjustHallEffectStartAngle,
                            kCompAngleAdjustHallEffectStopAngle,
                            kCompAngleAdjustUpperLimit,
                            kCompAngleAdjustLowerLimit,
                            kCompAngleAdjustUpperPhysicalLimit,
                            kCompAngleAdjustLowerPhysicalLimit,
                            kAngleAdjustZeroingSpeed,
                            kAngleAdjustZeroingOffSpeed,
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
                            kWristZeroingOffSpeed,
                            kPracticeAngleAdjustHallEffectStartAngle,
                            kPracticeAngleAdjustHallEffectStopAngle,
                            kPracticeAngleAdjustUpperLimit,
                            kPracticeAngleAdjustLowerLimit,
                            kPracticeAngleAdjustUpperPhysicalLimit,
                            kPracticeAngleAdjustLowerPhysicalLimit,
                            kAngleAdjustZeroingSpeed,
                            kAngleAdjustZeroingOffSpeed,
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

bool wrist_zeroing_off_speed(double *speed) {
  const Values *const values = GetValues();
  if (values == NULL) return false;
  *speed = values->wrist_zeroing_off_speed;
  return true;
}

bool angle_adjust_hall_effect_start_angle(double *angle) {
  const Values *const values = GetValues();
  if (values == NULL) return false;
  angle[0] = values->angle_adjust_hall_effect_start_angle[0];
  angle[1] = values->angle_adjust_hall_effect_start_angle[1];
  return true;
}
bool angle_adjust_hall_effect_stop_angle(double *angle) {
  const Values *const values = GetValues();
  if (values == NULL) return false;
  angle[0] = values->angle_adjust_hall_effect_stop_angle[0];
  angle[1] = values->angle_adjust_hall_effect_stop_angle[1];
  return true;
}
bool angle_adjust_upper_limit(double *angle) {
  const Values *const values = GetValues();
  if (values == NULL) return false;
  *angle = values->angle_adjust_upper_limit;
  return true;
}

bool angle_adjust_lower_limit(double *angle) {
  const Values *const values = GetValues();
  if (values == NULL) return false;
  *angle = values->angle_adjust_lower_limit;
  return true;
}

bool angle_adjust_upper_physical_limit(double *angle) {
  const Values *const values = GetValues();
  if (values == NULL) return false;
  *angle = values->angle_adjust_upper_physical_limit;
  return true;
}

bool angle_adjust_lower_physical_limit(double *angle) {
  const Values *const values = GetValues();
  if (values == NULL) return false;
  *angle = values->angle_adjust_lower_physical_limit;
  return true;
}

bool angle_adjust_zeroing_speed(double *speed) {
  const Values *const values = GetValues();
  if (values == NULL) return false;
  *speed = values->angle_adjust_zeroing_speed;
  return true;
}

bool angle_adjust_zeroing_off_speed(double *speed) {
  const Values *const values = GetValues();
  if (values == NULL) return false;
  *speed = values->angle_adjust_zeroing_off_speed;
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

#include "frc971/constants.h"

#include <stddef.h>
#include <inttypes.h>

#include "aos/common/messages/RobotState.q.h"
#include "aos/atom_code/output/MotorOutput.h"
#include "aos/common/logging/logging.h"

namespace frc971 {
namespace constants {

namespace {

const double kCompHorizontal = -1.77635 + 0.180;
const double kPracticeHorizontal = -1.77635 + -0.073631;
const int kCompCameraCenter = -2;
const int kPracticeCameraCenter = -5;

struct Values {
  // what horizontal_offset returns
  double horizontal;
  // what camera_center returns
  int camera_center;
};
Values *values = NULL;
// Attempts to retrieve a new Values instance and stores it in values if
// necessary.
// Returns a valid Values instance or NULL.
const Values *GetValues() {
  if (values == NULL) {
    LOG(INFO, "creating a Constants for team %"PRIu16"\n",
        aos::robot_state->team_id);
    switch (aos::robot_state->team_id) {
      case kCompTeamNumber:
        values = new Values{kCompHorizontal, kCompCameraCenter};
        break;
      case kPracticeTeamNumber:
        values = new Values{kPracticeHorizontal, kPracticeCameraCenter};
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

bool horizontal_offset(double *horizontal) {
  const Values *const values = GetValues();
  if (values == NULL) return false;
  *horizontal = values->horizontal;
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

#include <stdint.h>

namespace frc971 {
namespace constants {

// Has all of the numbers that change for both robots and makes it easy to
// retrieve the values for the current one.
//
// All of the public functions to retrieve various values take a pointer to
// store their output value into and assume that aos::robot_state->get() is
// not null and is correct.  They return true on success.

const uint16_t kCompTeamNumber = 971;
const uint16_t kPracticeTeamNumber = 5971;

// Sets *horizontal to how many radians from the hall effect transition point
// to horizontal for the wrist.
bool horizontal_offset(double *horizontal);
// Sets *center to how many pixels off center the vertical line
// on the camera view is.
bool camera_center(int *center);

}  // namespace constants
}  // namespace frc971

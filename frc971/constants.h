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

// Sets *angle to how many radians from horizontal to the location of interest.
bool wrist_hall_effect_start_angle(double *angle);
bool wrist_hall_effect_stop_angle(double *angle);
// These are the soft stops for up and down.
bool wrist_lower_limit(double *angle);
bool wrist_upper_limit(double *angle);
// These are the hard stops.  Don't use these for anything but testing.
bool wrist_lower_physical_limit(double *angle);
bool wrist_upper_physical_limit(double *angle);

// Returns the speed to move the wrist at when zeroing in rad/sec
bool wrist_zeroing_speed(double *speed);

// Sets *center to how many pixels off center the vertical line
// on the camera view is.
bool camera_center(int *center);

}  // namespace constants
}  // namespace frc971

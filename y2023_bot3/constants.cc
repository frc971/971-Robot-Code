#include "y2023_bot3/constants.h"

#include <cinttypes>
#include <map>

#if __has_feature(address_sanitizer)
#include "sanitizer/lsan_interface.h"
#endif

#include "absl/base/call_once.h"
#include "glog/logging.h"

#include "aos/mutex/mutex.h"
#include "aos/network/team_number.h"
#include "y2023_bot3/control_loops/superstructure/pivot_joint/integral_pivot_joint_plant.h"

namespace y2023_bot3 {
namespace constants {

Values MakeValues(uint16_t team) {
  LOG(INFO) << "creating a Constants for team: " << team;

  Values r;
  auto *const pivot_joint = &r.pivot_joint;

  r.pivot_joint_flipped = true;

  pivot_joint->subsystem_params.zeroing_voltage = 3.0;
  pivot_joint->subsystem_params.operating_voltage = 12.0;
  pivot_joint->subsystem_params.zeroing_profile_params = {0.5, 3.0};
  pivot_joint->subsystem_params.default_profile_params = {0.5, 5.0};
  pivot_joint->subsystem_params.range = Values::kPivotJointRange();
  pivot_joint->subsystem_params.make_integral_loop =
      control_loops::superstructure::pivot_joint::MakeIntegralPivotJointLoop;
  pivot_joint->subsystem_params.zeroing_constants.average_filter_size =
      Values::kZeroingSampleSize;
  pivot_joint->subsystem_params.zeroing_constants.one_revolution_distance =
      M_PI * 2.0 * constants::Values::kPivotJointEncoderRatio();
  pivot_joint->subsystem_params.zeroing_constants.zeroing_threshold = 0.0005;
  pivot_joint->subsystem_params.zeroing_constants.moving_buffer_size = 20;
  pivot_joint->subsystem_params.zeroing_constants.allowable_encoder_error = 0.9;

  switch (team) {
    // A set of constants for tests.
    case 1:

      pivot_joint->subsystem_params.zeroing_constants
          .measured_absolute_position = 0.0;

      break;

    case kThirdRobotTeamNumber:
      break;

    default:
      LOG(FATAL) << "unknown team: " << team;
  }

  return r;
}

Values MakeValues() { return MakeValues(aos::network::GetTeamNumber()); }

}  // namespace constants
}  // namespace y2023_bot3

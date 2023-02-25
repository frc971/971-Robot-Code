#include "y2023/constants.h"

#include <cinttypes>
#include <map>

#if __has_feature(address_sanitizer)
#include "sanitizer/lsan_interface.h"
#endif

#include "absl/base/call_once.h"
#include "aos/mutex/mutex.h"
#include "aos/network/team_number.h"
#include "glog/logging.h"
#include "y2023/control_loops/superstructure/roll/integral_roll_plant.h"
#include "y2023/control_loops/superstructure/wrist/integral_wrist_plant.h"

namespace y2023 {
namespace constants {

Values MakeValues(uint16_t team) {
  LOG(INFO) << "creating a Constants for team: " << team;

  Values r;
  auto *const arm_proximal = &r.arm_proximal;
  auto *const arm_distal = &r.arm_distal;
  auto *const wrist = &r.wrist;
  auto *const roll_joint = &r.roll_joint;

  arm_proximal->zeroing.average_filter_size = Values::kZeroingSampleSize;
  arm_proximal->zeroing.one_revolution_distance =
      M_PI * 2.0 * constants::Values::kProximalEncoderRatio();
  arm_proximal->zeroing.zeroing_threshold = 0.0005;
  arm_proximal->zeroing.moving_buffer_size = 20;
  arm_proximal->zeroing.allowable_encoder_error = 0.9;

  arm_distal->zeroing.average_filter_size = Values::kZeroingSampleSize;
  arm_distal->zeroing.one_revolution_distance =
      M_PI * 2.0 * constants::Values::kDistalEncoderRatio();
  arm_distal->zeroing.zeroing_threshold = 0.0005;
  arm_distal->zeroing.moving_buffer_size = 20;
  arm_distal->zeroing.allowable_encoder_error = 0.9;

  roll_joint->zeroing.average_filter_size = Values::kZeroingSampleSize;
  roll_joint->zeroing.one_revolution_distance =
      M_PI * 2.0 * constants::Values::kRollJointEncoderRatio();
  roll_joint->zeroing.zeroing_threshold = 0.0005;
  roll_joint->zeroing.moving_buffer_size = 20;
  roll_joint->zeroing.allowable_encoder_error = 0.9;

  wrist->subsystem_params.zeroing_voltage = 3.0;
  wrist->subsystem_params.operating_voltage = 12.0;
  wrist->subsystem_params.zeroing_profile_params = {0.5, 3.0};
  wrist->subsystem_params.default_profile_params = {0.5, 5.0};
  wrist->subsystem_params.range = Values::kWristRange();
  wrist->subsystem_params.make_integral_loop =
      control_loops::superstructure::wrist::MakeIntegralWristLoop;
  wrist->subsystem_params.zeroing_constants.average_filter_size =
      Values::kZeroingSampleSize;
  wrist->subsystem_params.zeroing_constants.one_revolution_distance =
      M_PI * 2.0 * constants::Values::kWristEncoderRatio();
  wrist->subsystem_params.zeroing_constants.zeroing_threshold = 0.0005;
  wrist->subsystem_params.zeroing_constants.moving_buffer_size = 20;
  wrist->subsystem_params.zeroing_constants.allowable_encoder_error = 0.9;
  wrist->subsystem_params.zeroing_constants.middle_position =
      Values::kWristRange().middle();

  switch (team) {
    // A set of constants for tests.
    case 1:
      arm_proximal->zeroing.measured_absolute_position = 0.0;
      arm_proximal->potentiometer_offset = 0.0;

      arm_distal->zeroing.measured_absolute_position = 0.0;
      arm_distal->potentiometer_offset = 0.0;

      roll_joint->zeroing.measured_absolute_position = 0.0;
      roll_joint->potentiometer_offset = 0.0;

      wrist->subsystem_params.zeroing_constants.measured_absolute_position =
          0.0;

      break;

    case kCompTeamNumber:
      arm_proximal->zeroing.measured_absolute_position = 0.0910237406998185;
      arm_proximal->potentiometer_offset = 0.931355973012855 + 8.6743197253382 -
                                           0.101200335326309 -
                                           0.0820901660993467;

      arm_distal->zeroing.measured_absolute_position = 0.556077643172765;
      arm_distal->potentiometer_offset =
          0.436664933370656 + 0.49457213779426 + 6.78213223139724 -
          0.0220711555235029 - 0.0162945074111813 + 0.00630344935527365 -
          0.0164398318919943;

      roll_joint->zeroing.measured_absolute_position = 1.10682573591996;
      roll_joint->potentiometer_offset =
          3.87038557084874 - 0.0241774522172967 + 0.0711345168020632 -
          0.866186131631967 - 0.0256788357596952 + 0.18101759154572017 -
          0.0208958996127179 - 0.186395903925026 + 0.45801689548395 -
          0.5935210745062;

      wrist->subsystem_params.zeroing_constants.measured_absolute_position =
          0.183283543884167;

      break;

    case kPracticeTeamNumber:
      arm_proximal->zeroing.measured_absolute_position = 0.0;
      arm_proximal->potentiometer_offset = 0.0;

      arm_distal->zeroing.measured_absolute_position = 0.0;
      arm_distal->potentiometer_offset = 0.0;

      roll_joint->zeroing.measured_absolute_position = 0.0;
      roll_joint->potentiometer_offset = 0.0;

      wrist->subsystem_params.zeroing_constants.measured_absolute_position =
          0.0;

      break;

    case kCodingRobotTeamNumber:
      arm_proximal->zeroing.measured_absolute_position = 0.0;
      arm_proximal->potentiometer_offset = 0.0;

      arm_distal->zeroing.measured_absolute_position = 0.0;
      arm_distal->potentiometer_offset = 0.0;

      roll_joint->zeroing.measured_absolute_position = 0.0;
      roll_joint->potentiometer_offset = 0.0;

      wrist->subsystem_params.zeroing_constants.measured_absolute_position =
          0.0;

      break;

    default:
      LOG(FATAL) << "unknown team: " << team;

      // TODO(milind): add pot range checks once we add ranges
  }

  return r;
}

Values MakeValues() { return MakeValues(aos::network::GetTeamNumber()); }

}  // namespace constants
}  // namespace y2023

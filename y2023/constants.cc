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

  ::frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemParams<
      ::frc971::zeroing::PotAndAbsoluteEncoderZeroingEstimator>
      *const roll_joint_params = &roll_joint->subsystem_params;

  roll_joint_params->zeroing_voltage = 3.0;
  roll_joint_params->operating_voltage = 12.0;
  roll_joint_params->zeroing_profile_params = {0.5, 3.0};
  roll_joint_params->default_profile_params = {6.0, 30.0};
  roll_joint_params->range = Values::kRollJointRange();
  roll_joint_params->make_integral_loop =
      control_loops::superstructure::roll::MakeIntegralRollLoop;
  roll_joint_params->zeroing_constants.average_filter_size =
      Values::kZeroingSampleSize;
  roll_joint_params->zeroing_constants.one_revolution_distance =
      M_PI * 2.0 * constants::Values::kRollJointEncoderRatio();
  roll_joint_params->zeroing_constants.zeroing_threshold = 0.0005;
  roll_joint_params->zeroing_constants.moving_buffer_size = 20;
  roll_joint_params->zeroing_constants.allowable_encoder_error = 0.9;

  wrist->zeroing_voltage = 3.0;
  wrist->operating_voltage = 12.0;
  wrist->zeroing_profile_params = {0.5, 3.0};
  wrist->default_profile_params = {6.0, 30.0};
  wrist->range = Values::kWristRange();
  wrist->make_integral_loop =
      control_loops::superstructure::wrist::MakeIntegralWristLoop;
  wrist->zeroing_constants.average_filter_size = Values::kZeroingSampleSize;
  wrist->zeroing_constants.one_revolution_distance =
      M_PI * 2.0 * constants::Values::kWristEncoderRatio();
  wrist->zeroing_constants.zeroing_threshold = 0.0005;
  wrist->zeroing_constants.moving_buffer_size = 20;
  wrist->zeroing_constants.allowable_encoder_error = 0.9;
  wrist->zeroing_constants.middle_position = Values::kWristRange().middle();

  switch (team) {
    // A set of constants for tests.
    case 1:
      arm_proximal->zeroing.measured_absolute_position = 0.0;
      arm_proximal->potentiometer_offset = 0.0;

      arm_distal->zeroing.measured_absolute_position = 0.0;
      arm_distal->potentiometer_offset = 0.0;

      roll_joint_params->zeroing_constants.measured_absolute_position = 0.0;
      roll_joint->potentiometer_offset = 0.0;

      wrist->zeroing_constants.measured_absolute_position = 0.0;

      break;

    case kCompTeamNumber:
      arm_proximal->zeroing.measured_absolute_position = 0.0;
      arm_proximal->potentiometer_offset = 0.0;

      arm_distal->zeroing.measured_absolute_position = 0.0;
      arm_distal->potentiometer_offset = 0.0;

      roll_joint_params->zeroing_constants.measured_absolute_position = 0.0;
      roll_joint->potentiometer_offset = 0.0;

      wrist->zeroing_constants.measured_absolute_position = 0.0;

      break;

    case kPracticeTeamNumber:
      arm_proximal->zeroing.measured_absolute_position = 0.0;
      arm_proximal->potentiometer_offset = 0.0;

      arm_distal->zeroing.measured_absolute_position = 0.0;
      arm_distal->potentiometer_offset = 0.0;

      roll_joint_params->zeroing_constants.measured_absolute_position = 0.0;
      roll_joint->potentiometer_offset = 0.0;

      wrist->zeroing_constants.measured_absolute_position = 0.0;

      break;

    case kCodingRobotTeamNumber:
      arm_proximal->zeroing.measured_absolute_position = 0.0;
      arm_proximal->potentiometer_offset = 0.0;

      arm_distal->zeroing.measured_absolute_position = 0.0;
      arm_distal->potentiometer_offset = 0.0;

      roll_joint_params->zeroing_constants.measured_absolute_position = 0.0;
      roll_joint->potentiometer_offset = 0.0;

      wrist->zeroing_constants.measured_absolute_position = 0.0;

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

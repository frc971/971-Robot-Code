#include "y2018/constants.h"

#include <inttypes.h>
#include <math.h>
#include <stdint.h>

#include <map>

#if __has_feature(address_sanitizer)
#include "sanitizer/lsan_interface.h"
#endif

#include "aos/common/logging/logging.h"
#include "aos/common/mutex.h"
#include "aos/common/network/team_number.h"
#include "aos/once.h"

#include "y2018/control_loops/drivetrain/drivetrain_dog_motor_plant.h"
#include "y2018/control_loops/drivetrain/polydrivetrain_dog_motor_plant.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace y2018 {
namespace constants {
namespace {

const uint16_t kCompTeamNumber = 971;
const uint16_t kPracticeTeamNumber = 9971;

const Values *DoGetValuesForTeam(uint16_t team) {
  Values *const r = new Values();
  Values::IntakeSide *const left_intake = &r->left_intake;
  Values::IntakeSide *const right_intake = &r->right_intake;
  Values::Proximal *const arm_proximal = &r->arm_proximal;
  Values::Distal *const arm_distal = &r->arm_distal;

  left_intake->zeroing.average_filter_size = Values::kZeroingSampleSize();
  left_intake->zeroing.one_revolution_distance =
      M_PI * 2.0 * constants::Values::kIntakeMotorEncoderRatio();
  left_intake->zeroing.zeroing_threshold = 0.0005;
  left_intake->zeroing.moving_buffer_size = 20;
  left_intake->zeroing.allowable_encoder_error = 1.9;

  *right_intake = *left_intake;

  arm_proximal->zeroing.average_filter_size = Values::kZeroingSampleSize();
  arm_proximal->zeroing.one_revolution_distance =
      M_PI * 2.0 * constants::Values::kProximalEncoderRatio();
  arm_proximal->zeroing.zeroing_threshold = 0.0005;
  arm_proximal->zeroing.moving_buffer_size = 20;
  arm_proximal->zeroing.allowable_encoder_error = 0.9;

  arm_distal->zeroing.average_filter_size = Values::kZeroingSampleSize();
  arm_distal->zeroing.one_revolution_distance =
      M_PI * 2.0 * constants::Values::kDistalEncoderRatio();
  arm_distal->zeroing.zeroing_threshold = 0.0005;
  arm_distal->zeroing.moving_buffer_size = 20;
  arm_distal->zeroing.allowable_encoder_error = 0.9;

  switch (team) {
    // A set of constants for tests.
    case 1:
      r->vision_name = "test";
      r->vision_error = -0.030;

      left_intake->zeroing.measured_absolute_position = 0.0;
      left_intake->potentiometer_offset = 0.0;
      left_intake->spring_offset = 0.0;

      right_intake->zeroing.measured_absolute_position = 0.0;
      right_intake->potentiometer_offset = 0.0;
      right_intake->spring_offset = 0.0;

      arm_proximal->zeroing.measured_absolute_position = 0.0;
      arm_proximal->potentiometer_offset = 0.0;

      arm_distal->zeroing.measured_absolute_position = 0.0;
      arm_distal->potentiometer_offset = 0.0;
      break;

    case kCompTeamNumber:
      r->vision_name = "competition";
      r->vision_error = 0.0;

      left_intake->zeroing.measured_absolute_position = 0.0;
      left_intake->potentiometer_offset = 0.0;
      left_intake->spring_offset = 0.0;

      right_intake->zeroing.measured_absolute_position = 0.0;
      right_intake->potentiometer_offset = 0.0;
      right_intake->spring_offset = 0.0;

      arm_proximal->zeroing.measured_absolute_position = 0.0;
      arm_proximal->potentiometer_offset = 0.0;

      arm_distal->zeroing.measured_absolute_position = 0.0;
      arm_distal->potentiometer_offset = 0.0;
      break;

    case kPracticeTeamNumber:
      r->vision_name = "practice";
      r->vision_error = 0.0;

      left_intake->zeroing.measured_absolute_position = 0.3332;
      left_intake->potentiometer_offset = -10.55;
      left_intake->spring_offset = -0.249;

      right_intake->zeroing.measured_absolute_position = 0.539284;
      right_intake->potentiometer_offset = 9.59;
      right_intake->spring_offset = 0.255;

      arm_proximal->zeroing.measured_absolute_position = 0.1877;
      arm_proximal->potentiometer_offset = -1.242;

      arm_distal->zeroing.measured_absolute_position = 0.28366 + M_PI;
      arm_distal->potentiometer_offset = 2.772210 + M_PI;
      break;

    default:
      LOG(FATAL, "unknown team #%" PRIu16 "\n", team);
  }

  return r;
}

const Values &DoGetValues() {
  uint16_t team = ::aos::network::GetTeamNumber();
  LOG(INFO, "creating a Constants for team %" PRIu16 "\n", team);
  return GetValuesForTeam(team);
}

}  // namespace

const Values &GetValues() { return DoGetValues(); }

const Values &GetValuesForTeam(uint16_t team_number) {
  static ::aos::Mutex mutex;
  ::aos::MutexLocker locker(&mutex);

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
}  // namespace y2018

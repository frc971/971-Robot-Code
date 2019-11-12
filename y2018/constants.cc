#include "y2018/constants.h"

#include <inttypes.h>
#include <math.h>
#include <stdint.h>

#include <map>

#if __has_feature(address_sanitizer)
#include "sanitizer/lsan_interface.h"
#endif

#include "aos/logging/logging.h"
#include "aos/mutex/mutex.h"
#include "aos/network/team_number.h"

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

  constexpr double kDistalZeroingPosition =
      M_PI * 3.0 / 2.0 + (28.5 / 180.0) * M_PI;
  // 5.209807817203074

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

      left_intake->zeroing.measured_absolute_position = 0.219024;
      left_intake->potentiometer_offset = -5.45258 + 1.299206 - 0.525603;
      left_intake->spring_offset = -0.25 - 0.009 + 0.029 - 0.025;

      right_intake->zeroing.measured_absolute_position = 0.37022 - 0.04;
      right_intake->potentiometer_offset = 3.739919 + 1.087098 + 0.825;
      right_intake->spring_offset = 0.25 + 0.015 - 0.025;

      arm_proximal->zeroing.measured_absolute_position =
          0.067941 + 1.047 - 0.116 + 0.06 - 0.004 + 0.009 + 0.0938;
      arm_proximal->potentiometer_offset =
          1.047 - 3.653298 + -0.078 + 0.9455 + 0.265 - 0.36;

      arm_distal->zeroing.measured_absolute_position =
          -0.870445 + 5.209807817203074 + 0.118 - 0.004 + 0.407 - 0.53;
      arm_distal->potentiometer_offset = 5.209807817203074 + 1.250476 + 0.110 + 0.52;
      break;

    case kPracticeTeamNumber:
      r->vision_name = "practice";
      r->vision_error = 0.0;

      left_intake->zeroing.measured_absolute_position = 0.031709;
      left_intake->potentiometer_offset = -10.55 - 3.621232 + 4.996959;
      left_intake->spring_offset = -0.249 - 0.002;

      right_intake->zeroing.measured_absolute_position = 0.351376;
      right_intake->potentiometer_offset = 9.59 + 1.530320 - 3.620648;
      right_intake->spring_offset = 0.255 + 0.008 - 0.09;

      arm_proximal->zeroing.measured_absolute_position = -0.253183 + 1.0652774488034022 + 0.009566448803402405;
      arm_proximal->potentiometer_offset = -1.242 - 0.03 - 0.1 - 1.0652;

      arm_distal->zeroing.measured_absolute_position =
          1.102987 - kDistalZeroingPosition + 0.12 + 0.0095 + 0.22300918279692628;
      arm_distal->potentiometer_offset =
          2.772210 + M_PI + 0.434 - 0.12 + 1.25 - 0.226 + 0.862067 - 0.121925182796926;
      break;

    default:
      AOS_LOG(FATAL, "unknown team #%" PRIu16 "\n", team);
  }

  return r;
}

const Values &DoGetValues() {
  const uint16_t team = ::aos::network::GetTeamNumber();
  AOS_LOG(INFO, "creating a Constants for team %" PRIu16 "\n", team);
  return GetValuesForTeam(team);
}

}  // namespace

const Values &GetValues() {
  static const Values r = DoGetValues();
  return r;
}

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

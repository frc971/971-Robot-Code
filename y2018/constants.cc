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
  Values::Intake *const intake = &r->intake;
  Values::Proximal *const proximal = &r->proximal;
  Values::Distal *const distal = &r->distal;

  switch (team) {
    // A set of constants for tests.
    case 1:
      r->down_error = 0;
      r->vision_name = "test";
      r->vision_error = -0.030;
      intake->left_pot_offset = 0;
      intake->right_pot_offset = 0;
      proximal->pot_offset = 0;
      distal->pot_offset = 0;
      break;

    case kCompTeamNumber:
      r->down_error = 0;
      r->vision_name = "competition";
      r->vision_error = 0.0;
      intake->left_pot_offset = 0;
      intake->right_pot_offset = 0;
      proximal->pot_offset = 0;
      distal->pot_offset = 0;
      break;

    case kPracticeTeamNumber:
      r->down_error = 0;
      r->vision_name = "practice";
      r->vision_error = 0.0;
      intake->left_pot_offset = 0;
      intake->right_pot_offset = 0;
      proximal->pot_offset = 0;
      distal->pot_offset = 0;
      break;

    default:
      LOG(FATAL, "unknown team #%" PRIu16 "\n", team);
  }

  return r;
}

const Values *DoGetValues() {
  uint16_t team = ::aos::network::GetTeamNumber();
  LOG(INFO, "creating a Constants for team %" PRIu16 "\n", team);
  return DoGetValuesForTeam(team);
}

}  // namespace

const Values &GetValues() {
  const Values &r = *DoGetValues();
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

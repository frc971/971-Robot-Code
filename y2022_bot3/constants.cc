#include "y2022_bot3/constants.h"

#include <cinttypes>
#include <map>

#if __has_feature(address_sanitizer)
#include "sanitizer/lsan_interface.h"
#endif

#include "absl/base/call_once.h"
#include "aos/mutex/mutex.h"
#include "aos/network/team_number.h"
#include "glog/logging.h"

namespace y2022_bot3 {
namespace constants {

const int Values::kZeroingSampleSize;

namespace {

const uint16_t kCompTeamNumber = 971;
const uint16_t kPracticeTeamNumber = 9971;
const uint16_t kCodingRobotTeamNumber = 7971;

}  // namespace

Values MakeValues(uint16_t team) {
  LOG(INFO) << "creating a Constants for team: " << team;

  Values r;

  switch (team) {
    // A set of constants for tests.
    case 1:
      break;

    case kCompTeamNumber:
      break;

    case kPracticeTeamNumber:
      break;

    case kCodingRobotTeamNumber:
      break;

    default:
      LOG(FATAL) << "unknown team: " << team;
  }

  return r;
}

Values MakeValues() { return MakeValues(aos::network::GetTeamNumber()); }

}  // namespace constants
}  // namespace y2022_bot3

#include "y2022/constants.h"

#include <cinttypes>
#include <map>

#if __has_feature(address_sanitizer)
#include "sanitizer/lsan_interface.h"
#endif

#include "absl/base/call_once.h"
#include "aos/mutex/mutex.h"
#include "aos/network/team_number.h"
#include "glog/logging.h"

namespace y2022 {
namespace constants {

const int Values::kZeroingSampleSize;

namespace {

const uint16_t kCompTeamNumber = 971;
const uint16_t kPracticeTeamNumber = 9971;
const uint16_t kCodingRobotTeamNumber = 7971;

const Values *DoGetValuesForTeam(uint16_t team) {
  Values *const r = new Values();

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

const Values *values = nullptr;

void DoGetValues() {
  uint16_t team = ::aos::network::GetTeamNumber();
  LOG(INFO) << "creating a Constants for team: " << team;
  values = DoGetValuesForTeam(team);
}

}  // namespace

void InitValues() {
  static absl::once_flag once;
  absl::call_once(once, DoGetValues);
}

const Values &GetValues() {
  CHECK(values)
      << "Values are uninitialized. Call InitValues before accessing them.";
  return *values;
}

}  // namespace constants
}  // namespace y2022

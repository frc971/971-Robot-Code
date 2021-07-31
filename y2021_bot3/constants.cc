#include "y2021_bot3/constants.h"

#include <cinttypes>
#include <map>

#if __has_feature(address_sanitizer)
#include "sanitizer/lsan_interface.h"
#endif

#include "absl/base/call_once.h"
#include "aos/logging/logging.h"
#include "aos/mutex/mutex.h"
#include "aos/network/team_number.h"

namespace y2021_bot3 {
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
      AOS_LOG(FATAL, "unknown team #%" PRIu16 "\n", team);
  }

  return r;
}

void DoGetValues(const Values **result) {
  uint16_t team = ::aos::network::GetTeamNumber();
  AOS_LOG(INFO, "creating a Constants for team %" PRIu16 "\n", team);
  *result = DoGetValuesForTeam(team);
}

}  // namespace

const Values &GetValues() {
  static absl::once_flag once;
  static const Values *result;
  absl::call_once(once, DoGetValues, &result);
  return *result;
}

const Values &GetValuesForTeam(uint16_t team_number) {
  static ::aos::Mutex mutex;
  ::aos::MutexLocker locker(&mutex);

  // IMPORTANT: This declaration has to stay after the mutex is locked to avoid
  // race conditions.
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
}  // namespace y2021_bot3

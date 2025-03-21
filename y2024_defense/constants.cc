#include "y2024_defense/constants.h"

#include <cinttypes>
#include <map>

#if __has_feature(address_sanitizer)
#include "sanitizer/lsan_interface.h"
#endif

#include "absl/base/call_once.h"
#include "absl/log/check.h"
#include "absl/log/log.h"

#include "aos/mutex/mutex.h"
#include "aos/network/team_number.h"

namespace y2024_defense::constants {

Values MakeValues(uint16_t team) {
  LOG(INFO) << "creating a Constants for team: " << team;

  Values r;

  return r;
}

Values MakeValues() { return MakeValues(aos::network::GetTeamNumber()); }

}  // namespace y2024_defense::constants

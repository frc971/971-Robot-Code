#include "aos/network/team_number.h"

#include <netinet/in.h>
#include <inttypes.h>
#include <unistd.h>
#include <stdlib.h>

#include <string>

#include "absl/base/call_once.h"
#include "aos/logging/logging.h"
#include "aos/util/string_to_num.h"

namespace aos {
namespace network {
namespace internal {
int ParseTeamNumber(const std::string &hostname, uint16_t *teamnumber) {
  for (size_t i = 0; i < hostname.size(); i++) {
    if (hostname[i] == '-') {
      const std::string num_as_s =
          hostname[hostname.size() - 1] == 'C'
              ? hostname.substr(i + 1, hostname.size() - 5 - i)
              : hostname.substr(i + 1);

      int num;
      if (!::aos::util::StringToNumber(num_as_s, &num)) {
        return -1;
      }
      if (hostname.substr(0, i) == "roboRIO" &&
          std::to_string(num) == num_as_s) {
        *teamnumber = num;
        return 0;
      } else {
        return -1;
      }
    }
  }
  return -1;
}
}  // namespace internal

namespace {

uint16_t override_team;

void DoGetTeamNumber(uint16_t *result) {
  if (override_team != 0) {
      *result = override_team;
      return;
  }

  const char *override_number = getenv("AOS_TEAM_NUMBER");
  if (override_number != nullptr) {
    if (!::aos::util::StringToNumber(override_number, result)) {
      AOS_LOG(FATAL, "error parsing AOS_TEAM_NUMBER '%s'\n", override_number);
    }
    AOS_LOG(WARNING,
            "team number overridden by AOS_TEAM_NUMBER to %" PRIu16 "\n", *result);
  } else {
    int error = internal::ParseTeamNumber(GetHostname(), result);
    if (error) {
      AOS_LOG(FATAL, "Invalid hostname %s\n", GetHostname().c_str());
    }
    AOS_LOG(INFO, "team number is %" PRIu16 "\n", *result);
  }
}

}  // namespace

::std::string GetHostname() {
  char buf[256];
  buf[sizeof(buf) - 1] = '\0';
  AOS_PCHECK(gethostname(buf, sizeof(buf) - 1));
  return buf;
}

uint16_t GetTeamNumber() {
  static absl::once_flag once;
  static uint16_t result;
  absl::call_once(once, DoGetTeamNumber, &result);
  return result;
}

void OverrideTeamNumber(uint16_t team) { override_team = team; }

}  // namespace network
}  // namespace aos

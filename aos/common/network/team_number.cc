#include "aos/common/network/team_number.h"

#include <netinet/in.h>
#include <inttypes.h>
#include <unistd.h>
#include <stdlib.h>

#include <string>

#include "aos/common/once.h"
#include "aos/linux_code/configuration.h"
#include "aos/common/logging/logging.h"
#include "aos/common/util/string_to_num.h"

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

::std::string GetHostname() {
  char buf[256];
  buf[sizeof(buf) - 1] = '\0';
  PCHECK(gethostname(buf, sizeof(buf) - 1));
  return buf;
}

uint16_t *DoGetTeamNumber() {
  if (override_team != 0) return &override_team;

  static uint16_t r;

  const char *override_number = getenv("AOS_TEAM_NUMBER");
  if (override_number != nullptr) {
    if (!::aos::util::StringToNumber(override_number, &r)) {
      LOG(FATAL, "error parsing AOS_TEAM_NUMBER '%s'\n", override_number);
    }
    LOG(WARNING, "team number overridden by AOS_TEAM_NUMBER to %" PRIu16 "\n",
        r);
  } else {
    int error = internal::ParseTeamNumber(GetHostname(), &r);
    if (error) {
      LOG(FATAL, "Invalid hostname %s\n", GetHostname().c_str());
    }
    LOG(INFO, "team number is %" PRIu16 "\n", r);
  }
  return &r;
}

}  // namespace

uint16_t GetTeamNumber() {
  static Once<uint16_t> once(DoGetTeamNumber);
  return *once.Get();
}

void OverrideTeamNumber(uint16_t team) { override_team = team; }

}  // namespace network
}  // namespace aos

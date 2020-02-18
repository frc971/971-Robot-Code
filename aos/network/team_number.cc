#include "aos/network/team_number.h"

#include <netinet/in.h>
#include <inttypes.h>
#include <unistd.h>
#include <stdlib.h>

#include "glog/logging.h"

#include "aos/util/string_to_num.h"

namespace aos {
namespace network {
namespace team_number_internal {

std::optional<uint16_t> ParseRoborioTeamNumber(const std::string &hostname) {
  for (size_t i = 0; i < hostname.size(); i++) {
    if (hostname[i] == '-') {
      const std::string num_as_s =
          hostname[hostname.size() - 1] == 'C'
              ? hostname.substr(i + 1, hostname.size() - 5 - i)
              : hostname.substr(i + 1);

      int num;
      if (!::aos::util::StringToNumber(num_as_s, &num)) {
        return std::nullopt;
      }
      if (hostname.substr(0, i) == "roboRIO" &&
          std::to_string(num) == num_as_s) {
        return num;
      }
      return std::nullopt;
    }
  }
  return std::nullopt;
}

std::optional<uint16_t> ParsePiTeamNumber(const std::string &hostname) {
  if (hostname.substr(0, 3) != "pi-") {
    return std::nullopt;
  }
  size_t first_separator = hostname.find('-');
  if (first_separator == hostname.npos ||
      first_separator >= hostname.size() - 2) {
    return std::nullopt;
  }
  ++first_separator;
  const size_t second_separator = hostname.find('-', first_separator);
  if (second_separator == hostname.npos) {
    return std::nullopt;
  }
  const std::string number_string =
      hostname.substr(first_separator, second_separator - first_separator);
  int number;
  if (!util::StringToNumber(number_string, &number)) {
    return std::nullopt;
  }
  return number;
}

}  // namespace team_number_internal

namespace {

uint16_t override_team;

uint16_t DoGetTeamNumber() {
  if (override_team != 0) {
      return override_team;
  }

  const char *override_number = getenv("AOS_TEAM_NUMBER");
  if (override_number != nullptr) {
    uint16_t result;
    if (!::aos::util::StringToNumber(override_number, &result)) {
      LOG(FATAL) << "Error parsing AOS_TEAM_NUMBER: " << override_number;
    }
    LOG(WARNING)
        << "Team number overriden by AOS_TEAM_NUMBER environment variable to "
        << result;
    return result;
  }
  const auto hostname = GetHostname();
  {
    const auto result = team_number_internal::ParseRoborioTeamNumber(hostname);
    if (result) {
      LOG(INFO) << "roboRIO hostname team number is: " << *result;
      return *result;
    }
  }
  {
    const auto result = team_number_internal::ParsePiTeamNumber(hostname);
    if (result) {
      LOG(INFO) << "Pi hostname team number is: " << *result;
      return *result;
    }
  }
  LOG(FATAL) << "Failed to parse a team number from hostname: " << hostname;
}

}  // namespace

::std::string GetHostname() {
  char buf[256];
  buf[sizeof(buf) - 1] = '\0';
  PCHECK(gethostname(buf, sizeof(buf) - 1) == 0);
  return buf;
}

uint16_t GetTeamNumber() {
  const static uint16_t result = DoGetTeamNumber();
  return result;
}

void OverrideTeamNumber(uint16_t team) { override_team = team; }

}  // namespace network
}  // namespace aos

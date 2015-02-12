#include "aos/common/network/team_number.h"

#include <netinet/in.h>
#include <inttypes.h>

#include "aos/common/once.h"
#include "aos/linux_code/configuration.h"
#include "aos/common/logging/logging.h"

namespace aos {
namespace network {
namespace {

uint16_t override_team;

uint16_t *DoGetTeamNumber() {
  if (override_team != 0) return &override_team;
  const in_addr &address = configuration::GetOwnIPAddress();
  static uint16_t r =
      (((address.s_addr & 0xFF00) >> 8) * 100) +
      (((address.s_addr & 0xFF0000) >> 16) & 0xFF);
  LOG(INFO, "team number is %" PRIu16 "\n", r);
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

#include "aos/common/network/team_number.h"

#include <netinet/in.h>

#include "aos/common/once.h"
#include "aos/atom_code/configuration.h"

namespace aos {
namespace network {
namespace {

uint16_t *DoGetTeamNumber() {
  const in_addr &address = configuration::GetOwnIPAddress();
  static uint16_t r = address.s_addr >> 8;
  return &r;
}

}  // namespace

uint16_t GetTeamNumber() {
  static Once<uint16_t> once(DoGetTeamNumber);
  return *once.Get();
}

}  // namespace network
}  // namespace aos

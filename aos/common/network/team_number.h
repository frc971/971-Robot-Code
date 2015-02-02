#ifndef AOS_COMMON_NETWORK_TEAM_NUMBER_H_
#define AOS_COMMON_NETWORK_TEAM_NUMBER_H_

#include <stdint.h>

namespace aos {
namespace network {

// Retrieves the current team number based off of the network address.
// This function will only do the complicated stuff once so it is cheap to call
// repeatedly.
uint16_t GetTeamNumber();

// Overrides the team number returned from GetTeamNumber(). Must be called
// before GetTeamNumber() is ever called.
// Overriding to team 0 won't work.
// Intended only for tests.
// Guaranteed to be safe to call during static initialization time.
void OverrideTeamNumber(uint16_t team);

}  // namespace network
}  // namespace aos

#endif  // AOS_COMMON_NETWORK_TEAM_NUMBER_H_

#ifndef AOS_COMMON_NETWORK_TEAM_NUMBER_H_
#define AOS_COMMON_NETWORK_TEAM_NUMBER_H_

#include <stdint.h>

namespace aos {
namespace network {

// Retrieves the current team number based off of the network address.
// This function will only do the complicated stuff once so it is cheap to call
// repeatedly.
uint16_t GetTeamNumber();

}  // namespace network
}  // namespace aos

#endif  // AOS_COMMON_NETWORK_TEAM_NUMBER_H_

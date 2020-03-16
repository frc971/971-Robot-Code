#ifndef AOS_NETWORK_TEAM_NUMBER_H_
#define AOS_NETWORK_TEAM_NUMBER_H_

#include <stdint.h>

#include <optional>
#include <string>

namespace aos {
namespace network {

// Retrieves the current team number based off of the network address.
// This function will only do the complicated stuff once so it is cheap to call
// repeatedly.
uint16_t GetTeamNumber();

// Returns the current hostname.
::std::string GetHostname();

// Overrides the team number returned from GetTeamNumber(). Must be called
// before GetTeamNumber() is ever called.
// Overriding to team 0 won't work.
// Intended only for tests.
// Guaranteed to be safe to call during static initialization time.
void OverrideTeamNumber(uint16_t team);

// Returns the pi number for a pi formated hostname.  pi-team#-pi# (pi-971-5)
std::optional<uint16_t> ParsePiNumber(const std::string &hostname);

namespace team_number_internal {

std::optional<uint16_t> ParseRoborioTeamNumber(const std::string &hostname);

// Returns the team number for a pi formated hostname.  pi-team#-pi#
std::optional<uint16_t> ParsePiTeamNumber(const std::string &hostname);

}  // namespace team_number_internal
}  // namespace network
}  // namespace aos

#endif  // AOS_NETWORK_TEAM_NUMBER_H_

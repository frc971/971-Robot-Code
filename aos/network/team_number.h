#ifndef AOS_NETWORK_TEAM_NUMBER_H_
#define AOS_NETWORK_TEAM_NUMBER_H_

#include <cstdint>
#include <optional>
#include <string_view>

#include "absl/flags/declare.h"

ABSL_DECLARE_FLAG(std::string, override_hostname);

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

// Returns the number for a pi/orin formatted hostname.  pi-team#-pi# (e.g., 5
// for pi-971-5 or 2 for orin-9971-2)
std::optional<uint16_t> ParsePiOrOrinNumber(const std::string_view hostname);

// Returns whether the device is a "pi" or an "orin" based on hostname
std::optional<std::string_view> ParsePiOrOrin(const std::string_view hostname);

namespace team_number_internal {

std::optional<uint16_t> ParseRoborioTeamNumber(const std::string_view hostname);

// Returns the team number for a pi/orin formatted hostname.  pi-team#-pi#
// (e.g., 971 for pi-971-5 or 9971 for orin-9971-2)
std::optional<uint16_t> ParsePiOrOrinTeamNumber(
    const std::string_view hostname);

}  // namespace team_number_internal
}  // namespace network
}  // namespace aos

#endif  // AOS_NETWORK_TEAM_NUMBER_H_

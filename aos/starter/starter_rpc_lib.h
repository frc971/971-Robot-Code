#ifndef AOS_STARTER_STARTER_RPC_LIB_H_
#define AOS_STARTER_STARTER_RPC_LIB_H_

#include <chrono>
#include <optional>

#include "aos/configuration.h"
#include "aos/starter/starter_generated.h"
#include "aos/starter/starter_rpc_generated.h"

namespace aos {
namespace starter {

// Finds the status of an individual application within a starter status message
// Returns nullptr if no application found by the given name.
const aos::starter::ApplicationStatus *FindApplicationStatus(
    const aos::starter::Status &status, std::string_view name);

// Checks if the name is an executable name and if it is, it returns that
// application's name, otherwise returns name as given
std::string_view FindApplication(const std::string_view &name,
                                 const aos::Configuration *config);

// Sends the given command to the application with the name name. Creates a
// temporary event loop from the provided config for sending the command and
// receiving back status messages. Returns true if the command executed
// successfully, or false otherwise. Returns false if the desired state was not
// achieved within timeout.
bool SendCommandBlocking(aos::starter::Command, std::string_view name,
                         const aos::Configuration *config,
                         std::chrono::milliseconds timeout);

// Sends lots of commands and waits for them all to succeed.  There must not be
// more than 1 conflicting command in here which modifies the state of a single
// application otherwise it will never succeed.  An example is having both a
// start and stop command for a single application.
bool SendCommandBlocking(
    std::vector<std::pair<aos::starter::Command, std::string_view>> commands,
    const aos::Configuration *config, std::chrono::milliseconds timeout);

// Fetches the status of the application with the given name. Creates a
// temporary event loop from the provided config for fetching. Returns an empty
// flatbuffer if the application is not found.
const aos::FlatbufferDetachedBuffer<aos::starter::ApplicationStatus> GetStatus(
    std::string_view name, const aos::Configuration *config);

// Fetches the entire status message of starter. Creates a temporary event loop
// from the provided config for fetching.
std::optional<const aos::FlatbufferVector<aos::starter::Status>>
GetStarterStatus(const aos::Configuration *config);

}  // namespace starter
}  // namespace aos

#endif  // AOS_STARTER_STARTER_RPC_LIB_H_

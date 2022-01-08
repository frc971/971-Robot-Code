#ifndef AOS_STARTER_STARTER_RPC_LIB_H_
#define AOS_STARTER_STARTER_RPC_LIB_H_

#include <chrono>
#include <map>
#include <optional>
#include <vector>

#include "aos/configuration.h"
#include "aos/events/event_loop.h"
#include "aos/starter/starter_generated.h"
#include "aos/starter/starter_rpc_generated.h"

namespace aos {
namespace starter {

// Data required to command that starter start/stop/restart a given application.
struct ApplicationCommand {
  Command command;
  std::string_view application;
  std::vector<const aos::Node *> nodes;
};

// This class manages interacting with starterd so that you can conveniently
// start/stop applications programmatically.
// Note that the StarterClient only maintains internal state for a single set of
// commands at once, so once the user calls SendCommands() they must wait for
// the timeout or success handler to be called before calling SendCommands
// again.
class StarterClient {
 public:
  StarterClient(EventLoop *event_loop);

  void SendCommands(const std::vector<ApplicationCommand> &commands,
                    monotonic_clock::duration timeout);

  void SetTimeoutHandler(std::function<void()> handler) {
    timeout_handler_ = handler;
  }

  void SetSuccessHandler(std::function<void()> handler) {
    success_handler_ = handler;
  }

 private:
  struct CommandStatus {
    State expected_state;
    std::string application;
    std::optional<uint64_t> old_id;
  };

  bool CheckCommandsSucceeded();

  void Timeout();

  void Succeed();

  EventLoop *event_loop_;
  TimerHandler *timeout_timer_;
  Sender<StarterRpc> cmd_sender_;
  // Map of fetchers by node name.
  std::map<std::string, Fetcher<Status>> status_fetchers_;

  // Mapping of node name to a list of applications with pending commands.
  std::map<std::string, std::vector<CommandStatus>> current_commands_;

  std::function<void()> timeout_handler_;
  std::function<void()> success_handler_;
};

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
                         std::chrono::milliseconds timeout,
                         std::vector<const aos::Node *> nodes = {});

// Sends lots of commands and waits for them all to succeed.  There must not be
// more than 1 conflicting command in here which modifies the state of a single
// application otherwise it will never succeed.  An example is having both a
// start and stop command for a single application.
bool SendCommandBlocking(const std::vector<ApplicationCommand> &commands,
                         const aos::Configuration *config,
                         std::chrono::milliseconds timeout);

// Fetches the status of the application with the given name. Creates a
// temporary event loop from the provided config for fetching. Returns nullopt
// if the application is not found.
const std::optional<
    aos::FlatbufferDetachedBuffer<aos::starter::ApplicationStatus>>
GetStatus(std::string_view name, const aos::Configuration *config,
          const aos::Node *node);

// Fetches the entire status message of starter. Creates a temporary event loop
// from the provided config for fetching.
// The returned pair is the time at which the Status was sent on the node it was
// sent from, to allow calculating uptimes on remote nodes.
// TODO(james): Use the ServerStatistics message and return the monotonic offset
// instead, so that we can correctly handle high message latencies. Because
// people don't generally care about ultra-high-precision uptime calculations,
// this hasn't been prioritized.
std::optional<std::pair<aos::monotonic_clock::time_point,
                        const aos::FlatbufferVector<aos::starter::Status>>>
GetStarterStatus(const aos::Configuration *config, const aos::Node *node);

}  // namespace starter
}  // namespace aos

#endif  // AOS_STARTER_STARTER_RPC_LIB_H_

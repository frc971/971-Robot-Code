#ifndef AOS_EVENTS_SIMULATED_NETWORK_BRIDGE_H_
#define AOS_EVENTS_SIMULATED_NETWORK_BRIDGE_H_

#include "aos/events/event_loop.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/network/message_bridge_client_status.h"
#include "aos/network/message_bridge_server_status.h"
#include "aos/network/remote_message_generated.h"

namespace aos {
namespace message_bridge {

class RawMessageDelayer;

// This class moves messages between nodes.  It is implemented as a separate
// class because it would have been even harder to manage forwarding in the
// SimulatedEventLoopFactory.
class SimulatedMessageBridge {
 public:
  // Constructs the bridge.
  SimulatedMessageBridge(
      SimulatedEventLoopFactory *simulated_event_loop_factory);
  ~SimulatedMessageBridge();

  // Disables forwarding for this channel.  This should be used very rarely only
  // for things like the logger.
  void DisableForwarding(const Channel *channel);

  void Disconnect(const Node *source, const Node *other);
  void Connect(const Node *source, const Node *other);
  void SetState(const Node *source, const Node *other,
                message_bridge::State state);

  // Disables generating and sending the messages which message_gateway sends.
  // The messages are the ClientStatistics, ServerStatistics and Timestamp
  // messages.
  void DisableStatistics();

  // Calls SkipTimingReport() on all EventLoops used in the implementation. This
  // may improve the performance of long-simulated-duration tests.
  void SkipTimingReport();

 private:
  struct State {
    State(std::unique_ptr<aos::EventLoop> &&new_event_loop);

    State(const State &state) = delete;

    std::unique_ptr<aos::EventLoop> event_loop;
    MessageBridgeServerStatus server_status;
    MessageBridgeClientStatus client_status;

    std::vector<aos::Sender<RemoteMessage>> timestamp_loggers;
  };
  // Map of nodes to event loops.  This is a member variable so that the
  // lifetime of the event loops matches the lifetime of the bridge.
  std::map<const Node *, State> event_loop_map_;

  // List of delayers used to resend the messages.
  using DelayersVector = std::vector<std::unique_ptr<RawMessageDelayer>>;
  std::vector<std::unique_ptr<DelayersVector>> delayers_list_;
};

}  // namespace message_bridge
}  // namespace aos

#endif  // AOS_EVENTS_SIMULATED_NETWORK_BRIDGE_H_

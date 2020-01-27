#ifndef AOS_EVENTS_SIMULATED_NETWORK_BRIDGE_H_
#define AOS_EVENTS_SIMULATED_NETWORK_BRIDGE_H_

#include "aos/events/event_loop.h"
#include "aos/events/simulated_event_loop.h"

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

 private:
  // Map of nodes to event loops.  This is a member variable so that the
  // lifetime of the event loops matches the lifetime of the bridge.
  std::map<const Node *, std::unique_ptr<aos::EventLoop>> event_loop_map_;

  // List of delayers used to resend the messages.
  using DelayersVector = std::vector<std::unique_ptr<RawMessageDelayer>>;
  std::vector<std::unique_ptr<DelayersVector>> delayers_list_;
};

}  // namespace message_bridge
}  // namespace aos

#endif  // AOS_EVENTS_SIMULATED_NETWORK_BRIDGE_H_

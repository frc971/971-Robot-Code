#ifndef AOS_EVENTS_SIMULATED_NETWORK_BRIDGE_H_
#define AOS_EVENTS_SIMULATED_NETWORK_BRIDGE_H_

#include "aos/events/event_loop.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/network/message_bridge_client_status.h"
#include "aos/network/message_bridge_server_status.h"
#include "aos/network/remote_message_generated.h"
#include "aos/network/timestamp_channel.h"

namespace aos::message_bridge {

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
  enum class DestroySenders { kNo, kYes };
  void DisableStatistics(DestroySenders destroy_senders = DestroySenders::kNo);
  void DisableStatistics(const Node *node,
                         DestroySenders destroy_senders = DestroySenders::kNo);
  void EnableStatistics();
  void EnableStatistics(const Node *node);

 private:
  struct DelayersVector {
    std::vector<std::unique_ptr<RawMessageDelayer>> v;

    bool disable_forwarding = false;
  };
  struct State {
    State(NodeEventLoopFactory *node_factory) : node_factory_(node_factory) {
      const size_t num_nodes = node_factory->configuration()->nodes()->size();
      boot_uuids_.resize(num_nodes, UUID::Zero());
      client_state_.resize(num_nodes, message_bridge::State::CONNECTED);
      server_state_.resize(num_nodes, message_bridge::State::CONNECTED);
    }
    State(const State &state) = delete;

    void DisableStatistics(DestroySenders destroy_senders) {
      disable_statistics_ = true;
      destroy_senders_ = destroy_senders;
      if (server_status_) {
        server_status_->DisableStatistics(destroy_senders ==
                                          DestroySenders::kYes);
      }
      if (client_status) {
        client_status->DisableStatistics(destroy_senders ==
                                         DestroySenders::kYes);
      }
    }

    void EnableStatistics() {
      disable_statistics_ = false;
      if (server_status_) {
        server_status_->EnableStatistics();
      }
      if (client_status) {
        client_status->EnableStatistics();
      }
    }

    void AddSourceDelayer(RawMessageDelayer *delayer) {
      source_delayers_.emplace_back(delayer);
    }
    void AddDestinationDelayer(RawMessageDelayer *delayer) {
      destination_delayers_.emplace_back(delayer);
    }

    void MakeEventLoop();

    void SetEventLoop(std::unique_ptr<aos::EventLoop> loop);

    void SetSendData(
        std::function<void(uint32_t, monotonic_clock::time_point)> fn);

    void AddDelayerWatcher(const Channel *channel, DelayersVector *v) {
      delayer_watchers_.emplace_back(channel, v);
    }

    void SetBootUUID(size_t node_index, const UUID &boot_uuid);

    void SetServerState(const Node *destination, message_bridge::State state);

    void SetClientState(const Node *source, message_bridge::State state);

    std::vector<UUID> boot_uuids_;
    std::vector<message_bridge::State> client_state_;
    std::vector<message_bridge::State> server_state_;

    std::vector<std::pair<const Channel *, DelayersVector *>> delayer_watchers_;

    std::function<void(uint32_t, monotonic_clock::time_point)> fn_;

    NodeEventLoopFactory *node_factory_;
    std::unique_ptr<aos::EventLoop> event_loop;
    ChannelTimestampSender timestamp_loggers;
    std::unique_ptr<MessageBridgeServerStatus> server_status_;
    std::unique_ptr<MessageBridgeClientStatus> client_status;

    // List of delayers to update whenever this node starts or stops.
    // Source delayers (which are the ones fetching).
    std::vector<RawMessageDelayer *> source_delayers_;
    // Destination delayers (which are the ones sending on the receiving nodes).
    std::vector<RawMessageDelayer *> destination_delayers_;

    bool disable_statistics_ = false;
    DestroySenders destroy_senders_ = DestroySenders::kNo;
  };

  // Map of nodes to event loops.  This is a member variable so that the
  // lifetime of the event loops matches the lifetime of the bridge.
  std::map<const Node *, State> event_loop_map_;

  // List of delayers used to resend the messages.
  std::vector<std::unique_ptr<DelayersVector>> delayers_list_;
};

}  // namespace aos::message_bridge

#endif  // AOS_EVENTS_SIMULATED_NETWORK_BRIDGE_H_

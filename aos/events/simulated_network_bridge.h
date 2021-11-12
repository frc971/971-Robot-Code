#ifndef AOS_EVENTS_SIMULATED_NETWORK_BRIDGE_H_
#define AOS_EVENTS_SIMULATED_NETWORK_BRIDGE_H_

#include "aos/events/event_loop.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/network/message_bridge_client_status.h"
#include "aos/network/message_bridge_server_status.h"
#include "aos/network/remote_message_generated.h"
#include "aos/network/timestamp_channel.h"

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
  void DisableStatistics(const Node *node);
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

    void DisableStatistics() {
      disable_statistics_ = true;
      if (server_status) {
        server_status->DisableStatistics();
      }
      if (client_status) {
        client_status->DisableStatistics();
      }
    }

    void EnableStatistics() {
      disable_statistics_ = false;
      if (server_status) {
        server_status->EnableStatistics();
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

    void MakeEventLoop() {
      SetEventLoop(node_factory_->MakeEventLoop("message_bridge"));
    }

    void ClearEventLoop() { SetEventLoop(nullptr); }
    void SetEventLoop(std::unique_ptr<aos::EventLoop> loop);

    void SetSendData(std::function<void(const Context &)> fn) {
      CHECK(!fn_);
      fn_ = std::move(fn);
      if (server_status) {
        server_status->set_send_data(fn_);
      }
    }

    void AddDelayerWatcher(const Channel *channel, DelayersVector *v) {
      delayer_watchers_.emplace_back(channel, v);
    }

    void SetBootUUID(size_t node_index, const UUID &boot_uuid) {
      boot_uuids_[node_index] = boot_uuid;
      const Node *node =
          node_factory_->configuration()->nodes()->Get(node_index);
      if (server_status) {
        ServerConnection *connection =
            server_status->FindServerConnection(node);
        if (connection) {
          connection->mutate_state(server_state_[node_index]);
          server_status->ResetFilter(node_index);
          server_status->SetBootUUID(node_index, boot_uuid);
        }
      }
      if (client_status) {
        const int client_index =
            client_status->FindClientIndex(node->name()->string_view());
        ClientConnection *client_connection =
            client_status->GetClientConnection(client_index);
        if (client_connection) {
          client_status->SampleReset(client_index);
          client_connection->mutate_state(client_state_[node_index]);
        }
      }
    }

    void SetServerState(const Node *destination, message_bridge::State state) {
      const size_t node_index = configuration::GetNodeIndex(
          node_factory_->configuration(), destination);
      server_state_[node_index] = state;
      if (server_status) {
        ServerConnection *connection =
            server_status->FindServerConnection(destination);
        if (connection == nullptr) return;

        connection->mutate_state(state);
      }
    }

    void SetClientState(const Node *source, message_bridge::State state) {
      const size_t node_index =
          configuration::GetNodeIndex(node_factory_->configuration(), source);
      client_state_[node_index] = state;
      if (client_status) {
        ClientConnection *connection =
            client_status->GetClientConnection(source);

        if (connection == nullptr) return;

        connection->mutate_state(state);
      }
    }

    std::vector<UUID> boot_uuids_;
    std::vector<message_bridge::State> client_state_;
    std::vector<message_bridge::State> server_state_;

    std::vector<std::pair<const Channel *, DelayersVector *>> delayer_watchers_;

    std::function<void(const Context &)> fn_;

    NodeEventLoopFactory *node_factory_;
    std::unique_ptr<aos::EventLoop> event_loop;
    ChannelTimestampSender timestamp_loggers;
    std::unique_ptr<MessageBridgeServerStatus> server_status;
    std::unique_ptr<MessageBridgeClientStatus> client_status;

    // List of delayers to update whenever this node starts or stops.
    // Source delayers (which are the ones fetching).
    std::vector<RawMessageDelayer *> source_delayers_;
    // Destination delayers (which are the ones sending on the receiving nodes).
    std::vector<RawMessageDelayer *> destination_delayers_;

    bool disable_statistics_ = false;
  };
  // Map of nodes to event loops.  This is a member variable so that the
  // lifetime of the event loops matches the lifetime of the bridge.
  std::map<const Node *, State> event_loop_map_;

  // List of delayers used to resend the messages.
  std::vector<std::unique_ptr<DelayersVector>> delayers_list_;
};

}  // namespace message_bridge
}  // namespace aos

#endif  // AOS_EVENTS_SIMULATED_NETWORK_BRIDGE_H_

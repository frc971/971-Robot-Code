#include "aos/network/message_bridge_server_lib.h"

#include "absl/types/span.h"
#include "aos/events/logging/logger.h"
#include "aos/events/logging/logger_generated.h"
#include "aos/events/shm_event_loop.h"
#include "aos/network/connect_generated.h"
#include "aos/network/message_bridge_protocol.h"
#include "aos/network/message_bridge_server_generated.h"
#include "aos/network/sctp_server.h"
#include "glog/logging.h"

namespace aos {
namespace message_bridge {
namespace {

namespace chrono = std::chrono;

// Builds up the "empty" server statistics message to be pointed to by all the
// connections, updated at runtime, and periodically sent.
FlatbufferDetachedBuffer<ServerStatistics> MakeServerStatistics(
    const std::vector<std::string_view> &source_node_names,
    const Configuration *configuration) {
  flatbuffers::FlatBufferBuilder fbb;

  std::vector<flatbuffers::Offset<ServerConnection>> connection_offsets;
  for (const std::string_view node_name : source_node_names) {
    flatbuffers::Offset<Node> node_offset =
        CopyFlatBuffer(configuration::GetNode(configuration, node_name), &fbb);
    ServerConnection::Builder connection_builder(fbb);
    connection_builder.add_node(node_offset);
    connection_builder.add_state(State::DISCONNECTED);
    connection_builder.add_dropped_packets(0);
    connection_builder.add_sent_packets(0);
    connection_offsets.emplace_back(connection_builder.Finish());
  }
  flatbuffers::Offset<
      flatbuffers::Vector<flatbuffers::Offset<ServerConnection>>>
      connections_offset = fbb.CreateVector(connection_offsets);

  ServerStatistics::Builder server_statistics_builder(fbb);
  server_statistics_builder.add_connections(connections_offset);
  fbb.Finish(server_statistics_builder.Finish());

  return fbb.Release();
}

// Finds the statistics for the provided node name.
ServerConnection *FindServerConnection(ServerStatistics *statistics,
                                       std::string_view node_name) {
  ServerConnection *matching_server_connection = nullptr;
  for (size_t i = 0; i < statistics->mutable_connections()->size(); ++i) {
    ServerConnection *server_connection =
        statistics->mutable_connections()->GetMutableObject(i);
    if (server_connection->node()->name()->string_view() == node_name) {
      matching_server_connection = server_connection;
      break;
    }
  }

  CHECK(matching_server_connection != nullptr) << ": Unknown client";

  return matching_server_connection;
}

}  // namespace

bool ChannelState::Matches(const Channel *other_channel) {
  // Confirm the normal tuple, plus make sure that the other side isn't going to
  // send more data over than we expect with a mismatching size.
  return (
      channel_->name()->string_view() == other_channel->name()->string_view() &&
      channel_->type()->string_view() == other_channel->type()->string_view() &&
      channel_->max_size() == other_channel->max_size());
}

void ChannelState::SendData(SctpServer *server, const Context &context) {
  // TODO(austin): I don't like allocating this buffer when we are just freeing
  // it at the end of the function.
  flatbuffers::FlatBufferBuilder fbb(channel_->max_size() + 100);
  VLOG(1) << "Found " << peers_.size() << " peers on channel "
          << channel_->name()->string_view() << " size " << context.size;

  // TODO(austin): Use an iovec to build it up in 3 parts to avoid the copy?
  // Only useful when not logging.
  fbb.FinishSizePrefixed(logger::PackMessage(&fbb, context, channel_index_,
                                             logger::LogType::kLogMessage));

  // TODO(austin): Track which connections need to be reliable and handle
  // resending properly.
  size_t sent_count = 0;
  bool logged_remotely = false;
  for (Peer &peer : peers_) {
    logged_remotely = logged_remotely || peer.logged_remotely;

    if (peer.sac_assoc_id != 0) {
      server->Send(std::string_view(
                       reinterpret_cast<const char *>(fbb.GetBufferPointer()),
                       fbb.GetSize()),
                   peer.sac_assoc_id, peer.stream,
                   peer.connection->time_to_live() / 1000000);
      peer.server_connection_statistics->mutate_sent_packets(
          peer.server_connection_statistics->sent_packets() + 1);
      if (peer.logged_remotely) {
        ++sent_count;
      }
    } else {
      peer.server_connection_statistics->mutate_dropped_packets(
          peer.server_connection_statistics->dropped_packets() + 1);
    }
  }

  if (logged_remotely) {
    if (sent_count == 0) {
      VLOG(1) << "No clients, rejecting";
      HandleFailure(fbb.Release());
    } else {
      sent_messages_.emplace_back(fbb.Release());
    }
  } else {
    VLOG(1) << "Not bothering to track this message since nobody cares.";
  }

  // TODO(austin): Limit the size of this queue.  Flush messages to disk
  // which are too old.  We really care about messages which didn't make it
  // to a logger...  Which is a new concept or two.

  // Need to handle logging and disk in another thread.  Need other thread
  // since we sometimes want to skip disk, and synchronization is easier.
  // This thread then spins on the queue until empty, then polls at 1-10 hz.

  // TODO(austin): ~10 MB chunks on disk and push them over the logging
  // channel?  Threadsafe disk backed queue object which can handle restarts
  // and flushes.  Whee.
}

void ChannelState::HandleDelivery(sctp_assoc_t /*rcv_assoc_id*/,
                                  uint16_t /*ssn*/,
                                  absl::Span<const uint8_t> data) {
  const logger::MessageHeader *message_header =
      flatbuffers::GetRoot<logger::MessageHeader>(data.data());
  while (sent_messages_.size() > 0u) {
    if (sent_messages_.begin()->message().monotonic_sent_time() ==
        message_header->monotonic_sent_time()) {
      sent_messages_.pop_front();
      continue;
    }

    if (sent_messages_.begin()->message().monotonic_sent_time() <
        message_header->monotonic_sent_time()) {
      VLOG(1) << "Delivery looks wrong, rejecting";
      HandleFailure(std::move(sent_messages_.front()));
      sent_messages_.pop_front();
      continue;
    }

    break;
  }
}

void ChannelState::HandleFailure(
    SizePrefixedFlatbufferDetachedBuffer<logger::MessageHeader> &&message) {
  // TODO(austin): Put it in the log queue.
  LOG(INFO) << "Failed to send " << FlatbufferToJson(message);

  // Note: this may be really out of order when we avoid the queue...  We
  // have the ones we know didn't make it immediately, and the ones which
  // time out eventually.  Need to sort that out.
}

void ChannelState::AddPeer(const Connection *connection,
                           ServerConnection *server_connection_statistics,
                           bool logged_remotely) {
  peers_.emplace_back(0, 0, connection, server_connection_statistics,
                      logged_remotely);
}

void ChannelState::NodeDisconnected(sctp_assoc_t assoc_id) {
  for (ChannelState::Peer &peer : peers_) {
    if (peer.sac_assoc_id == assoc_id) {
      // TODO(austin): This will not handle multiple clients from
      // a single node.  But that should be rare.
      peer.server_connection_statistics->mutate_state(State::DISCONNECTED);
      peer.sac_assoc_id = 0;
      peer.stream = 0;
      break;
    }
  }
}

void ChannelState::NodeConnected(const Node *node, sctp_assoc_t assoc_id,
                                 int stream, SctpServer *server) {
  for (ChannelState::Peer &peer : peers_) {
    if (peer.connection->name()->string_view() == node->name()->string_view()) {
      peer.sac_assoc_id = assoc_id;
      peer.stream = stream;
      peer.server_connection_statistics->mutate_state(State::CONNECTED);
      server->SetStreamPriority(assoc_id, stream, peer.connection->priority());

      break;
    }
  }
}

MessageBridgeServer::MessageBridgeServer(aos::ShmEventLoop *event_loop)
    : event_loop_(event_loop),
      sender_(event_loop_->MakeSender<ServerStatistics>("/aos")),
      statistics_(MakeServerStatistics(
          configuration::DestinationNodeNames(event_loop->configuration(),
                                              event_loop->node()),
          event_loop->configuration())),
      server_("::", event_loop->node()->port()) {
  CHECK(event_loop_->node() != nullptr) << ": No nodes configured.";

  // TODO(austin): Time sync.  sctp gives us filtered round trip time, not
  // target time.

  // TODO(austin): Logging synchronization.
  //
  // TODO(austin): How do we handle parameter channels?  The oldest value
  // needs to be sent regardless on connection (though probably only if it has
  // changed).
  event_loop_->epoll()->OnReadable(server_.fd(),
                                   [this]() { MessageReceived(); });

  LOG(INFO) << "Hostname: " << event_loop_->node()->hostname()->string_view();

  int channel_index = 0;
  for (const Channel *channel : *event_loop_->configuration()->channels()) {
    CHECK(channel->has_source_node());
    if (channel->source_node()->string_view() ==
            event_loop_->node()->name()->string_view() &&
        channel->has_destination_nodes()) {
      std::unique_ptr<ChannelState> state(
          new ChannelState{channel, channel_index});

      for (const Connection *connection : *channel->destination_nodes()) {
        const Node *other_node = configuration::GetNode(
            event_loop_->configuration(), connection->name()->string_view());
        state->AddPeer(
            connection,
            FindServerConnection(statistics_.mutable_message(),
                                 connection->name()->string_view()),
            configuration::ChannelMessageIsLoggedOnNode(channel, other_node));
      }

      // Call SendData for every message.
      ChannelState *state_ptr = state.get();
      event_loop_->MakeRawWatcher(
          channel,
          [this, state_ptr](const Context &context, const void * /*message*/) {
            state_ptr->SendData(&server_, context);
          });
      channels_.emplace_back(std::move(state));
    } else {
      channels_.emplace_back(nullptr);
    }
    ++channel_index;
  }

  statistics_timer_ = event_loop_->AddTimer([this]() { SendStatistics(); });
  event_loop_->OnRun([this]() {
    statistics_timer_->Setup(event_loop_->monotonic_now() + chrono::seconds(1),
                             chrono::seconds(1));
  });
}

void MessageBridgeServer::NodeConnected(sctp_assoc_t assoc_id) {
  server_.SetPriorityScheduler(assoc_id);
}

void MessageBridgeServer::NodeDisconnected(sctp_assoc_t assoc_id) {
  // Find any matching peers and remove them.
  for (std::unique_ptr<ChannelState> &channel_state : channels_) {
    if (channel_state.get() == nullptr) {
      continue;
    }

    channel_state->NodeDisconnected(assoc_id);
  }
}

void MessageBridgeServer::MessageReceived() {
  aos::unique_c_ptr<Message> message = server_.Read();

  if (message->message_type == Message::kNotification) {
    const union sctp_notification *snp =
        (const union sctp_notification *)message->data();

    switch (snp->sn_header.sn_type) {
      case SCTP_ASSOC_CHANGE: {
        const struct sctp_assoc_change *sac = &snp->sn_assoc_change;
        switch (sac->sac_state) {
          case SCTP_COMM_UP:
            NodeConnected(sac->sac_assoc_id);
            VLOG(1) << "Peer connected";
            break;
          case SCTP_COMM_LOST:
          case SCTP_SHUTDOWN_COMP:
          case SCTP_CANT_STR_ASSOC:
            NodeDisconnected(sac->sac_assoc_id);
            VLOG(1) << "Disconnect";
            break;
          case SCTP_RESTART:
            LOG(FATAL) << "Never seen this before.";
            break;
        }
      } break;
    }

    if (VLOG_IS_ON(1)) {
      PrintNotification(message.get());
    }
  } else if (message->message_type == Message::kMessage) {
    HandleData(message.get());
  }
}

void MessageBridgeServer::HandleData(const Message *message) {
  VLOG(1) << "Received data of length " << message->size;

  if (message->header.rcvinfo.rcv_sid == kConnectStream()) {
    // Control channel!
    const Connect *connect = flatbuffers::GetRoot<Connect>(message->data());
    VLOG(1) << FlatbufferToJson(connect);

    // Account for the control channel and delivery times channel.
    size_t channel_index = kControlStreams();
    for (const Channel *channel : *connect->channels_to_transfer()) {
      bool matched = false;
      for (std::unique_ptr<ChannelState> &channel_state : channels_) {
        if (channel_state.get() == nullptr) {
          continue;
        }
        if (channel_state->Matches(channel)) {
          channel_state->NodeConnected(connect->node(),
                                       message->header.rcvinfo.rcv_assoc_id,
                                       channel_index, &server_);

          matched = true;
          break;
        }
      }
      if (!matched) {
        LOG(ERROR) << "Remote tried registering for unknown channel "
                   << FlatbufferToJson(channel);
      } else {
        ++channel_index;
      }
    }
  } else if (message->header.rcvinfo.rcv_sid == kTimestampStream()) {
    // Message delivery
    const logger::MessageHeader *message_header =
        flatbuffers::GetRoot<logger::MessageHeader>(message->data());

    channels_[message_header->channel_index()]->HandleDelivery(
        message->header.rcvinfo.rcv_assoc_id, message->header.rcvinfo.rcv_ssn,
        absl::Span<const uint8_t>(message->data(), message->size));
  }

  if (VLOG_IS_ON(1)) {
    message->LogRcvInfo();
  }
}

}  // namespace message_bridge
}  // namespace aos

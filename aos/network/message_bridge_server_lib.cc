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
  fbb.ForceDefaults(true);

  std::vector<flatbuffers::Offset<ServerConnection>> connection_offsets;
  for (const std::string_view node_name : source_node_names) {
    flatbuffers::Offset<Node> node_offset =
        CopyFlatBuffer(configuration::GetNode(configuration, node_name), &fbb);
    ServerConnection::Builder connection_builder(fbb);
    connection_builder.add_node(node_offset);
    connection_builder.add_state(State::DISCONNECTED);
    connection_builder.add_dropped_packets(0);
    connection_builder.add_sent_packets(0);
    connection_builder.add_monotonic_offset(0);
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
  fbb.ForceDefaults(true);
  VLOG(1) << "Found " << peers_.size() << " peers on channel "
          << channel_->name()->string_view() << " "
          << channel_->type()->string_view() << " size " << context.size;

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

    if (peer.sac_assoc_id != 0 &&
        server->Send(std::string_view(
                         reinterpret_cast<const char *>(fbb.GetBufferPointer()),
                         fbb.GetSize()),
                     peer.sac_assoc_id, peer.stream,
                     peer.connection->time_to_live() / 1000000)) {
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
  if (VLOG_IS_ON(2)) {
    LOG(INFO) << "Failed to send " << FlatbufferToJson(message);
  } else if (VLOG_IS_ON(1)) {
    message.mutable_message()->clear_data();
    LOG(INFO) << "Failed to send " << FlatbufferToJson(message);
  }

  // Note: this may be really out of order when we avoid the queue...  We
  // have the ones we know didn't make it immediately, and the ones which
  // time out eventually.  Need to sort that out.
}

void ChannelState::AddPeer(const Connection *connection, int node_index,
                           ServerConnection *server_connection_statistics,
                           bool logged_remotely) {
  peers_.emplace_back(connection, node_index, server_connection_statistics,
                      logged_remotely);
}

int ChannelState::NodeDisconnected(sctp_assoc_t assoc_id) {
  for (ChannelState::Peer &peer : peers_) {
    if (peer.sac_assoc_id == assoc_id) {
      // TODO(austin): This will not handle multiple clients from
      // a single node.  But that should be rare.
      peer.server_connection_statistics->mutate_state(State::DISCONNECTED);
      peer.sac_assoc_id = 0;
      peer.stream = 0;
      return peer.node_index;
    }
  }
  return -1;
}

int ChannelState::NodeConnected(const Node *node, sctp_assoc_t assoc_id,
                                int stream, SctpServer *server) {
  for (ChannelState::Peer &peer : peers_) {
    if (peer.connection->name()->string_view() == node->name()->string_view()) {
      peer.sac_assoc_id = assoc_id;
      peer.stream = stream;
      peer.server_connection_statistics->mutate_state(State::CONNECTED);
      server->SetStreamPriority(assoc_id, stream, peer.connection->priority());
      return peer.node_index;
    }
  }
  return -1;
}

MessageBridgeServer::MessageBridgeServer(aos::ShmEventLoop *event_loop)
    : event_loop_(event_loop),
      sender_(event_loop_->MakeSender<ServerStatistics>("/aos")),
      statistics_(MakeServerStatistics(
          configuration::DestinationNodeNames(event_loop->configuration(),
                                              event_loop->node()),
          event_loop->configuration())),
      timestamp_sender_(event_loop_->MakeSender<Timestamp>("/aos")),
      client_statistics_fetcher_(
          event_loop_->MakeFetcher<ClientStatistics>("/aos")),
      server_("::", event_loop->node()->port()) {
  CHECK(event_loop_->node() != nullptr) << ": No nodes configured.";

  server_connection_offsets_.reserve(
      statistics_.message().connections()->size());

  int32_t max_size = 0;
  timestamp_fetchers_.resize(event_loop->configuration()->nodes()->size());
  filters_.resize(event_loop->configuration()->nodes()->size());
  server_connection_.resize(event_loop->configuration()->nodes()->size());

  // Seed up all the per-node connection state.
  // We are making the assumption here that every connection is bidirectional
  // (data is being sent both ways).  This is pretty safe because we are
  // forwarding timestamps between nodes.
  for (std::string_view destination_node_name :
       configuration::DestinationNodeNames(event_loop->configuration(),
                                           event_loop->node())) {
    // Find the largest connection message so we can size our buffers big enough
    // to receive a connection message.  The connect message comes from the
    // client to the server, so swap the node arguments.
    const int32_t connect_size = static_cast<int32_t>(
        MakeConnectMessage(event_loop->configuration(),
                           configuration::GetNode(event_loop->configuration(),
                                                  destination_node_name),
                           event_loop->node()->name()->string_view())
            .size());
    VLOG(1) << "Connection to " << destination_node_name << " has size "
            << connect_size;
    max_size = std::max(max_size, connect_size);
    const Node *destination_node = configuration::GetNode(
        event_loop->configuration(), destination_node_name);

    const int node_index = configuration::GetNodeIndex(
        event_loop->configuration(), destination_node);

    // Now find the timestamp channel forwarded from the other node.
    const Channel *const other_timestamp_channel =
        configuration::GetChannel(event_loop_->configuration(), "/aos",
                                  Timestamp::GetFullyQualifiedName(),
                                  event_loop_->name(), destination_node);

    timestamp_fetchers_[node_index] = event_loop_->MakeFetcher<Timestamp>(
        other_timestamp_channel->name()->string_view());

    // And then find the server connection that we should be populating
    // statistics into.
    server_connection_[node_index] = FindServerConnection(
        statistics_.mutable_message(), destination_node->name()->string_view());
  }

  // TODO(austin): Logging synchronization.
  //
  // TODO(austin): How do we handle parameter channels?  The oldest value
  // needs to be sent regardless on connection (though probably only if it has
  // changed).
  event_loop_->epoll()->OnReadable(server_.fd(),
                                   [this]() { MessageReceived(); });

  LOG(INFO) << "Hostname: " << event_loop_->node()->hostname()->string_view();

  int channel_index = 0;
  const Channel *const timestamp_channel = configuration::GetChannel(
      event_loop_->configuration(), "/aos", Timestamp::GetFullyQualifiedName(),
      event_loop_->name(), event_loop_->node());

  for (const Channel *channel : *event_loop_->configuration()->channels()) {
    CHECK(channel->has_source_node());

    if (configuration::ChannelIsSendableOnNode(channel, event_loop_->node()) &&
        channel->has_destination_nodes()) {
      max_size = std::max(channel->max_size(), max_size);
      std::unique_ptr<ChannelState> state(
          new ChannelState{channel, channel_index});

      for (const Connection *connection : *channel->destination_nodes()) {
        const Node *other_node = configuration::GetNode(
            event_loop_->configuration(), connection->name()->string_view());
        state->AddPeer(
            connection,
            configuration::GetNodeIndex(event_loop_->configuration(),
                                        connection->name()->string_view()),
            FindServerConnection(statistics_.mutable_message(),
                                 connection->name()->string_view()),
            configuration::ChannelMessageIsLoggedOnNode(channel, other_node));
      }

      // Don't subscribe to timestamps on the timestamp channel.  Those get
      // handled specially.
      if (channel != timestamp_channel) {
        // Call SendData for every message.
        ChannelState *state_ptr = state.get();
        event_loop_->MakeRawWatcher(
            channel, [this, state_ptr](const Context &context,
                                       const void * /*message*/) {
              state_ptr->SendData(&server_, context);
            });
      } else {
        CHECK(timestamp_state_ == nullptr);
        timestamp_state_ = state.get();
      }
      channels_.emplace_back(std::move(state));
    } else {
      channels_.emplace_back(nullptr);
    }
    ++channel_index;
  }

  // Buffer up the max size a bit so everything fits nicely.
  LOG(INFO) << "Max message size for all clients is " << max_size;
  server_.SetMaxSize(max_size + 100);

  statistics_timer_ = event_loop_->AddTimer([this]() { Tick(); });
  event_loop_->OnRun([this]() {
    statistics_timer_->Setup(event_loop_->monotonic_now() + kPingPeriod,
                             kPingPeriod);
  });
}

void MessageBridgeServer::NodeConnected(sctp_assoc_t assoc_id) {
  server_.SetPriorityScheduler(assoc_id);
}

void MessageBridgeServer::NodeDisconnected(sctp_assoc_t assoc_id) {
  // Find any matching peers and remove them.
  int node_index = -1;
  for (std::unique_ptr<ChannelState> &channel_state : channels_) {
    if (channel_state.get() == nullptr) {
      continue;
    }

    node_index = channel_state->NodeDisconnected(assoc_id);
    CHECK_NE(node_index, -1);
  }

  if (node_index != -1) {
    VLOG(1) << "Resetting filters for " << node_index << " "
            << event_loop_->configuration()
                   ->nodes()
                   ->Get(node_index)
                   ->name()
                   ->string_view();
    ResetFilter(node_index);
  }
}

void MessageBridgeServer::ResetFilter(int node_index) {
  filters_[node_index].Reset();
  server_connection_[node_index]->mutate_monotonic_offset(0);
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

    if (VLOG_IS_ON(2)) {
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
    int node_index = -1;
    for (const Channel *channel : *connect->channels_to_transfer()) {
      bool matched = false;
      for (std::unique_ptr<ChannelState> &channel_state : channels_) {
        if (channel_state.get() == nullptr) {
          continue;
        }
        if (channel_state->Matches(channel)) {
          node_index = channel_state->NodeConnected(
              connect->node(), message->header.rcvinfo.rcv_assoc_id,
              channel_index, &server_);
          CHECK_NE(node_index, -1);

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
    ResetFilter(node_index);
    VLOG(1) << "Resetting filters for " << node_index << " "
            << event_loop_->configuration()
                   ->nodes()
                   ->Get(node_index)
                   ->name()
                   ->string_view();
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

void MessageBridgeServer::SendStatistics() {
  aos::Sender<ServerStatistics>::Builder builder = sender_.MakeBuilder();

  server_connection_offsets_.clear();

  // Copy the statistics over, but only add monotonic_offset if it is valid.
  for (const ServerConnection *connection :
       *statistics_.message().connections()) {
    flatbuffers::Offset<flatbuffers::String> node_name_offset =
        builder.fbb()->CreateString(connection->node()->name()->string_view());
    Node::Builder node_builder = builder.MakeBuilder<Node>();
    node_builder.add_name(node_name_offset);
    flatbuffers::Offset<Node> node_offset = node_builder.Finish();

    ServerConnection::Builder server_connection_builder =
        builder.MakeBuilder<ServerConnection>();
    server_connection_builder.add_node(node_offset);
    server_connection_builder.add_state(connection->state());
    server_connection_builder.add_dropped_packets(
        connection->dropped_packets());
    server_connection_builder.add_sent_packets(connection->sent_packets());

    // TODO(austin): If it gets stale, drop it too.
    if (connection->monotonic_offset() != 0) {
      server_connection_builder.add_monotonic_offset(
          connection->monotonic_offset());
    }

    server_connection_offsets_.emplace_back(server_connection_builder.Finish());
  }

  flatbuffers::Offset<
      flatbuffers::Vector<flatbuffers::Offset<ServerConnection>>>
      server_connections_offset =
          builder.fbb()->CreateVector(server_connection_offsets_);

  ServerStatistics::Builder server_statistics_builder =
      builder.MakeBuilder<ServerStatistics>();
  server_statistics_builder.add_connections(server_connections_offset);
  builder.Send(server_statistics_builder.Finish());
}

void MessageBridgeServer::Tick() {
  // Send statistics every kStatisticsPeriod. Use the context so we don't get
  // caught up with the wakeup delay and jitter.
  if (event_loop_->context().monotonic_event_time >=
      last_statistics_send_time_ + kStatisticsPeriod) {
    SendStatistics();
    last_statistics_send_time_ = event_loop_->context().monotonic_event_time;
  }

  // The message_bridge_client application measures and filters the offsets from
  // all messages it receives.  It then sends this on in the ClientStatistics
  // message.  Collect that up and forward it back over the Timestamp message so
  // we have guarenteed traffic on the other node for timestamping.  This also
  // moves the offsets back across the network so both directions can be
  // observed.
  client_statistics_fetcher_.Fetch();

  // Build up the timestamp message.  Do it here so that we don't have invalid
  // data in it.
  FlatbufferFixedAllocatorArray<Timestamp, 1000> timestamp_copy;
  flatbuffers::FlatBufferBuilder *fbb = timestamp_copy.Builder();

  if (client_statistics_fetcher_.get()) {
    // Build up the list of client offsets.
    std::vector<flatbuffers::Offset<ClientOffset>> client_offsets;

    // Iterate through the connections this node has made.
    for (const ClientConnection *connection :
         *client_statistics_fetcher_->connections()) {
      const int node_index = configuration::GetNodeIndex(
          event_loop_->configuration(),
          connection->node()->name()->string_view());

      // Filter out the ones which aren't connected.
      // And the ones without monotonic offsets.
      if (connection->state() != State::CONNECTED ||
          !connection->has_monotonic_offset() ||
          client_statistics_fetcher_.context().monotonic_event_time +
                  kClientStatisticsStaleTimeout <
              event_loop_->context().monotonic_event_time) {
        VLOG(1) << "Disconnected, no offset, or client message too old for "
                << connection->node()->name()->string_view();
        ResetFilter(node_index);
        continue;
      }

      timestamp_fetchers_[node_index].Fetch();

      // Find the offset computed on their node for this client connection
      // using their timestamp message.
      bool has_their_offset = false;
      std::chrono::nanoseconds their_offset = std::chrono::nanoseconds(0);
      if (timestamp_fetchers_[node_index].get() != nullptr) {
        for (const ClientOffset *client_offset :
             *timestamp_fetchers_[node_index]->offsets()) {
          if (client_offset->node()->name()->string_view() ==
              event_loop_->node()->name()->string_view()) {
            // Make sure it has an offset and the message isn't stale.
            if (client_offset->has_monotonic_offset()) {
              if (timestamp_fetchers_[node_index]
                          .context()
                          .monotonic_event_time +
                      kTimestampStaleTimeout >
                  event_loop_->context().monotonic_event_time) {
                their_offset =
                    std::chrono::nanoseconds(client_offset->monotonic_offset());
                has_their_offset = true;
              } else {
                ResetFilter(node_index);
                VLOG(1) << "Timestamp old, resetting.";
              }
            }
            break;
          }
        }
      }

      if (has_their_offset &&
          server_connection_[node_index]->state() == State::CONNECTED) {
        // Update the filters.
        if (filters_[node_index].MissingSamples()) {
          // Update the offset the first time.  This should be representative.
          filters_[node_index].set_base_offset(
              std::chrono::nanoseconds(connection->monotonic_offset()));
        }
        // The message_bridge_clients are the ones running the first filter.  So
        // set the values from that and let the averaging filter run from there.
        filters_[node_index].FwdSet(
            timestamp_fetchers_[node_index].context().monotonic_remote_time,
            std::chrono::nanoseconds(connection->monotonic_offset()));
        filters_[node_index].RevSet(
            client_statistics_fetcher_.context().monotonic_event_time,
            their_offset);

        // Publish!
        server_connection_[node_index]->mutate_monotonic_offset(
            -filters_[node_index].offset().count());
      }

      // Now fill out the Timestamp message with the offset from the client.
      flatbuffers::Offset<flatbuffers::String> node_name_offset =
          fbb->CreateString(connection->node()->name()->string_view());

      Node::Builder node_builder(*fbb);
      node_builder.add_name(node_name_offset);
      flatbuffers::Offset<Node> node_offset = node_builder.Finish();

      ClientOffset::Builder client_offset_builder(*fbb);
      client_offset_builder.add_node(node_offset);
      client_offset_builder.add_monotonic_offset(
          connection->monotonic_offset());
      client_offsets.emplace_back(client_offset_builder.Finish());
    }
    flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<ClientOffset>>>
        offsets_offset = fbb->CreateVector(client_offsets);

    Timestamp::Builder builder(*fbb);
    builder.add_offsets(offsets_offset);
    timestamp_copy.Finish(builder.Finish());
  } else {
    // Publish an empty timestamp if we have nothing.
    flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<ClientOffset>>>
        offsets_offset =
            fbb->CreateVector(std::vector<flatbuffers::Offset<ClientOffset>>{});
    Timestamp::Builder builder(*fbb);
    builder.add_offsets(offsets_offset);
    timestamp_copy.Finish(builder.Finish());
  }

  // Send it out over shm, and using that timestamp, then send it out over sctp.
  // This avoid some context switches.
  timestamp_sender_.Send(timestamp_copy);

  Context context;
  context.monotonic_event_time = timestamp_sender_.monotonic_sent_time();
  context.realtime_event_time = timestamp_sender_.realtime_sent_time();
  context.queue_index = timestamp_sender_.sent_queue_index();
  context.size = timestamp_copy.size();
  context.data = timestamp_copy.data();

  // Since we are building up the timestamp to send here, we need to trigger the
  // SendData call ourselves.
  timestamp_state_->SendData(&server_, context);
}

}  // namespace message_bridge
}  // namespace aos

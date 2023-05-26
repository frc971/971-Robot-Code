#include "aos/network/message_bridge_server_lib.h"

#include "absl/strings/str_cat.h"
#include "absl/types/span.h"
#include "glog/logging.h"
#include "glog/raw_logging.h"

#include "aos/events/logging/log_reader.h"
#include "aos/events/logging/logger_generated.h"
#include "aos/events/shm_event_loop.h"
#include "aos/network/connect_generated.h"
#include "aos/network/message_bridge_protocol.h"
#include "aos/network/message_bridge_server_generated.h"
#include "aos/network/remote_data_generated.h"
#include "aos/network/remote_message_generated.h"
#include "aos/network/sctp_server.h"
#include "aos/network/timestamp_channel.h"

namespace aos {
namespace message_bridge {
namespace chrono = std::chrono;

bool ChannelState::Matches(const Channel *other_channel) {
  return channel_->name()->string_view() ==
             other_channel->name()->string_view() &&
         channel_->type()->string_view() ==
             other_channel->type()->string_view();
}

flatbuffers::FlatBufferBuilder ChannelState::PackContext(
    FixedAllocator *allocator, const Context &context) {
  flatbuffers::FlatBufferBuilder fbb(
      channel_->max_size() + kHeaderSizeOverhead(), allocator);
  fbb.ForceDefaults(true);
  VLOG(2) << "Found " << peers_.size() << " peers on channel "
          << channel_->name()->string_view() << " "
          << channel_->type()->string_view() << " size " << context.size;

  flatbuffers::Offset<flatbuffers::Vector<uint8_t>> data_offset =
      fbb.CreateVector(static_cast<const uint8_t *>(context.data),
                       context.size);

  flatbuffers::Offset<flatbuffers::Vector<uint8_t>> boot_uuid_offset =
      context.source_boot_uuid.PackVector(&fbb);

  RemoteData::Builder remote_data_builder(fbb);
  remote_data_builder.add_channel_index(channel_index_);
  remote_data_builder.add_queue_index(context.queue_index);
  remote_data_builder.add_monotonic_sent_time(
      context.monotonic_event_time.time_since_epoch().count());
  remote_data_builder.add_realtime_sent_time(
      context.realtime_event_time.time_since_epoch().count());
  remote_data_builder.add_data(data_offset);
  remote_data_builder.add_boot_uuid(boot_uuid_offset);

  // TODO(austin): Use an iovec to build it up in 3 parts to avoid the copy?
  // Only useful when not logging.
  fbb.FinishSizePrefixed(remote_data_builder.Finish());

  return fbb;
}

void ChannelState::SendData(SctpServer *server, FixedAllocator *allocator,
                            const Context &context) {
  // TODO(austin): I don't like allocating this buffer when we are just freeing
  // it at the end of the function.
  flatbuffers::FlatBufferBuilder fbb = PackContext(allocator, context);

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
      VLOG(1)
          << "No clients, rejecting. TODO(austin): do backup logging to disk";
    } else {
      VLOG(2) << "TODO(austin): backup log to disk if this fails eventually";
    }
  } else {
    VLOG(2) << "Not bothering to track this message since nobody cares.";
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

void ChannelState::HandleDelivery(sctp_assoc_t rcv_assoc_id, uint16_t /*ssn*/,
                                  absl::Span<const uint8_t> data,
                                  uint32_t partial_deliveries,
                                  MessageBridgeServerStatus *server_status) {
  const logger::MessageHeader *message_header =
      flatbuffers::GetRoot<logger::MessageHeader>(data.data());
  for (Peer &peer : peers_) {
    if (peer.sac_assoc_id == rcv_assoc_id) {
      if (peer.timestamp_logger != nullptr) {
        // TODO(austin): Need to implement reliable sending of the delivery
        // timestamps.  Track what made it, and retry what didn't.
        //
        // This needs to be munged and cleaned up to match the timestamp
        // standard.

        aos::Sender<RemoteMessage>::Builder builder =
            peer.timestamp_logger->MakeBuilder();

        flatbuffers::Offset<flatbuffers::Vector<uint8_t>> boot_uuid_offset =
            server_status->BootUUID(peer.node_index)
                .value()
                .PackVector(builder.fbb());

        RemoteMessage::Builder remote_message_builder =
            builder.MakeBuilder<RemoteMessage>();

        remote_message_builder.add_channel_index(
            message_header->channel_index());

        remote_message_builder.add_queue_index(
            message_header->remote_queue_index());
        remote_message_builder.add_monotonic_sent_time(
            message_header->monotonic_remote_time());
        remote_message_builder.add_realtime_sent_time(
            message_header->realtime_remote_time());

        // Swap the remote and sent metrics.  They are from the sender's
        // perspective, not the receiver's perspective.
        remote_message_builder.add_monotonic_remote_time(
            message_header->monotonic_sent_time());
        remote_message_builder.add_realtime_remote_time(
            message_header->realtime_sent_time());
        remote_message_builder.add_remote_queue_index(
            message_header->queue_index());
        remote_message_builder.add_boot_uuid(boot_uuid_offset);

        server_status->AddPartialDeliveries(peer.node_index,
                                            partial_deliveries);

        builder.CheckOk(builder.Send(remote_message_builder.Finish()));
      }
      break;
    }
  }

  // TODO(austin): record success of preceding messages, and log to disk if we
  // don't find this in the list.
}

void ChannelState::AddPeer(const Connection *connection, int node_index,
                           ServerConnection *server_connection_statistics,
                           MessageBridgeServerStatus *server_status,
                           bool logged_remotely,
                           aos::Sender<RemoteMessage> *timestamp_logger) {
  peers_.emplace_back(connection, node_index, server_connection_statistics,
                      server_status, logged_remotely, timestamp_logger);
}

int ChannelState::NodeDisconnected(sctp_assoc_t assoc_id) {
  VLOG(1) << "Disconnected " << assoc_id;
  for (ChannelState::Peer &peer : peers_) {
    if (peer.sac_assoc_id == assoc_id) {
      // TODO(austin): This will not handle multiple clients from
      // a single node.  But that should be rare.
      peer.sac_assoc_id = 0;
      peer.stream = 0;
      return peer.node_index;
    }
  }
  return -1;
}

int ChannelState::NodeConnected(const Node *node, sctp_assoc_t assoc_id,
                                int stream, SctpServer *server,
                                FixedAllocator *allocator,
                                aos::monotonic_clock::time_point monotonic_now,
                                std::vector<sctp_assoc_t> *reconnected) {
  VLOG(1) << "Channel " << channel_->name()->string_view() << " "
          << channel_->type()->string_view() << " mapped to stream " << stream
          << " for node " << node->name()->string_view() << " assoc_id "
          << assoc_id;
  for (ChannelState::Peer &peer : peers_) {
    // The node name is the most reliable method of detecting the same peer
    // because in a multihomed system, the same IP address may not always be
    // used to connect.
    if (peer.connection->name()->string_view() == node->name()->string_view()) {
      // There's a peer already connected.  Disconnect them and take over.
      if (peer.sac_assoc_id != 0 &&
          (std::find(reconnected->begin(), reconnected->end(),
                     peer.sac_assoc_id) == reconnected->end())) {
        reconnected->push_back(peer.sac_assoc_id);
        if (peer.sac_assoc_id == assoc_id) {
          if (VLOG_IS_ON(1)) {
            LOG_EVERY_T(WARNING, 0.025)
                << "Node " << node->name()->string_view() << " reconnecting on "
                << assoc_id << " with the same ID, something got lost";
          }
        } else {
          if (VLOG_IS_ON(1)) {
            LOG_EVERY_T(WARNING, 0.025)
                << "Node " << node->name()->string_view() << " "
                << " already connected on " << peer.sac_assoc_id
                << " aborting old connection and switching to " << assoc_id;
          }
          server->Abort(peer.sac_assoc_id);
        }
      }

      peer.sac_assoc_id = assoc_id;
      peer.stream = stream;
      peer.server_status->Connect(peer.node_index, monotonic_now);

      server->SetStreamPriority(assoc_id, stream, peer.connection->priority());
      if (last_message_fetcher_ && peer.connection->time_to_live() == 0) {
        last_message_fetcher_->Fetch();
        VLOG(1) << "Got a connection on a reliable channel "
                << configuration::StrippedChannelToString(
                       last_message_fetcher_->channel())
                << ", sending? "
                << (last_message_fetcher_->context().data != nullptr);
        if (last_message_fetcher_->context().data != nullptr) {
          // SendData sends to all...  Only send to the new one.
          flatbuffers::FlatBufferBuilder fbb =
              PackContext(allocator, last_message_fetcher_->context());

          if (server->Send(std::string_view(reinterpret_cast<const char *>(
                                                fbb.GetBufferPointer()),
                                            fbb.GetSize()),
                           peer.sac_assoc_id, peer.stream,
                           peer.connection->time_to_live() / 1000000)) {
            peer.server_connection_statistics->mutate_sent_packets(
                peer.server_connection_statistics->sent_packets() + 1);
          } else {
            peer.server_connection_statistics->mutate_dropped_packets(
                peer.server_connection_statistics->dropped_packets() + 1);
          }
        }
      }
      return peer.node_index;
    }
  }
  return -1;
}

MessageBridgeServer::MessageBridgeServer(aos::ShmEventLoop *event_loop,
                                         std::string config_sha256)
    : event_loop_(event_loop),
      timestamp_loggers_(event_loop_),
      server_(max_channels() + kControlStreams(), "",
              event_loop->node()->port()),
      server_status_(event_loop,
                     [this](const Context &context) {
                       timestamp_state_->SendData(&server_, &allocator_,
                                                  context);
                     }),
      config_sha256_(std::move(config_sha256)),
      allocator_(0) {
  CHECK_EQ(config_sha256_.size(), 64u) << ": Wrong length sha256sum";
  CHECK(event_loop_->node() != nullptr) << ": No nodes configured.";

  // Start out with a decent size big enough to hold timestamps.
  size_t max_size = 204;

  size_t destination_nodes = 0u;
  // Seed up all the per-node connection state.
  // We are making the assumption here that every connection is bidirectional
  // (data is being sent both ways).  This is pretty safe because we are
  // forwarding timestamps between nodes.
  for (std::string_view destination_node_name :
       configuration::DestinationNodeNames(event_loop->configuration(),
                                           event_loop->node())) {
    ++destination_nodes;
    // Find the largest connection message so we can size our buffers big enough
    // to receive a connection message.  The connect message comes from the
    // client to the server, so swap the node arguments.
    const size_t connect_size =
        MakeConnectMessage(event_loop->configuration(),
                           configuration::GetNode(event_loop->configuration(),
                                                  destination_node_name),
                           event_loop->node()->name()->string_view(),
                           UUID::Zero(), config_sha256_)
            .span()
            .size();
    VLOG(1) << "Connection to " << destination_node_name << " has size "
            << connect_size;
    max_size = std::max(max_size, connect_size);
  }

  // TODO(austin): Logging synchronization.
  event_loop_->epoll()->OnReadable(server_.fd(),
                                   [this]() { MessageReceived(); });

  LOG(INFO) << "Hostname: " << event_loop_->node()->hostname()->string_view();

  int channel_index = 0;
  size_t max_channel_buffer_size = 0u;
  size_t max_channel_size = 0u;
  size_t reliable_buffer_size = 0u;
  const Channel *const timestamp_channel = configuration::GetChannel(
      event_loop_->configuration(), "/aos", Timestamp::GetFullyQualifiedName(),
      event_loop_->name(), event_loop_->node());
  CHECK(timestamp_channel != nullptr)
      << ": Failed to find timestamp channel {\"name\": \"/aos\", \"type\": \""
      << Timestamp::GetFullyQualifiedName() << "\"}";
  CHECK(configuration::ChannelIsSendableOnNode(timestamp_channel,
                                               event_loop_->node()))
      << ": Timestamp channel is not sendable on this node.";

  for (const Channel *channel : *event_loop_->configuration()->channels()) {
    CHECK(channel->has_source_node());

    if (configuration::ChannelIsForwardedFromNode(channel,
                                                  event_loop_->node())) {
      bool any_reliable = false;
      for (const Connection *connection : *channel->destination_nodes()) {
        if (connection->time_to_live() == 0) {
          reliable_buffer_size +=
              static_cast<size_t>(channel->max_size() + kHeaderSizeOverhead());
          any_reliable = true;
        }
      }

      max_channel_size = std::max(
          static_cast<size_t>(channel->max_size() + kHeaderSizeOverhead()),
          max_channel_size);
      max_channel_buffer_size = std::max(
          static_cast<size_t>(channel->max_size() + kHeaderSizeOverhead()) *
              channel->destination_nodes()->size(),
          max_channel_buffer_size);

      std::unique_ptr<ChannelState> state(new ChannelState{
          channel, channel_index,
          any_reliable ? event_loop_->MakeRawFetcher(channel) : nullptr});

      for (const Connection *connection : *channel->destination_nodes()) {
        const Node *other_node = configuration::GetNode(
            event_loop_->configuration(), connection->name()->string_view());

        const bool delivery_time_is_logged =
            configuration::ConnectionDeliveryTimeIsLoggedOnNode(
                connection, event_loop_->node());

        state->AddPeer(
            connection,
            configuration::GetNodeIndex(event_loop_->configuration(),
                                        connection->name()->string_view()),
            server_status_.FindServerConnection(
                connection->name()->string_view()),
            &server_status_,
            configuration::ChannelMessageIsLoggedOnNode(channel, other_node),
            delivery_time_is_logged
                ? timestamp_loggers_.SenderForChannel(channel, connection)
                : nullptr);
      }

      // Don't subscribe to timestamps on the timestamp channel.  Those get
      // handled specially.
      if (channel != timestamp_channel) {
        // Call SendData for every message.
        ChannelState *state_ptr = state.get();
        event_loop_->MakeRawWatcher(
            channel, [this, state_ptr](const Context &context,
                                       const void * /*message*/) {
              state_ptr->SendData(&server_, &allocator_, context);
            });
      } else {
        for (const Connection *connection : *channel->destination_nodes()) {
          CHECK_GE(connection->time_to_live(), 1000u);
        }
        CHECK(timestamp_state_ == nullptr);
        timestamp_state_ = state.get();
      }
      channels_.emplace_back(std::move(state));
    } else if (channel == timestamp_channel) {
      std::unique_ptr<ChannelState> state(
          new ChannelState{channel, channel_index, nullptr});
      for (const Connection *connection : *channel->destination_nodes()) {
        CHECK_GE(connection->time_to_live(), 1000u);
      }
      timestamp_state_ = state.get();
      channels_.emplace_back(std::move(state));
    } else {
      channels_.emplace_back(nullptr);
    }
    ++channel_index;
  }
  CHECK(timestamp_state_ != nullptr);

  // Buffer up the max size a bit so everything fits nicely.
  LOG(INFO) << "Max message read size for all clients is " << max_size;
  LOG(INFO) << "Max message write size for all clients is "
            << max_channel_buffer_size;
  LOG(INFO) << "Reliable buffer size for all clients is "
            << reliable_buffer_size;
  server_.SetMaxReadSize(max_size);
  server_.SetMaxWriteSize(
      std::max(max_channel_buffer_size, reliable_buffer_size));

  // Since we are doing interleaving mode 1, we will see at most 1 message being
  // delivered at a time for an association.  That means, if a message is
  // started to be delivered, all the following parts will be from the same
  // message in the same stream.  The server can have at most 1 association per
  // client active, and can then (reasonably) have 1 new client connecting
  // trying to talk.  And 2 messages per association (one partially filled one,
  // and 1 new one with more of the data).
  server_.SetPoolSize((destination_nodes + 1) * 2);

  allocator_ = FixedAllocator(max_channel_size);

  reconnected_.reserve(max_channels());
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

    int new_node_index = channel_state->NodeDisconnected(assoc_id);
    if (new_node_index != -1) {
      node_index = new_node_index;
    }
  }

  if (node_index != -1) {
    VLOG(1) << "Resetting filters for " << node_index << " "
            << event_loop_->configuration()
                   ->nodes()
                   ->Get(node_index)
                   ->name()
                   ->string_view();
    server_status_.Disconnect(node_index);
    server_status_.ResetFilter(node_index);
    server_status_.ClearBootUUID(node_index);
    server_status_.ResetPartialDeliveries(node_index);
  }
}

void MessageBridgeServer::MessageReceived() {
  aos::unique_c_ptr<Message> message = server_.Read();
  if (!message) {
    return;
  }

  switch (message->message_type) {
    case Message::kNotification: {
      const union sctp_notification *snp =
          (const union sctp_notification *)message->data();

      if (VLOG_IS_ON(2)) {
        PrintNotification(message.get());
      }

      switch (snp->sn_header.sn_type) {
        case SCTP_ASSOC_CHANGE: {
          const struct sctp_assoc_change *sac = &snp->sn_assoc_change;
          switch (sac->sac_state) {
            case SCTP_RESTART:
              NodeDisconnected(sac->sac_assoc_id);
              [[fallthrough]];
            case SCTP_COMM_UP:
              NodeConnected(sac->sac_assoc_id);
              VLOG(1) << "Received up from " << message->PeerAddress() << " on "
                      << sac->sac_assoc_id << " state " << sac->sac_state;
              break;
            case SCTP_COMM_LOST:
            case SCTP_SHUTDOWN_COMP:
            case SCTP_CANT_STR_ASSOC:
              NodeDisconnected(sac->sac_assoc_id);
              VLOG(1) << "Disconnect from " << message->PeerAddress() << " on "
                      << sac->sac_assoc_id << " state " << sac->sac_state;
              break;
            default:
              LOG(FATAL) << "Never seen state " << sac->sac_state << " before.";
              break;
          }
        } break;
      }
    } break;
    case Message::kMessage:
      HandleData(message.get());
      break;
    case Message::kOverflow:
      MaybeIncrementInvalidConnectionCount(nullptr);
      NodeDisconnected(message->header.rcvinfo.rcv_assoc_id);
      break;
  }
  server_.FreeMessage(std::move(message));
}

void MessageBridgeServer::MaybeIncrementInvalidConnectionCount(
    const Node *node) {
  server_status_.increment_invalid_connection_count();

  if (node == nullptr) {
    return;
  }

  if (!node->has_name()) {
    return;
  }

  const aos::Node *client_node = configuration::GetNode(
      event_loop_->configuration(), node->name()->string_view());

  if (client_node == nullptr) {
    return;
  }

  const int node_index =
      configuration::GetNodeIndex(event_loop_->configuration(), client_node);

  ServerConnection *connection =
      server_status_.nodes()[node_index].value().server_connection;

  if (connection != nullptr) {
    connection->mutate_invalid_connection_count(
        connection->invalid_connection_count() + 1);
  }
}

void MessageBridgeServer::HandleData(const Message *message) {
  VLOG(2) << "Received data of length " << message->size;

  if (message->header.rcvinfo.rcv_sid == kConnectStream()) {
    // Control channel!
    const Connect *connect = flatbuffers::GetRoot<Connect>(message->data());
    {
      flatbuffers::Verifier verifier(message->data(), message->size);
      if (!connect->Verify(verifier)) {
        if (VLOG_IS_ON(1)) {
          LOG_EVERY_T(WARNING, 1.0)
              << "Failed to verify message, disconnecting client";
        }
        server_.Abort(message->header.rcvinfo.rcv_assoc_id);

        MaybeIncrementInvalidConnectionCount(nullptr);
        return;
      }
    }
    VLOG(1) << FlatbufferToJson(connect);

    if (!connect->has_config_sha256()) {
      if (VLOG_IS_ON(1)) {
        LOG(WARNING) << "Client missing config_sha256, disconnecting client";
      }
      server_.Abort(message->header.rcvinfo.rcv_assoc_id);

      MaybeIncrementInvalidConnectionCount(connect->node());
      return;
    }

    if (connect->config_sha256()->string_view() != config_sha256_) {
      if (VLOG_IS_ON(1)) {
        LOG(WARNING) << "Client config sha256 of "
                     << connect->config_sha256()->string_view()
                     << " doesn't match our config sha256 of " << config_sha256_
                     << ", disconnecting client";
      }
      server_.Abort(message->header.rcvinfo.rcv_assoc_id);

      MaybeIncrementInvalidConnectionCount(connect->node());
      return;
    }

    if (connect->channels_to_transfer()->size() >
        static_cast<size_t>(max_channels())) {
      if (VLOG_IS_ON(1)) {
        LOG(WARNING)
            << "Client has more channels than we do, disconnecting client";
      }
      server_.Abort(message->header.rcvinfo.rcv_assoc_id);

      MaybeIncrementInvalidConnectionCount(connect->node());
      return;
    }

    monotonic_clock::time_point monotonic_now = event_loop_->monotonic_now();

    // Account for the control channel and delivery times channel.
    size_t channel_index = kControlStreams();
    int node_index = -1;
    // TODO(sarah.newman): it would be better to do a refactor so that peers
    // don't belong to channels. I am trying to do a quick hack to reduce the
    // number of log messages without potentially losing information because the
    // number of messages is overwhelming right now at first boot. This also
    // should mean that we only send a single abort per association change,
    // which is more correct behavior.
    reconnected_.clear();
    for (const Channel *channel : *connect->channels_to_transfer()) {
      bool matched = false;
      for (std::unique_ptr<ChannelState> &channel_state : channels_) {
        if (channel_state.get() == nullptr) {
          continue;
        }
        if (channel_state->Matches(channel)) {
          node_index = channel_state->NodeConnected(
              connect->node(), message->header.rcvinfo.rcv_assoc_id,
              channel_index, &server_, &allocator_, monotonic_now,
              &reconnected_);
          CHECK_NE(node_index, -1)
              << ": Failed to find node "
              << aos::FlatbufferToJson(connect->node()) << " for connection "
              << aos::FlatbufferToJson(connect);
          matched = true;
          break;
        }
      }
      if (!matched) {
        if (VLOG_IS_ON(1)) {
          LOG(ERROR) << "Remote tried registering for unknown channel "
                     << FlatbufferToJson(channel);
        }
        server_.Abort(message->header.rcvinfo.rcv_assoc_id);

        MaybeIncrementInvalidConnectionCount(connect->node());
        return;
      }
      ++channel_index;
    }
    // TODO(sarah.newman): what if node_index is -1?
    server_status_.ResetFilter(node_index);
    server_status_.AddPartialDeliveries(node_index,
                                        message->partial_deliveries);
    server_status_.SetBootUUID(
        node_index, UUID::FromString(connect->boot_uuid()->string_view()));
    VLOG(1) << "Resetting filters for " << node_index << " "
            << event_loop_->configuration()
                   ->nodes()
                   ->Get(node_index)
                   ->name()
                   ->string_view();
    if (VLOG_IS_ON(1)) {
      message->LogRcvInfo();
    }
  } else if (message->header.rcvinfo.rcv_sid == kTimestampStream()) {
    // Message delivery
    const logger::MessageHeader *message_header =
        flatbuffers::GetRoot<logger::MessageHeader>(message->data());
    {
      flatbuffers::Verifier verifier(message->data(), message->size);
      CHECK(message_header->Verify(verifier));
    }

    CHECK_LT(message_header->channel_index(), channels_.size());
    CHECK_NOTNULL(channels_[message_header->channel_index()])
        ->HandleDelivery(
            message->header.rcvinfo.rcv_assoc_id,
            message->header.rcvinfo.rcv_ssn,
            absl::Span<const uint8_t>(message->data(), message->size),
            message->partial_deliveries, &server_status_);
    if (VLOG_IS_ON(2)) {
      message->LogRcvInfo();
    }
  } else {
    // We should never see the client sending us something on the wrong stream.
    // Just explode...  In theory, this could let a client DOS us, but we trust
    // the client.
    if (VLOG_IS_ON(2)) {
      message->LogRcvInfo();
    }
    LOG(FATAL) << "Unexpected stream id " << message->header.rcvinfo.rcv_sid;
  }
}

}  // namespace message_bridge
}  // namespace aos

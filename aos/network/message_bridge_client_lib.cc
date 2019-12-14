#include "aos/network/message_bridge_client_lib.h"

#include <chrono>
#include <string_view>

#include "aos/events/logging/logger.h"
#include "aos/events/shm_event_loop.h"
#include "aos/network/connect_generated.h"
#include "aos/network/message_bridge_client_generated.h"
#include "aos/network/message_bridge_protocol.h"
#include "aos/network/sctp_client.h"
#include "aos/unique_malloc_ptr.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

// This application receives messages from another node and re-publishes them on
// this node.
//
// To simulate packet loss for testing, run:
//   tc qdisc add dev eth0 root netem loss random 10
// To restore it, run:
//   tc qdisc del dev eth0 root netem

namespace aos {
namespace message_bridge {
namespace {
namespace chrono = std::chrono;

aos::FlatbufferDetachedBuffer<aos::message_bridge::Connect> MakeConnectMessage(
    const Configuration *config, const Node *my_node,
    std::string_view remote_name) {
  CHECK(config->has_nodes()) << ": Config must have nodes to transfer.";

  flatbuffers::FlatBufferBuilder fbb;

  flatbuffers::Offset<Node> node_offset = CopyFlatBuffer<Node>(my_node, &fbb);
  const std::string_view node_name = my_node->name()->string_view();

  std::vector<flatbuffers::Offset<Channel>> channel_offsets;
  for (const Channel *channel : *config->channels()) {
    if (channel->has_destination_nodes()) {
      for (const Connection *connection : *channel->destination_nodes()) {
        if (connection->name()->string_view() == node_name &&
            channel->source_node()->string_view() == remote_name) {
          channel_offsets.emplace_back(CopyFlatBuffer<Channel>(channel, &fbb));
        }
      }
    }
  }

  flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<Channel>>>
      channels_offset = fbb.CreateVector(channel_offsets);

  Connect::Builder connect_builder(fbb);
  connect_builder.add_channels_to_transfer(channels_offset);
  connect_builder.add_node(node_offset);
  fbb.Finish(connect_builder.Finish());

  return fbb.Release();
}

std::vector<int> StreamToChannel(const Configuration *config,
                                 const Node *my_node, const Node *other_node) {
  std::vector<int> stream_to_channel;
  int channel_index = 0;
  for (const Channel *channel : *config->channels()) {
    if (configuration::ChannelIsSendableOnNode(channel, other_node)) {
      const Connection *connection =
          configuration::ConnectionToNode(channel, my_node);
      if (connection != nullptr) {
        stream_to_channel.emplace_back(channel_index);
      }
    }
    ++channel_index;
  }

  return stream_to_channel;
}

std::vector<bool> StreamReplyWithTimestamp(const Configuration *config,
                                           const Node *my_node,
                                           const Node *other_node) {
  std::vector<bool> stream_reply_with_timestamp;
  int channel_index = 0;
  for (const Channel *channel : *config->channels()) {
    if (configuration::ChannelIsSendableOnNode(channel, other_node)) {
      const Connection *connection =
          configuration::ConnectionToNode(channel, my_node);
      if (connection != nullptr) {
        // We want to reply with a timestamp if the other node is logging the
        // timestamp (and it therefore needs the timestamp), or if we are
        // logging the message and it needs to know if we received it so it can
        // log (in the future) it through different mechanisms on failure.
        stream_reply_with_timestamp.emplace_back(
            configuration::ConnectionDeliveryTimeIsLoggedOnNode(connection,
                                                                other_node) ||
            configuration::ChannelMessageIsLoggedOnNode(channel, my_node));
      }
    }
    ++channel_index;
  }

  return stream_reply_with_timestamp;
}

aos::FlatbufferDetachedBuffer<aos::logger::MessageHeader>
MakeMessageHeaderReply() {
  flatbuffers::FlatBufferBuilder fbb;
  logger::MessageHeader::Builder message_header_builder(fbb);
  message_header_builder.add_channel_index(0);
  message_header_builder.add_monotonic_sent_time(0);
  message_header_builder.add_monotonic_remote_time(0);
  message_header_builder.add_realtime_remote_time(0);
  message_header_builder.add_remote_queue_index(0);
  fbb.Finish(message_header_builder.Finish());

  return fbb.Release();
}

FlatbufferDetachedBuffer<ClientStatistics> MakeClientStatistics(
    const std::vector<std::string_view> &source_node_names,
    const Configuration *configuration) {
  flatbuffers::FlatBufferBuilder fbb;

  std::vector<flatbuffers::Offset<ClientConnection>> connection_offsets;
  for (const std::string_view node_name : source_node_names) {
    flatbuffers::Offset<Node> node_offset =
        CopyFlatBuffer(configuration::GetNode(configuration, node_name), &fbb);
    ClientConnection::Builder connection_builder(fbb);
    connection_builder.add_node(node_offset);
    connection_builder.add_state(State::DISCONNECTED);
    // TODO(austin): Track dropped packets.
    connection_builder.add_received_packets(0);
    connection_offsets.emplace_back(connection_builder.Finish());
  }
  flatbuffers::Offset<
      flatbuffers::Vector<flatbuffers::Offset<ClientConnection>>>
      connections_offset = fbb.CreateVector(connection_offsets);

  ClientStatistics::Builder client_statistics_builder(fbb);
  client_statistics_builder.add_connections(connections_offset);
  fbb.Finish(client_statistics_builder.Finish());

  return fbb.Release();
}

}  // namespace

SctpClientConnection::SctpClientConnection(
    aos::ShmEventLoop *const event_loop, std::string_view remote_name,
    const Node *my_node, std::string_view local_host,
    std::vector<std::unique_ptr<aos::RawSender>> *channels,
    ClientConnection *connection)
    : event_loop_(event_loop),
      connect_message_(MakeConnectMessage(event_loop->configuration(), my_node,
                                          remote_name)),
      message_reception_reply_(MakeMessageHeaderReply()),
      remote_node_(CHECK_NOTNULL(
          configuration::GetNode(event_loop->configuration(), remote_name))),
      client_(remote_node_->hostname()->string_view(), remote_node_->port(),
              connect_message_.message().channels_to_transfer()->size() +
                  kControlStreams(),
              local_host, 0),
      channels_(channels),
      stream_to_channel_(
          StreamToChannel(event_loop->configuration(), my_node, remote_node_)),
      stream_reply_with_timestamp_(StreamReplyWithTimestamp(
          event_loop->configuration(), my_node, remote_node_)),
      connection_(connection) {
  VLOG(1) << "Connect request for " << remote_node_->name()->string_view()
          << ": " << FlatbufferToJson(connect_message_);

  connect_timer_ = event_loop_->AddTimer([this]() { SendConnect(); });
  event_loop_->OnRun(
      [this]() { connect_timer_->Setup(event_loop_->monotonic_now()); });

  event_loop_->epoll()->OnReadable(client_.fd(),
                                   [this]() { MessageReceived(); });
}

void SctpClientConnection::MessageReceived() {
  // Dispatch the message to the correct receiver.
  aos::unique_c_ptr<Message> message = client_.Read();

  if (message->message_type == Message::kNotification) {
    const union sctp_notification *snp =
        (const union sctp_notification *)message->data();

    switch (snp->sn_header.sn_type) {
      case SCTP_ASSOC_CHANGE: {
        const struct sctp_assoc_change *sac = &snp->sn_assoc_change;
        switch (sac->sac_state) {
          case SCTP_COMM_UP:
            NodeConnected(sac->sac_assoc_id);

            VLOG(1) << "Received up from " << message->PeerAddress() << " on "
                    << sac->sac_assoc_id;
            break;
          case SCTP_COMM_LOST:
          case SCTP_SHUTDOWN_COMP:
          case SCTP_CANT_STR_ASSOC: {
            NodeDisconnected();
          } break;
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

void SctpClientConnection::SendConnect() {
  // Try to send the connect message.  If that fails, retry.
  if (!client_.Send(kConnectStream(),
                    std::string_view(
                        reinterpret_cast<const char *>(connect_message_.data()),
                        connect_message_.size()),
                    0)) {
    NodeDisconnected();
  }
}

void SctpClientConnection::NodeConnected(sctp_assoc_t assoc_id) {
  connect_timer_->Disable();

  // We want to tell the kernel to schedule the packets on this new stream with
  // the priority scheduler.  This only needs to be done once per stream.
  client_.SetPriorityScheduler(assoc_id);

  remote_assoc_id_ = assoc_id;
  connection_->mutate_state(State::CONNECTED);
}

void SctpClientConnection::NodeDisconnected() {
  connect_timer_->Setup(
      event_loop_->monotonic_now() + chrono::milliseconds(100),
      chrono::milliseconds(100));
  remote_assoc_id_ = 0;
  connection_->mutate_state(State::DISCONNECTED);
}

void SctpClientConnection::HandleData(const Message *message) {
  const logger::MessageHeader *message_header =
      flatbuffers::GetSizePrefixedRoot<logger::MessageHeader>(message->data());

  connection_->mutate_received_packets(connection_->received_packets() + 1);

  const int stream = message->header.rcvinfo.rcv_sid - kControlStreams();

  // Publish the message.
  RawSender *sender = (*channels_)[stream_to_channel_[stream]].get();
  sender->Send(message_header->data()->data(), message_header->data()->size(),
               aos::monotonic_clock::time_point(
                   chrono::nanoseconds(message_header->monotonic_sent_time())),
               aos::realtime_clock::time_point(
                   chrono::nanoseconds(message_header->realtime_sent_time())),
               message_header->queue_index());

  if (stream_reply_with_timestamp_[stream]) {
    // TODO(austin): Send back less if we are only acking.  Maybe only a
    // stream id?  Nothing if we are only forwarding?

    // Now fill out the message received reply.  This uses a MessageHeader
    // container so it can be directly logged.
    message_reception_reply_.mutable_message()->mutate_channel_index(
        message_header->channel_index());
    message_reception_reply_.mutable_message()->mutate_monotonic_sent_time(
        message_header->monotonic_sent_time());

    // And capture the relevant data needed to generate the forwarding
    // MessageHeader.
    message_reception_reply_.mutable_message()->mutate_monotonic_remote_time(
        sender->monotonic_sent_time().time_since_epoch().count());
    message_reception_reply_.mutable_message()->mutate_realtime_remote_time(
        sender->realtime_sent_time().time_since_epoch().count());
    message_reception_reply_.mutable_message()->mutate_remote_queue_index(
        sender->sent_queue_index());

    // Unique ID is channel_index and monotonic clock.
    // TODO(austin): Depending on if we are the logger node or not, we need to
    // guarentee that this ack gets received too...  Same path as the logger.
    client_.Send(kTimestampStream(),
                 std::string_view(reinterpret_cast<const char *>(
                                      message_reception_reply_.data()),
                                  message_reception_reply_.size()),
                 0);
  }

  VLOG(1) << "Received data of length " << message->size << " from "
          << message->PeerAddress();

  if (VLOG_IS_ON(1)) {
    client_.LogSctpStatus(message->header.rcvinfo.rcv_assoc_id);
  }

  VLOG(2) << "\tSNDRCV (stream=" << message->header.rcvinfo.rcv_sid
          << " ssn=" << message->header.rcvinfo.rcv_ssn
          << " tsn=" << message->header.rcvinfo.rcv_tsn << " flags=0x"
          << std::hex << message->header.rcvinfo.rcv_flags << std::dec
          << " ppid=" << message->header.rcvinfo.rcv_ppid
          << " cumtsn=" << message->header.rcvinfo.rcv_cumtsn << ")";
}

MessageBridgeClient::MessageBridgeClient(aos::ShmEventLoop *event_loop)
    : event_loop_(event_loop),
      sender_(event_loop_->MakeSender<ClientStatistics>("/aos")),
      source_node_names_(configuration::SourceNodeNames(
          event_loop->configuration(), event_loop->node())),
      statistics_(MakeClientStatistics(source_node_names_,
                                       event_loop->configuration())) {
  std::string_view node_name = event_loop->node()->name()->string_view();

  // Find all the channels which are supposed to be delivered to us.
  channels_.resize(event_loop_->configuration()->channels()->size());
  int channel_index = 0;
  for (const Channel *channel : *event_loop_->configuration()->channels()) {
    if (channel->has_destination_nodes()) {
      for (const Connection *connection : *channel->destination_nodes()) {
        if (connection->name()->string_view() == node_name) {
          // Give the config a chance to remap us.  This helps with testing on a
          // single node.
          const Channel *mapped_channel = configuration::GetChannel(
              event_loop_->configuration(), channel->name()->string_view(),
              channel->type()->string_view(), event_loop_->name(),
              event_loop_->node());
          channels_[channel_index] = event_loop_->MakeRawSender(mapped_channel);
          break;
        }
      }
    }
    ++channel_index;
  }

  // Now, for each source node, build a connection.
  int node_index = 0;
  for (const std::string_view source_node : source_node_names_) {
    // Open an unspecified connection (:: in ipv6 terminology)
    connections_.emplace_back(new SctpClientConnection(
        event_loop, source_node, event_loop->node(), "::", &channels_,
        statistics_.mutable_message()->mutable_connections()->GetMutableObject(
            node_index)));
    ++node_index;
  }

  // And kick it all off.
  statistics_timer_ = event_loop_->AddTimer([this]() { SendStatistics(); });
  event_loop_->OnRun([this]() {
    statistics_timer_->Setup(event_loop_->monotonic_now() + chrono::seconds(1),
                             chrono::seconds(1));
  });
}

}  // namespace message_bridge
}  // namespace aos

#include "aos/network/message_bridge_client_lib.h"

#include <chrono>
#include <string_view>

#include "aos/events/logging/log_reader.h"
#include "aos/events/shm_event_loop.h"
#include "aos/network/connect_generated.h"
#include "aos/network/message_bridge_client_generated.h"
#include "aos/network/message_bridge_protocol.h"
#include "aos/network/remote_data_generated.h"
#include "aos/network/sctp_client.h"
#include "aos/network/timestamp_generated.h"
#include "aos/unique_malloc_ptr.h"
#include "aos/util/file.h"
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
  fbb.ForceDefaults(true);
  logger::MessageHeader::Builder message_header_builder(fbb);
  message_header_builder.add_channel_index(0);
  message_header_builder.add_monotonic_sent_time(0);
  message_header_builder.add_realtime_sent_time(0);
  message_header_builder.add_queue_index(0);
  message_header_builder.add_monotonic_remote_time(0);
  message_header_builder.add_realtime_remote_time(0);
  message_header_builder.add_remote_queue_index(0);
  fbb.Finish(message_header_builder.Finish());

  return fbb.Release();
}

}  // namespace

SctpClientConnection::SctpClientConnection(
    aos::ShmEventLoop *const event_loop, std::string_view remote_name,
    const Node *my_node, std::string_view local_host,
    std::vector<SctpClientChannelState> *channels, int client_index,
    MessageBridgeClientStatus *client_status)
    : event_loop_(event_loop),
      connect_message_(MakeConnectMessage(event_loop->configuration(), my_node,
                                          remote_name,
                                          event_loop->boot_uuid())),
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
      client_status_(client_status),
      client_index_(client_index),
      connection_(client_status_->GetClientConnection(client_index_)) {
  VLOG(1) << "Connect request for " << remote_node_->name()->string_view()
          << ": " << FlatbufferToJson(connect_message_);

  connect_timer_ = event_loop_->AddTimer([this]() { SendConnect(); });
  connect_timer_->set_name(std::string("connect_") +
                           remote_node_->name()->str());
  event_loop_->OnRun(
      [this]() { connect_timer_->Setup(event_loop_->monotonic_now()); });

  int max_size = connect_message_.span().size();

  for (const Channel *channel : *event_loop_->configuration()->channels()) {
    CHECK(channel->has_source_node());

    if (configuration::ChannelIsSendableOnNode(channel, remote_node_) &&
        configuration::ChannelIsReadableOnNode(channel, event_loop_->node())) {
      LOG(INFO) << "Receiving channel "
                << configuration::CleanedChannelToString(channel);
      max_size = std::max(channel->max_size(), max_size);
    }
  }

  // Buffer up the max size a bit so everything fits nicely.
  LOG(INFO) << "Max message size for all servers is " << max_size;
  client_.SetMaxSize(max_size + 100);

  event_loop_->epoll()->OnReadable(client_.fd(),
                                   [this]() { MessageReceived(); });
}

void SctpClientConnection::MessageReceived() {
  // Dispatch the message to the correct receiver.
  aos::unique_c_ptr<Message> message = client_.Read();
  if (!message) {
    return;
  }

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
  VLOG(1) << "Sending Connect";
  // Try to send the connect message.  If that fails, retry.
  if (client_.Send(kConnectStream(),
                   std::string_view(reinterpret_cast<const char *>(
                                        connect_message_.span().data()),
                                    connect_message_.span().size()),
                   0)) {
    ScheduleConnectTimeout();
  } else {
    NodeDisconnected();
  }
}

void SctpClientConnection::NodeConnected(sctp_assoc_t assoc_id) {
  ScheduleConnectTimeout();

  // We want to tell the kernel to schedule the packets on this new stream with
  // the priority scheduler.  This only needs to be done once per stream.
  client_.SetPriorityScheduler(assoc_id);

  client_status_->Connect(client_index_);
}

void SctpClientConnection::NodeDisconnected() {
  connect_timer_->Setup(
      event_loop_->monotonic_now() + chrono::milliseconds(100),
      chrono::milliseconds(100));
  client_status_->Disconnect(client_index_);
  client_status_->SampleReset(client_index_);
}

void SctpClientConnection::HandleData(const Message *message) {
  ScheduleConnectTimeout();

  const RemoteData *remote_data =
      flatbuffers::GetSizePrefixedRoot<RemoteData>(message->data());

  VLOG(1) << "Got a message of size " << message->size;
  CHECK_EQ(message->size, flatbuffers::GetPrefixedSize(message->data()) +
                              sizeof(flatbuffers::uoffset_t));
  {
    flatbuffers::Verifier verifier(message->data(), message->size);
    CHECK(remote_data->Verify(verifier));
  }

  const int stream = message->header.rcvinfo.rcv_sid - kControlStreams();
  SctpClientChannelState *channel_state =
      &((*channels_)[stream_to_channel_[stream]]);

  if (remote_data->queue_index() == channel_state->last_queue_index &&
      monotonic_clock::time_point(
          chrono::nanoseconds(remote_data->monotonic_sent_time())) ==
          channel_state->last_timestamp) {
    LOG(INFO) << "Duplicate message from " << message->PeerAddress();
    connection_->mutate_duplicate_packets(connection_->duplicate_packets() + 1);
    // Duplicate message, ignore.
  } else {
    connection_->mutate_received_packets(connection_->received_packets() + 1);
    connection_->mutate_partial_deliveries(connection_->partial_deliveries() +
                                           message->partial_deliveries);

    channel_state->last_queue_index = remote_data->queue_index();
    channel_state->last_timestamp = monotonic_clock::time_point(
        chrono::nanoseconds(remote_data->monotonic_sent_time()));

    // Publish the message.
    RawSender *sender = channel_state->sender.get();
    sender->CheckOk(sender->Send(
        remote_data->data()->data(), remote_data->data()->size(),
        monotonic_clock::time_point(
            chrono::nanoseconds(remote_data->monotonic_sent_time())),
        realtime_clock::time_point(
            chrono::nanoseconds(remote_data->realtime_sent_time())),
        remote_data->queue_index(),
        UUID::FromVector(remote_data->boot_uuid())));

    client_status_->SampleFilter(
        client_index_,
        monotonic_clock::time_point(
            chrono::nanoseconds(remote_data->monotonic_sent_time())),
        sender->monotonic_sent_time(),
        UUID::FromVector(remote_data->boot_uuid()));

    if (stream_reply_with_timestamp_[stream]) {
      // TODO(austin): Send back less if we are only acking.  Maybe only a
      // stream id?  Nothing if we are only forwarding?

      // Now fill out the message received reply.  This uses a MessageHeader
      // container so it can be directly logged.
      message_reception_reply_.mutable_message()->mutate_channel_index(
          remote_data->channel_index());
      message_reception_reply_.mutable_message()->mutate_monotonic_sent_time(
          remote_data->monotonic_sent_time());
      message_reception_reply_.mutable_message()->mutate_realtime_sent_time(
          remote_data->realtime_sent_time());
      message_reception_reply_.mutable_message()->mutate_queue_index(
          remote_data->queue_index());

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
                                        message_reception_reply_.span().data()),
                                    message_reception_reply_.span().size()),
                   0);
    }
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
    : event_loop_(event_loop), client_status_(event_loop_) {
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

          channels_[channel_index].sender =
              event_loop_->MakeRawSender(mapped_channel);

          std::unique_ptr<aos::RawFetcher> raw_fetcher =
              event_loop_->MakeRawFetcher(mapped_channel);
          raw_fetcher->Fetch();

          if (raw_fetcher->context().data != nullptr) {
            VLOG(1) << "Found data on "
                    << configuration::CleanedChannelToString(channel)
                    << ", won't resend it.";
            channels_[channel_index].last_queue_index =
                raw_fetcher->context().queue_index;
            channels_[channel_index].last_timestamp =
                raw_fetcher->context().monotonic_remote_time;
          }

          break;
        }
      }
    }
    ++channel_index;
  }

  // Now, for each source node, build a connection.
  for (const std::string_view source_node : configuration::SourceNodeNames(
           event_loop->configuration(), event_loop->node())) {
    // Open an unspecified connection (:: in ipv6 terminology)
    connections_.emplace_back(new SctpClientConnection(
        event_loop, source_node, event_loop->node(), "", &channels_,
        client_status_.FindClientIndex(source_node), &client_status_));
  }
}

}  // namespace message_bridge
}  // namespace aos

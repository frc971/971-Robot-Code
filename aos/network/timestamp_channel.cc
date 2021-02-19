#include "aos/network/timestamp_channel.h"

#include "absl/strings/str_cat.h"

namespace aos {
namespace message_bridge {

ChannelTimestampSender::ChannelTimestampSender(aos::EventLoop *event_loop)
    : event_loop_(event_loop) {
  CHECK(configuration::MultiNode(event_loop_->configuration()));
  timestamp_loggers_.resize(event_loop_->configuration()->nodes()->size());
}

aos::Sender<RemoteMessage> *ChannelTimestampSender::SenderForChannel(
    const Channel *channel, const Connection *connection) {
  // Look at any pre-created channel/connection pairs.
  auto it =
      channel_timestamp_loggers_.find(std::make_pair(channel, connection));
  if (it != channel_timestamp_loggers_.end()) {
    return it->second.get();
  }

  const Node *other_node = configuration::GetNode(
      event_loop_->configuration(), connection->name()->string_view());
  const size_t other_node_index =
      configuration::GetNodeIndex(event_loop_->configuration(), other_node);

  std::string type(channel->type()->string_view());
  std::replace(type.begin(), type.end(), '.', '-');
  const std::string single_timestamp_channel =
      absl::StrCat("/aos/remote_timestamps/", connection->name()->string_view(),
                   channel->name()->string_view(), "/", type);
  if (event_loop_->HasChannel<RemoteMessage>(single_timestamp_channel)) {
    LOG(INFO) << "Making RemoteMessage channel " << single_timestamp_channel;
    auto result = channel_timestamp_loggers_.try_emplace(
        std::make_pair(channel, connection),
        std::make_unique<aos::Sender<RemoteMessage>>(
            event_loop_->MakeSender<RemoteMessage>(single_timestamp_channel)));
    return result.first->second.get();
  } else {
    // Then look for any per-remote-node channels.
    if (timestamp_loggers_[other_node_index]) {
      return &timestamp_loggers_[other_node_index];
    }
    const std::string shared_timestamp_channel = absl::StrCat(
        "/aos/remote_timestamps/", connection->name()->string_view());
    LOG(INFO) << "Looking for " << shared_timestamp_channel;
    if (event_loop_->HasChannel<RemoteMessage>(shared_timestamp_channel)) {
      LOG(WARNING) << "Failed to find timestamp channel {\"name\": \""
                   << single_timestamp_channel << "\", \"type\": \""
                   << RemoteMessage::GetFullyQualifiedName()
                   << "\"}, falling back to old version.";
      timestamp_loggers_[other_node_index] =
          event_loop_->MakeSender<RemoteMessage>(shared_timestamp_channel);
      return &timestamp_loggers_[other_node_index];
    } else {
      LOG(ERROR) << "Failed";
    }

    // Explode with an error about the new channel.
    event_loop_->MakeSender<RemoteMessage>(single_timestamp_channel);
    return nullptr;
  }
}

}  // namespace message_bridge
}  // namespace aos

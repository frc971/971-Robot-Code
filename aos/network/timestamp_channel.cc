#include "aos/network/timestamp_channel.h"

#include "absl/strings/str_cat.h"

namespace aos {
namespace message_bridge {

ChannelTimestampFinder::ChannelTimestampFinder(
    const Configuration *configuration, const std::string_view name,
    const Node *node)
    : configuration_(configuration), name_(name), node_(node) {}

std::string ChannelTimestampFinder::SplitChannelName(
    const Channel *channel, const Connection *connection) {
  return SplitChannelName(channel->name()->string_view(), channel->type()->str(), connection);
}

std::string ChannelTimestampFinder::SplitChannelName(
    std::string_view name, std::string type, const Connection *connection) {
  std::replace(type.begin(), type.end(), '.', '-');
  return absl::StrCat("/aos/remote_timestamps/",
                      connection->name()->string_view(), name, "/", type);
}

std::string ChannelTimestampFinder::CombinedChannelName(
    std::string_view remote_node) {
  return absl::StrCat("/aos/remote_timestamps/", remote_node);
}

const Channel *ChannelTimestampFinder::ForChannel(
    const Channel *channel, const Connection *connection) {
  const std::string split_timestamp_channel_name =
      SplitChannelName(channel, connection);
  const Channel *split_timestamp_channel = configuration::GetChannel(
      configuration_, split_timestamp_channel_name,
      RemoteMessage::GetFullyQualifiedName(), name_, node_, true);
  if (split_timestamp_channel != nullptr) {
    return split_timestamp_channel;
  }

  const std::string shared_timestamp_channel_name =
      CombinedChannelName(connection->name()->string_view());
  const Channel *shared_timestamp_channel = configuration::GetChannel(
      configuration_, shared_timestamp_channel_name,
      RemoteMessage::GetFullyQualifiedName(), name_, node_, true);
  if (shared_timestamp_channel != nullptr) {
    LOG(WARNING) << "Failed to find timestamp channel {\"name\": \""
                 << split_timestamp_channel_name << "\", \"type\": \""
                 << RemoteMessage::GetFullyQualifiedName()
                 << "\"}, falling back to old version.";
    return shared_timestamp_channel;
  }

  CHECK(shared_timestamp_channel != nullptr)
      << ": Remote timestamp channel { \"name\": \""
      << split_timestamp_channel_name << "\", \"type\": \""
      << RemoteMessage::GetFullyQualifiedName()
      << "\" } not found in config for " << name_
      << (configuration::MultiNode(configuration_)
              ? absl::StrCat(" on node ", node_->name()->string_view())
              : ".");

  return nullptr;
}

ChannelTimestampSender::ChannelTimestampSender(aos::EventLoop *event_loop)
    : event_loop_(event_loop) {
  if (event_loop_) {
    CHECK(configuration::MultiNode(event_loop_->configuration()));
  }
}

aos::Sender<RemoteMessage> *ChannelTimestampSender::SenderForChannel(
    const Channel *channel, const Connection *connection) {
  CHECK(event_loop_);

  ChannelTimestampFinder finder(event_loop_);
  // Look at any pre-created channel/connection pairs.
  {
    auto it =
        channel_timestamp_loggers_.find(std::make_pair(channel, connection));
    if (it != channel_timestamp_loggers_.end()) {
      return it->second.get();
    }
  }

  const Channel *timestamp_channel = finder.ForChannel(channel, connection);

  {
    auto it = timestamp_loggers_.find(timestamp_channel);
    if (it != timestamp_loggers_.end()) {
      CHECK(channel_timestamp_loggers_
                .try_emplace(std::make_pair(channel, connection), it->second)
                .second);
      return it->second.get();
    }
  }

  auto result = channel_timestamp_loggers_.try_emplace(
      std::make_pair(channel, connection),
      std::make_shared<aos::Sender<RemoteMessage>>(
          event_loop_->MakeSender<RemoteMessage>(
              timestamp_channel->name()->string_view())));

  CHECK(timestamp_loggers_.try_emplace(timestamp_channel, result.first->second)
            .second);
  return result.first->second.get();
}

}  // namespace message_bridge
}  // namespace aos

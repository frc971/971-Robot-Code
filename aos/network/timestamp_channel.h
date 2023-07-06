#ifndef AOS_NETWORK_TIMESTAMP_CHANNEL_
#define AOS_NETWORK_TIMESTAMP_CHANNEL_

#include <string_view>
#include <vector>

#include "absl/container/btree_map.h"
#include "glog/logging.h"

#include "aos/configuration.h"
#include "aos/events/event_loop.h"
#include "aos/network/remote_message_generated.h"

namespace aos {
namespace message_bridge {

// Class to find the corresponding channel where timestamps for a specified data
// channel and connection will be logged.
//
// This abstracts (and detects) when we have combined or split remote timestamp
// logging channels.
class ChannelTimestampFinder {
 public:
  ChannelTimestampFinder(aos::EventLoop *event_loop)
      : ChannelTimestampFinder(event_loop->configuration(), event_loop->name(),
                               event_loop->node()) {}
  ChannelTimestampFinder(const Configuration *configuration,
                         const std::string_view name, const Node *node);

  // Returns the split timestamp logging channel for the provide channel and
  // connection if one exists, or nullptr otherwise.
  const Channel *SplitChannelForChannel(const Channel *channel,
                                        const Connection *connection);

  // Finds the timestamp logging channel for the provided data channel and
  // connection.
  const Channel *ForChannel(const Channel *channel,
                            const Connection *connection);
  std::string SplitChannelName(const Channel *channel,
                               const Connection *connection);
  std::string SplitChannelName(std::string_view name, std::string type,
                               const Connection *connection);
  std::string CombinedChannelName(std::string_view remote_node);

 private:
  const Configuration *configuration_;
  const std::string_view name_;
  const Node *node_;
};

// Class to manage lifetime, and creating senders for channels and connections.
class ChannelTimestampSender {
 public:
  ChannelTimestampSender(aos::EventLoop *event_loop);
  ChannelTimestampSender() : event_loop_(nullptr) {}

  ChannelTimestampSender(ChannelTimestampSender &&other) noexcept = default;
  ChannelTimestampSender &operator=(ChannelTimestampSender &&other) noexcept =
      default;

  aos::Sender<RemoteMessage> *SenderForChannel(const Channel *channel,
                                               const Connection *connection);
  void ClearSenderForChannel(const Channel *channel,
                             const Connection *connection);

 private:
  aos::EventLoop *event_loop_;

  // We've got 3 cases to consider.
  // 1) The old single channel per connection exists.
  // 2) A new channel per channel exists.
  // 3) Both exist.
  //
  // I want the default to be such that if no channel is found, we explode
  // looking for the single channel per channel.  This means users will add the
  // new channel when blindly fixing errors, which is what we want.
  //
  // I'd prefer 3) to be an error, but don't have strong opinions.  We will
  // still be correct if it gets used, as long as everything is consistent.

  // Mapping from channel and connection to logger.
  absl::btree_map<std::pair<const Channel *, const Connection *>,
                  std::shared_ptr<aos::Sender<RemoteMessage>>>
      channel_timestamp_loggers_;

  // Mapping from channel to RemoteMessage sender.  This is the channel that
  // timestamps are published to.
  absl::btree_map<const Channel *, std::shared_ptr<aos::Sender<RemoteMessage>>>
      timestamp_loggers_;
};

}  // namespace message_bridge
}  // namespace aos

#endif  // AOS_NETWORK_TIMESTAMP_CHANNEL_

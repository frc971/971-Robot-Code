#ifndef AOS_NETWORK_TIMESTAMP_CHANNEL_
#define AOS_NETWORK_TIMESTAMP_CHANNEL_

#include <vector>

#include "absl/container/btree_map.h"
#include "aos/configuration.h"
#include "aos/events/event_loop.h"
#include "aos/network/remote_message_generated.h"
#include "glog/logging.h"

namespace aos {
namespace message_bridge {

// Class to manage lifetime, and creating senders for channels and connections.
class ChannelTimestampSender {
 public:
  ChannelTimestampSender(aos::EventLoop *event_loop);

  aos::Sender<RemoteMessage> *SenderForChannel(const Channel *channel,
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

  // List of Senders per node.
  std::vector<aos::Sender<RemoteMessage>> timestamp_loggers_;

  // Mapping from channel and connection to logger.
  absl::btree_map<std::pair<const Channel *, const Connection *>,
                  std::unique_ptr<aos::Sender<RemoteMessage>>>
      channel_timestamp_loggers_;
};

}  // namespace message_bridge
}  // namespace aos

#endif  // AOS_NETWORK_TIMESTAMP_CHANNEL_

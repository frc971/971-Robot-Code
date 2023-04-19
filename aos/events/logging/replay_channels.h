#ifndef AOS_EVENTS_LOGGING_REPLAY_CHANNELS_H_
#define AOS_EVENTS_LOGGING_REPLAY_CHANNELS_H_

#include <string>
#include <vector>

namespace aos {
namespace logger {
// Vector of pair of name and type of the channel
using ReplayChannels = std::vector<std::pair<std::string, std::string>>;
// Vector of channel indices
using ReplayChannelIndices = std::vector<size_t>;
}  // namespace logger
}  // namespace aos
#endif  // AOS_EVENTS_LOGGING_REPLAY_CHANNELS_H_

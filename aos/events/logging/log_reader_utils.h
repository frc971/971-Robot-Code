#ifndef AOS_EVENTS_LOGGING_LOG_READER_UTILS_H_
#define AOS_EVENTS_LOGGING_LOG_READER_UTILS_H_

#include "aos/events/logging/log_reader.h"

namespace aos::logger {

// Utility struct for returning all channels segregated as senders, watchers and
// fetchers
struct ChannelsInLogResult {
  std::optional<std::vector<aos::ChannelT>> senders;
  std::optional<std::vector<aos::ChannelT>> watchers;
  std::optional<std::vector<aos::ChannelT>> fetchers;
  std::optional<std::vector<aos::ChannelT>>
      watchers_and_fetchers_without_senders;
};  // struct ChannelsInLogResult

// A struct to select what kind of channels we want to extract from the log
struct ChannelsInLogOptions {
  bool get_senders = false;
  bool get_watchers = false;
  bool get_fetchers = false;
};  // struct ChannelsInLogOptions

// Reads the first ~1 second of timing reports in a logfile and generates a list
// of all the channels sent on by the specified applications on the specified
// nodes.
ChannelsInLogResult ChannelsInLog(
    const std::vector<aos::logger::LogFile> &log_files,
    const std::vector<const aos::Node *> &nodes,
    const std::vector<std::string> &applications,
    const ChannelsInLogOptions options = ChannelsInLogOptions{true, true,
                                                              true});
// Wrapper for channelsinlog but only for sender channels
std::vector<aos::ChannelT> SenderChannelsInLog(
    const std::vector<aos::logger::LogFile> &log_files,
    const std::vector<const aos::Node *> &nodes,
    const std::vector<std::string> &applications);

// Wrapper for channelsinlog but only for watcher channels
std::vector<aos::ChannelT> WatcherChannelsInLog(
    const std::vector<aos::logger::LogFile> &log_files,
    const std::vector<const aos::Node *> &nodes,
    const std::vector<std::string> &applications);

// Wrapper for channelsinlog but only for fetcher channels
std::vector<aos::ChannelT> FetcherChannelsInLog(
    const std::vector<aos::logger::LogFile> &log_files,
    const std::vector<const aos::Node *> &nodes,
    const std::vector<std::string> &applications);

}  // namespace aos::logger

#endif  // AOS_EVENTS_LOGGING_LOG_READER_UTILS_H_

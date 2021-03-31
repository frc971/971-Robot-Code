#ifndef AOS_EVENTS_LOGGING_LOG_WRITER_H_
#define AOS_EVENTS_LOGGING_LOG_WRITER_H_

#include <chrono>
#include <string_view>
#include <vector>

#include "aos/events/event_loop.h"
#include "aos/events/logging/log_namer.h"
#include "aos/events/logging/logfile_utils.h"
#include "aos/events/logging/logger_generated.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/network/message_bridge_server_generated.h"
#include "aos/network/remote_message_generated.h"
#include "aos/time/time.h"
#include "aos/uuid.h"
#include "flatbuffers/flatbuffers.h"

namespace aos {
namespace logger {

// Logs all channels available in the event loop to disk every 100 ms.
// Start by logging one message per channel to capture any state and
// configuration that is sent rately on a channel and would affect execution.
class Logger {
 public:
  // Constructs a logger.
  //   event_loop: The event loop used to read the messages.
  //   configuration: When provided, this is the configuration to log, and the
  //     configuration to use for the channel list to log.  If not provided,
  //     this becomes the configuration from the event loop.
  //   should_log: When provided, a filter for channels to log. If not provided,
  //     all available channels are logged.
  Logger(EventLoop *event_loop)
      : Logger(event_loop, event_loop->configuration()) {}
  Logger(EventLoop *event_loop, const Configuration *configuration)
      : Logger(event_loop, configuration,
               [](const Channel *) { return true; }) {}
  Logger(EventLoop *event_loop, const Configuration *configuration,
         std::function<bool(const Channel *)> should_log);
  ~Logger();

  // Overrides the name in the log file header.
  void set_name(std::string_view name) { name_ = name; }

  // Sets the callback to run after each period of data is logged. Defaults to
  // doing nothing.
  //
  // This callback may safely do things like call Rotate().
  void set_on_logged_period(std::function<void()> on_logged_period) {
    on_logged_period_ = std::move(on_logged_period);
  }

  void set_separate_config(bool separate_config) {
    separate_config_ = separate_config;
  }

  // Sets the period between polling the data. Defaults to 100ms.
  //
  // Changing this while a set of files is being written may result in
  // unreadable files.
  void set_polling_period(std::chrono::nanoseconds polling_period) {
    polling_period_ = polling_period;
  }

  std::string_view log_start_uuid() const { return log_start_uuid_; }
  UUID logger_instance_uuid() const { return logger_instance_uuid_; }

  // The maximum time for a single fetch which returned a message, or 0 if none
  // of those have happened.
  std::chrono::nanoseconds max_message_fetch_time() const {
    return max_message_fetch_time_;
  }
  // The channel for that longest fetch which returned a message, or -1 if none
  // of those have happened.
  int max_message_fetch_time_channel() const {
    return max_message_fetch_time_channel_;
  }
  // The size of the message returned by that longest fetch, or -1 if none of
  // those have happened.
  int max_message_fetch_time_size() const {
    return max_message_fetch_time_size_;
  }
  // The total time spent fetching messages.
  std::chrono::nanoseconds total_message_fetch_time() const {
    return total_message_fetch_time_;
  }
  // The total number of fetch calls which returned messages.
  int total_message_fetch_count() const { return total_message_fetch_count_; }
  // The total number of bytes fetched.
  int64_t total_message_fetch_bytes() const {
    return total_message_fetch_bytes_;
  }

  // The total time spent in fetches which did not return a message.
  std::chrono::nanoseconds total_nop_fetch_time() const {
    return total_nop_fetch_time_;
  }
  // The total number of fetches which did not return a message.
  int total_nop_fetch_count() const { return total_nop_fetch_count_; }

  // The maximum time for a single copy, or 0 if none of those have happened.
  std::chrono::nanoseconds max_copy_time() const { return max_copy_time_; }
  // The channel for that longest copy, or -1 if none of those have happened.
  int max_copy_time_channel() const { return max_copy_time_channel_; }
  // The size of the message for that longest copy, or -1 if none of those have
  // happened.
  int max_copy_time_size() const { return max_copy_time_size_; }
  // The total time spent copying messages.
  std::chrono::nanoseconds total_copy_time() const { return total_copy_time_; }
  // The total number of messages copied.
  int total_copy_count() const { return total_copy_count_; }
  // The total number of bytes copied.
  int64_t total_copy_bytes() const { return total_copy_bytes_; }

  void ResetStatisics();

  // Rotates the log file(s), triggering new part files to be written for each
  // log file.
  void Rotate();

  // Starts logging to files with the given naming scheme.
  //
  // log_start_uuid may be used to tie this log event to other log events across
  // multiple nodes. The default (empty string) indicates there isn't one
  // available.
  void StartLogging(std::unique_ptr<LogNamer> log_namer,
                    std::string_view log_start_uuid = "");

  // Stops logging. Ensures any messages through end_time make it into the log.
  //
  // If you want to stop ASAP, pass min_time to avoid reading any more messages.
  //
  // Returns the LogNamer in case the caller wants to do anything else with it
  // before destroying it.
  std::unique_ptr<LogNamer> StopLogging(
      aos::monotonic_clock::time_point end_time);

  // Returns whether a log is currently being written.
  bool is_started() const { return static_cast<bool>(log_namer_); }

  // Shortcut to call StartLogging with a LocalLogNamer when event processing
  // starts.
  void StartLoggingLocalNamerOnRun(std::string base_name) {
    event_loop_->OnRun([this, base_name]() {
      StartLogging(
          std::make_unique<LocalLogNamer>(base_name, event_loop_->node()));
    });
  }

 private:
  // Structure to track both a fetcher, and if the data fetched has been
  // written.  We may want to delay writing data to disk so that we don't let
  // data get too far out of order when written to disk so we can avoid making
  // it too hard to sort when reading.
  struct FetcherStruct {
    std::unique_ptr<RawFetcher> fetcher;
    bool written = false;

    // Channel index to log to.
    int channel_index = -1;
    const Channel *channel = nullptr;
    const Node *timestamp_node = nullptr;

    LogType log_type = LogType::kLogMessage;

    // We fill out the metadata at construction, but the actual writers have to
    // be updated each time we start logging. To avoid duplicating the complex
    // logic determining whether each writer should be initialized, we just
    // stash the answer in separate member variables.
    bool wants_writer = false;
    DetachedBufferWriter *writer = nullptr;
    bool wants_timestamp_writer = false;
    DetachedBufferWriter *timestamp_writer = nullptr;
    bool wants_contents_writer = false;
    DetachedBufferWriter *contents_writer = nullptr;

    // Node which this data is from, or -1 if it is unknown.
    int data_node_index = -1;
    // Node that this timestamp is for, or -1 if it is known.
    int timestamp_node_index = -1;
    // Node that the contents this contents_writer will log are from.
    int contents_node_index = -1;
  };

  // Vector mapping from the channel index from the event loop to the logged
  // channel index.
  std::vector<int> event_loop_to_logged_channel_index_;

  struct NodeState {
    aos::monotonic_clock::time_point monotonic_start_time =
        aos::monotonic_clock::min_time;
    aos::realtime_clock::time_point realtime_start_time =
        aos::realtime_clock::min_time;

    bool has_source_node_boot_uuid = false;

    // This is an initial UUID that is a valid UUID4 and is pretty obvious that
    // it isn't valid.
    UUID source_node_boot_uuid = UUID::Zero();

    aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader> log_file_header =
        aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader>::Empty();

    // True if a header has been written to the start of a log file.
    bool header_written = false;
    // True if the current written header represents the contents which will
    // follow.  This is cleared when boot_uuid is known to not match anymore.
    bool header_valid = false;

    // Sets the source_node_boot_uuid, properly updating everything.  Returns
    // true if it changed, false otherwise.
    bool SetBootUUID(const UUID &new_source_node_boot_uuid) {
      if (has_source_node_boot_uuid &&
          source_node_boot_uuid == new_source_node_boot_uuid) {
        return false;
      }
      source_node_boot_uuid = new_source_node_boot_uuid;
      header_valid = false;
      has_source_node_boot_uuid = true;

      flatbuffers::String *source_node_boot_uuid_string =
          log_file_header.mutable_message()->mutable_source_node_boot_uuid();
      CHECK_EQ(UUID::kStringSize, source_node_boot_uuid_string->size());
      source_node_boot_uuid.CopyTo(source_node_boot_uuid_string->data());

      return true;
    }
  };

  void WriteHeader();

  aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader> MakeHeader(
      const Node *node, std::string_view config_sha256);

  // Writes the header for the provided node if enough information is valid.
  void MaybeWriteHeader(int node_index);
  // Overload for when we already know node as well.
  void MaybeWriteHeader(int node_index, const Node *node);

  bool MaybeUpdateTimestamp(
      const Node *node, int node_index,
      aos::monotonic_clock::time_point monotonic_start_time,
      aos::realtime_clock::time_point realtime_start_time);

  void DoLogData(const monotonic_clock::time_point end_time,
                 bool run_on_logged);

  void WriteMissingTimestamps();

  // Fetches from each channel until all the data is logged.  This is dangerous
  // because it lets you log for more than 1 period.  All calls need to verify
  // that t isn't greater than 1 period in the future.
  void LogUntil(monotonic_clock::time_point t);

  void RecordFetchResult(aos::monotonic_clock::time_point start,
                         aos::monotonic_clock::time_point end, bool got_new,
                         FetcherStruct *fetcher);

  void RecordCreateMessageTime(aos::monotonic_clock::time_point start,
                               aos::monotonic_clock::time_point end,
                               FetcherStruct *fetcher);

  // Sets the start time for a specific node.
  void SetStartTime(
      size_t node_index, aos::monotonic_clock::time_point monotonic_start_time,
      aos::realtime_clock::time_point realtime_start_time,
      aos::monotonic_clock::time_point logger_monotonic_start_time,
      aos::realtime_clock::time_point logger_realtime_start_time);

  EventLoop *const event_loop_;
  // The configuration to place at the top of the log file.
  const Configuration *const configuration_;

  UUID log_event_uuid_ = UUID::Zero();
  const UUID logger_instance_uuid_ = UUID::Random();
  std::unique_ptr<LogNamer> log_namer_;
  // Empty indicates there isn't one.
  std::string log_start_uuid_;

  // Name to save in the log file.  Defaults to hostname.
  std::string name_;

  std::function<void()> on_logged_period_ = []() {};

  std::chrono::nanoseconds max_message_fetch_time_ =
      std::chrono::nanoseconds::zero();
  int max_message_fetch_time_channel_ = -1;
  int max_message_fetch_time_size_ = -1;
  std::chrono::nanoseconds total_message_fetch_time_ =
      std::chrono::nanoseconds::zero();
  int total_message_fetch_count_ = 0;
  int64_t total_message_fetch_bytes_ = 0;

  std::chrono::nanoseconds total_nop_fetch_time_ =
      std::chrono::nanoseconds::zero();
  int total_nop_fetch_count_ = 0;

  std::chrono::nanoseconds max_copy_time_ = std::chrono::nanoseconds::zero();
  int max_copy_time_channel_ = -1;
  int max_copy_time_size_ = -1;
  std::chrono::nanoseconds total_copy_time_ = std::chrono::nanoseconds::zero();
  int total_copy_count_ = 0;
  int64_t total_copy_bytes_ = 0;

  std::vector<FetcherStruct> fetchers_;
  TimerHandler *timer_handler_;

  // Period to poll the channels.
  std::chrono::nanoseconds polling_period_ = std::chrono::milliseconds(100);

  // Last time that data was written for all channels to disk.
  monotonic_clock::time_point last_synchronized_time_;

  // Max size that the header has consumed.  This much extra data will be
  // reserved in the builder to avoid reallocating.
  size_t max_header_size_ = 0;

  // If true, write the message header into a separate file.
  bool separate_config_ = true;

  // Fetcher for all the statistics from all the nodes.
  aos::Fetcher<message_bridge::ServerStatistics> server_statistics_fetcher_;

  std::vector<NodeState> node_state_;
};

}  // namespace logger
}  // namespace aos

#endif  // AOS_EVENTS_LOGGING_LOG_WRITER_H_

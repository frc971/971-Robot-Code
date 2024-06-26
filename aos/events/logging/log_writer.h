#ifndef AOS_EVENTS_LOGGING_LOG_WRITER_H_
#define AOS_EVENTS_LOGGING_LOG_WRITER_H_

#include <chrono>
#include <fstream>
#include <string_view>
#include <vector>

#include "flatbuffers/flatbuffers.h"

#include "aos/events/event_loop.h"
#include "aos/events/logging/log_namer.h"
#include "aos/events/logging/logfile_utils.h"
#include "aos/events/logging/logger_generated.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/network/message_bridge_server_generated.h"
#include "aos/network/remote_message_generated.h"
#include "aos/time/time.h"
#include "aos/uuid.h"

namespace aos::logger {

// Packs the provided configuration into the separate config LogFileHeader
// container.
aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader> PackConfiguration(
    const Configuration *const configuration);

// A class to manage the writing of profile data. It will open a file during
// construction and close it when it goes out of scope.
class ProfileDataWriter {
 public:
  // A constructor to open the stream.
  ProfileDataWriter(const std::filesystem::path &csv_path);

  // Write the profile data to the file as a csv line.
  void WriteProfileData(
      const aos::monotonic_clock::time_point message_time,
      const aos::monotonic_clock::time_point encoding_start_time,
      const std::chrono::nanoseconds encode_duration, const Channel &channel);

  // A destructor to close the stream if it's open.
  ~ProfileDataWriter() {
    if (stream_.is_open()) {
      stream_.close();
    }
  }

 private:
  // The stream to write profiling data to.
  std::ofstream stream_;
};

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

  void set_logger_sha1(std::string_view sha1) { logger_sha1_ = sha1; }
  void set_logger_version(std::string_view version) {
    logger_version_ = version;
  }

  // Sets the callback to run *after* each period of data is logged.  Defaults
  // to doing nothing.  The argument to the callback is the time which we just
  // wrote until.  This is not called when rotating or finishing logs.
  //
  // This callback may safely do things like call Rotate().
  void set_on_logged_period(
      std::function<void(aos::monotonic_clock::time_point t)>
          on_logged_period) {
    on_logged_period_ = std::move(on_logged_period);
  }

  void set_separate_config(bool separate_config) {
    separate_config_ = separate_config;
  }

  // Sets the amount to run the logger behind the current time.  This lets us
  // make decisions about rotating or stopping logging before something happens.
  // Using this to start logging in the past isn't yet supported.  This can be
  // changed at runtime, but will only influence future writes, not what is
  // already written.
  void set_logging_delay(std::chrono::nanoseconds logging_delay) {
    logging_delay_ = logging_delay;
  }
  // Returns the current logging delay.
  std::chrono::nanoseconds logging_delay() const { return logging_delay_; }

  // Sets the period between polling the data. Defaults to 100ms.
  //
  // Changing this while a set of files is being written may result in
  // unreadable files.
  void set_polling_period(std::chrono::nanoseconds polling_period) {
    polling_period_ = polling_period;
  }
  std::chrono::nanoseconds polling_period() const { return polling_period_; }

  // Sets the path to write profiling data to. nullopt will disable profiling.
  void SetProfilingPath(const std::optional<std::filesystem::path> &path);

  std::optional<UUID> log_start_uuid() const { return log_start_uuid_; }
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

  // The maximum time between when a message was sent and when it was logged.
  // This is 0 if no message has been logged.
  std::chrono::nanoseconds max_log_delay() const { return max_log_delay_; }
  // The channel for longest logging delay, or -1 if no messages have been
  // logged.
  int max_log_delay_channel() const { return max_log_delay_channel_; }

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
                    std::optional<UUID> log_start_uuid = std::nullopt);

  // Restarts logging using a new naming scheme. Intended for log rotation.
  // Returns a unique_ptr to the prior log_namer instance.  If provided,
  // end_time is the time to log until.  It must be in the past.  Times before
  // the last_synchronized_time are ignored.
  std::unique_ptr<LogNamer> RestartLogging(
      std::unique_ptr<LogNamer> log_namer,
      std::optional<UUID> log_start_uuid = std::nullopt,
      std::optional<monotonic_clock::time_point> end_time = std::nullopt);

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

  // Shortcut to call StartLogging with a MultiNodeFilesLogNamer when event
  // processing starts.
  // Doesn't try to use odirect.
  void StartLoggingOnRun(std::string base_name) {
    event_loop_->OnRun([this, base_name]() {
      StartLogging(std::make_unique<MultiNodeFilesLogNamer>(
          base_name, configuration_, event_loop_, node_));
    });
  }

  // Returns the current log event UUID.  This is randomly assigned when the log
  // starts or restarts.
  const UUID &log_event_uuid() const { return log_event_uuid_; }

 private:
  // Structure to track both a fetcher, and if the data fetched has been
  // written.  We may want to delay writing data to disk so that we don't let
  // data get too far out of order when written to disk so we can avoid making
  // it too hard to sort when reading.
  struct FetcherStruct {
    std::unique_ptr<RawFetcher> fetcher;
    bool written = false;

    // Index of the channel in the logged configuration (not necessarily the
    // event loop configuration).
    int logged_channel_index = -1;

    // Channel from the event_loop configuration.
    const Channel *event_loop_channel = nullptr;

    const Node *timestamp_node = nullptr;

    LogType log_type = LogType::kLogMessage;

    // We fill out the metadata at construction, but the actual writers have to
    // be updated each time we start logging. To avoid duplicating the complex
    // logic determining whether each writer should be initialized, we just
    // stash the answer in separate member variables.
    bool wants_writer = false;
    NewDataWriter *writer = nullptr;
    bool wants_timestamp_writer = false;
    NewDataWriter *timestamp_writer = nullptr;
    bool wants_contents_writer = false;
    NewDataWriter *contents_writer = nullptr;

    // Node which this data is from, or -1 if it is unknown.
    int data_node_index = -1;
    // Node that this timestamp is for, or -1 if it is known.
    int timestamp_node_index = -1;
    // Node that the contents this contents_writer will log are from.
    int contents_node_index = -1;

    // If true, this message is being sent over a reliable channel.
    bool reliable_forwarding = false;

    // One of the following will be populated.  If channel_reliable_contents is
    // non zero size, it contains a mapping from the event loop channel (not the
    // logged channel) to a bool telling us if that particular channel is
    // reliable.
    //
    // If channel_reliable_contents is empty, reliable_contents will contain the
    // same info for all contents logged here.  This is the predominant case for
    // split timestamp channels (the prefered approach).
    bool reliable_contents = false;
    std::vector<bool> channel_reliable_contents;
  };

  // Vector mapping from the channel index from the event loop to the logged
  // channel index.
  // When using the constructor that allows manually specifying the
  // configuration, that configuration may have different channels than the
  // event loop's configuration. When there is a channel that is included in the
  // event loop configuration but not in the specified configuration, the value
  // in this mapping will be nullopt for that channel. Nullopt will result in
  // that channel not being included in the output log's configuration or data.
  std::vector<std::optional<uint32_t>> event_loop_to_logged_channel_index_;

  // Start/Restart write configuration into LogNamer space.
  std::string WriteConfiguration(LogNamer *log_namer);

  void WriteHeader(aos::monotonic_clock::time_point monotonic_start_time =
                       aos::monotonic_clock::min_time,
                   aos::realtime_clock::time_point realtime_start_time =
                       aos::realtime_clock::min_time);

  // Makes a template header for all the follower nodes.
  aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader> MakeHeader(
      std::string_view config_sha256);

  bool MaybeUpdateTimestamp(
      const Node *node, int node_index,
      aos::monotonic_clock::time_point monotonic_start_time,
      aos::realtime_clock::time_point realtime_start_time);

  void DoLogData(const monotonic_clock::time_point end_time,
                 bool run_on_logged);

  void WriteMissingTimestamps();

  void WriteData(NewDataWriter *writer, const FetcherStruct &f);
  void WriteTimestamps(NewDataWriter *timestamps_writer,
                       const FetcherStruct &f);
  void WriteContent(NewDataWriter *contents_writer, const FetcherStruct &f);

  void WriteFetchedRecord(FetcherStruct &f);

  // Fetches from each channel until all the data is logged.  This is dangerous
  // because it lets you log for more than 1 period.  All calls need to verify
  // that t isn't greater than 1 period in the future.
  //
  // Returns true if there is at least one message written, and also returns the
  // timestamp of the newest record that any fetcher is pointing to, or min_time
  // if there are no messages published on any logged channels.
  std::pair<bool, monotonic_clock::time_point> LogUntil(
      monotonic_clock::time_point t);

  void RecordFetchResult(aos::monotonic_clock::time_point start,
                         aos::monotonic_clock::time_point end, bool got_new,
                         FetcherStruct *fetcher);

  void RecordCreateMessageTime(aos::monotonic_clock::time_point start,
                               aos::monotonic_clock::time_point end,
                               const FetcherStruct &fetcher);

  // Write an entry to the profile file.
  void RecordProfileData(aos::monotonic_clock::time_point message_time,
                         aos::monotonic_clock::time_point encoding_start_time,
                         std::chrono::nanoseconds encode_duration,
                         const Channel &channel);

  EventLoop *const event_loop_;
  // The configuration to place at the top of the log file.
  const Configuration *const configuration_;

  // The node that is writing the log.
  // For most cases, this is the same node as the node that is reading the
  // messages. However, in some cases, these two nodes may be different. i.e. if
  // one node reading and modifying the messages, and another node is listening
  // and saving those messages to another log.
  //
  // node_ is a pointer to the writing node, and that node is guaranteed to be
  // in configuration_ which is the configuration being written to the top of
  // the log file.
  const Node *const node_;
  // The node_index_ is the index of the node in configuration_.
  const size_t node_index_;

  UUID log_event_uuid_ = UUID::Zero();
  const UUID logger_instance_uuid_ = UUID::Random();
  std::unique_ptr<LogNamer> log_namer_;
  // Empty indicates there isn't one.
  std::optional<UUID> log_start_uuid_;

  // Name to save in the log file.  Defaults to hostname.
  std::string name_;
  std::string logger_sha1_;
  std::string logger_version_;

  // The callback to get called on each logged period.  See
  // set_on_logged_period() above for more details.
  std::function<void(aos::monotonic_clock::time_point t)> on_logged_period_ =
      [](aos::monotonic_clock::time_point) {};

  std::chrono::nanoseconds max_message_fetch_time_ =
      std::chrono::nanoseconds::zero();
  int max_message_fetch_time_channel_ = -1;
  int max_message_fetch_time_size_ = -1;
  std::chrono::nanoseconds total_message_fetch_time_ =
      std::chrono::nanoseconds::zero();
  int total_message_fetch_count_ = 0;
  int64_t total_message_fetch_bytes_ = 0;

  std::chrono::nanoseconds max_log_delay_ = std::chrono::nanoseconds::zero();
  int max_log_delay_channel_ = -1;

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

  // If true, write the message header into a separate file.
  bool separate_config_ = true;

  // Fetcher for all the statistics from all the nodes.
  aos::Fetcher<message_bridge::ServerStatistics> server_statistics_fetcher_;

  monotonic_clock::time_point log_until_time_ = monotonic_clock::min_time;

  std::function<bool(const Context &)> fetch_next_if_fn_ =
      [this](const Context &context) {
        return context.monotonic_event_time < log_until_time_;
      };

  // Amount of time to run the logger behind now.
  std::chrono::nanoseconds logging_delay_ = std::chrono::nanoseconds(0);

  // Profiling info
  std::optional<ProfileDataWriter> profiling_info_;
};

}  // namespace aos::logger

#endif  // AOS_EVENTS_LOGGING_LOG_WRITER_H_

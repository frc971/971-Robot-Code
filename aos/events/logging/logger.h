#ifndef AOS_EVENTS_LOGGER_H_
#define AOS_EVENTS_LOGGER_H_

#include <chrono>
#include <deque>
#include <string_view>
#include <tuple>
#include <vector>

#include "Eigen/Dense"
#include "absl/strings/str_cat.h"
#include "absl/types/span.h"
#include "aos/events/event_loop.h"
#include "aos/events/logging/eigen_mpq.h"
#include "aos/events/logging/log_namer.h"
#include "aos/events/logging/logfile_sorting.h"
#include "aos/events/logging/logfile_utils.h"
#include "aos/events/logging/logger_generated.h"
#include "aos/events/logging/uuid.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/network/message_bridge_server_generated.h"
#include "aos/network/remote_message_generated.h"
#include "aos/network/timestamp_filter.h"
#include "aos/time/time.h"
#include "flatbuffers/flatbuffers.h"
#include "third_party/gmp/gmpxx.h"

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
    std::string source_node_boot_uuid = "00000000-0000-4000-8000-000000000000";

    aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader> log_file_header =
        aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader>::Empty();

    // True if a header has been written to the start of a log file.
    bool header_written = false;
    // True if the current written header represents the contents which will
    // follow.  This is cleared when boot_uuid is known to not match anymore.
    bool header_valid = false;

    // Sets the source_node_boot_uuid, properly updating everything.
    void SetBootUUID(std::string_view new_source_node_boot_uuid) {
      source_node_boot_uuid = new_source_node_boot_uuid;
      header_valid = false;
      has_source_node_boot_uuid = true;

      flatbuffers::String *source_node_boot_uuid_string =
          log_file_header.mutable_message()->mutable_source_node_boot_uuid();
      CHECK_EQ(source_node_boot_uuid.size(),
               source_node_boot_uuid_string->size());
      memcpy(source_node_boot_uuid_string->data(), source_node_boot_uuid.data(),
             source_node_boot_uuid.size());
    }
  };

  void WriteHeader();

  aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader> MakeHeader(
      const Node *node);

  // Writes the header for the provided node if enough information is valid.
  void MaybeWriteHeader(int node_index);
  // Overload for when we already know node as well.
  void MaybeWriteHeader(int node_index, const Node *node);

  bool MaybeUpdateTimestamp(
      const Node *node, int node_index,
      aos::monotonic_clock::time_point monotonic_start_time,
      aos::realtime_clock::time_point realtime_start_time);

  void DoLogData(const monotonic_clock::time_point end_time);

  void WriteMissingTimestamps();

  // Fetches from each channel until all the data is logged.
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

  // Fetcher for all the statistics from all the nodes.
  aos::Fetcher<message_bridge::ServerStatistics> server_statistics_fetcher_;

  std::vector<NodeState> node_state_;
};

std::vector<std::vector<std::string>> ToLogReaderVector(
    const std::vector<LogFile> &log_files);

// We end up with one of the following 3 log file types.
//
// Single node logged as the source node.
//   -> Replayed just on the source node.
//
// Forwarding timestamps only logged from the perspective of the destination
// node.
//   -> Matched with data on source node and logged.
//
// Forwarding timestamps with data logged as the destination node.
//   -> Replayed just as the destination
//   -> Replayed as the source (Much harder, ordering is not defined)
//
// Duplicate data logged. -> CHECK that it matches and explode otherwise.
//
// This can be boiled down to a set of constraints and tools.
//
// 1) Forwarding timestamps and data need to be logged separately.
// 2) Any forwarded data logged on the destination node needs to be logged
//   separately such that it can be sorted.
//
// 1) Log reader needs to be able to sort a list of log files.
// 2) Log reader needs to be able to merge sorted lists of log files.
// 3) Log reader needs to be able to match timestamps with messages.
//
// We also need to be able to generate multiple views of a log file depending on
// the target.

// Replays all the channels in the logfile to the event loop.
class LogReader {
 public:
  // If you want to supply a new configuration that will be used for replay
  // (e.g., to change message rates, or to populate an updated schema), then
  // pass it in here. It must provide all the channels that the original logged
  // config did.
  //
  // The single file constructor calls SortParts internally.
  LogReader(std::string_view filename,
            const Configuration *replay_configuration = nullptr);
  LogReader(std::vector<LogFile> log_files,
            const Configuration *replay_configuration = nullptr);
  ~LogReader();

  // Registers all the callbacks to send the log file data out on an event loop
  // created in event_loop_factory.  This also updates time to be at the start
  // of the log file by running until the log file starts.
  // Note: the configuration used in the factory should be configuration()
  // below, but can be anything as long as the locations needed to send
  // everything are available.
  void Register(SimulatedEventLoopFactory *event_loop_factory);
  // Creates an SimulatedEventLoopFactory accessible via event_loop_factory(),
  // and then calls Register.
  void Register();
  // Registers callbacks for all the events after the log file starts.  This is
  // only useful when replaying live.
  void Register(EventLoop *event_loop);

  // Unregisters the senders. You only need to call this if you separately
  // supplied an event loop or event loop factory and the lifetimes are such
  // that they need to be explicitly destroyed before the LogReader destructor
  // gets called.
  void Deregister();

  // Returns the configuration being used for replay from the log file.
  // Note that this may be different from the configuration actually used for
  // handling events. You should generally only use this to create a
  // SimulatedEventLoopFactory, and then get the configuration from there for
  // everything else.
  const Configuration *logged_configuration() const;
  // Returns the configuration being used for replay from the log file.
  // Note that this may be different from the configuration actually used for
  // handling events. You should generally only use this to create a
  // SimulatedEventLoopFactory, and then get the configuration from there for
  // everything else.
  // The pointer is invalidated whenever RemapLoggedChannel is called.
  const Configuration *configuration() const;

  // Returns the nodes that this log file was created on.  This is a list of
  // pointers to a node in the nodes() list inside configuration().  The
  // pointers here are invalidated whenever RemapLoggedChannel is called.
  std::vector<const Node *> Nodes() const;

  // Returns the starting timestamp for the log file.
  monotonic_clock::time_point monotonic_start_time(
      const Node *node = nullptr) const;
  realtime_clock::time_point realtime_start_time(
      const Node *node = nullptr) const;

  // Causes the logger to publish the provided channel on a different name so
  // that replayed applications can publish on the proper channel name without
  // interference. This operates on raw channel names, without any node or
  // application specific mappings.
  void RemapLoggedChannel(std::string_view name, std::string_view type,
                          std::string_view add_prefix = "/original",
                          std::string_view new_type = "");
  template <typename T>
  void RemapLoggedChannel(std::string_view name,
                          std::string_view add_prefix = "/original",
                          std::string_view new_type = "") {
    RemapLoggedChannel(name, T::GetFullyQualifiedName(), add_prefix, new_type);
  }

  // Remaps the provided channel, though this respects node mappings, and
  // preserves them too.  This makes it so if /aos -> /pi1/aos on one node,
  // /original/aos -> /original/pi1/aos on the same node after renaming, just
  // like you would hope.  If new_type is not empty, the new channel will use
  // the provided type instead.  This allows for renaming messages.
  //
  // TODO(austin): If you have 2 nodes remapping something to the same channel,
  // this doesn't handle that.  No use cases exist yet for that, so it isn't
  // being done yet.
  void RemapLoggedChannel(std::string_view name, std::string_view type,
                          const Node *node,
                          std::string_view add_prefix = "/original",
                          std::string_view new_type = "");
  template <typename T>
  void RemapLoggedChannel(std::string_view name, const Node *node,
                          std::string_view add_prefix = "/original",
                          std::string_view new_type = "") {
    RemapLoggedChannel(name, T::GetFullyQualifiedName(), node, add_prefix,
                       new_type);
  }

  template <typename T>
  bool HasChannel(std::string_view name, const Node *node = nullptr) {
    return configuration::GetChannel(log_file_header()->configuration(), name,
                                     T::GetFullyQualifiedName(), "", node,
                                     true) != nullptr;
  }

  SimulatedEventLoopFactory *event_loop_factory() {
    return event_loop_factory_;
  }

  const LogFileHeader *log_file_header() const {
    return &log_file_header_.message();
  }

  std::string_view name() const {
    return log_file_header()->name()->string_view();
  }

  // Set whether to exit the SimulatedEventLoopFactory when we finish reading
  // the logfile.
  void set_exit_on_finish(bool exit_on_finish) {
    exit_on_finish_ = exit_on_finish;
  }

 private:
  const Channel *RemapChannel(const EventLoop *event_loop,
                              const Channel *channel);

  // Queues at least max_out_of_order_duration_ messages into channels_.
  void QueueMessages();
  // Handle constructing a configuration with all the additional remapped
  // channels from calls to RemapLoggedChannel.
  void MakeRemappedConfig();

  // Returns the number of nodes.
  size_t nodes_count() const {
    return !configuration::MultiNode(logged_configuration())
               ? 1u
               : logged_configuration()->nodes()->size();
  }

  const std::vector<LogFile> log_files_;

  // This is *a* log file header used to provide the logged config.  The rest of
  // the header is likely distracting.
  SizePrefixedFlatbufferVector<LogFileHeader> log_file_header_;

  // Returns [ta; tb; ...] = tuple[0] * t + tuple[1]
  std::tuple<Eigen::Matrix<double, Eigen::Dynamic, 1>,
             Eigen::Matrix<double, Eigen::Dynamic, 1>>
  SolveOffsets();

  void LogFit(std::string_view prefix);

  // State per node.
  class State {
   public:
    State(std::unique_ptr<TimestampMapper> timestamp_mapper);

    // Connects up the timestamp mappers.
    void AddPeer(State *peer);

    // Returns the timestamps, channel_index, and message from a channel.
    // update_time (will be) set to true when popping this message causes the
    // filter to change the time offset estimation function.
    TimestampedMessage PopOldest(bool *update_time);

    // Returns the monotonic time of the oldest message.
    monotonic_clock::time_point OldestMessageTime() const;

    // Primes the queues inside State.  Should be called before calling
    // OldestMessageTime.
    void SeedSortedMessages();

    // Returns the starting time for this node.
    monotonic_clock::time_point monotonic_start_time() const {
      return timestamp_mapper_ ? timestamp_mapper_->monotonic_start_time()
                               : monotonic_clock::min_time;
    }
    realtime_clock::time_point realtime_start_time() const {
      return timestamp_mapper_ ? timestamp_mapper_->realtime_start_time()
                               : realtime_clock::min_time;
    }

    // Sets the node event loop factory for replaying into a
    // SimulatedEventLoopFactory.  Returns the EventLoop to use.
    EventLoop *SetNodeEventLoopFactory(
        NodeEventLoopFactory *node_event_loop_factory);

    // Sets and gets the event loop to use.
    void set_event_loop(EventLoop *event_loop) { event_loop_ = event_loop; }
    EventLoop *event_loop() { return event_loop_; }

    // Sets the current realtime offset from the monotonic clock for this node
    // (if we are on a simulated event loop).
    void SetRealtimeOffset(monotonic_clock::time_point monotonic_time,
                           realtime_clock::time_point realtime_time) {
      if (node_event_loop_factory_ != nullptr) {
        node_event_loop_factory_->SetRealtimeOffset(monotonic_time,
                                                    realtime_time);
      }
    }

    // Returns the MessageHeader sender to log delivery timestamps to for the
    // provided remote node.
    aos::Sender<message_bridge::RemoteMessage> *RemoteTimestampSender(
        const Node *delivered_node);

    // Converts a timestamp from the monotonic clock on this node to the
    // distributed clock.
    distributed_clock::time_point ToDistributedClock(
        monotonic_clock::time_point time) {
      return node_event_loop_factory_->ToDistributedClock(time);
    }

    monotonic_clock::time_point FromDistributedClock(
        distributed_clock::time_point time) {
      return node_event_loop_factory_->FromDistributedClock(time);
    }

    // Sets the offset (and slope) from the distributed clock.
    void SetDistributedOffset(std::chrono::nanoseconds distributed_offset,
                              double distributed_slope) {
      node_event_loop_factory_->SetDistributedOffset(distributed_offset,
                                                     distributed_slope);
    }

    // Returns the current time on the remote node which sends messages on
    // channel_index.
    monotonic_clock::time_point monotonic_remote_now(size_t channel_index) {
      return channel_source_state_[channel_index]
          ->node_event_loop_factory_->monotonic_now();
    }

    distributed_clock::time_point RemoteToDistributedClock(
        size_t channel_index, monotonic_clock::time_point time) {
      return channel_source_state_[channel_index]
          ->node_event_loop_factory_->ToDistributedClock(time);
    }

    const Node *remote_node(size_t channel_index) {
      return channel_source_state_[channel_index]
          ->node_event_loop_factory_->node();
    }

    monotonic_clock::time_point monotonic_now() {
      return node_event_loop_factory_->monotonic_now();
    }

    // Sets the number of channels.
    void SetChannelCount(size_t count);

    // Sets the sender, filter, and target factory for a channel.
    void SetChannel(
        size_t logged_channel_index, size_t factory_channel_index,
        std::unique_ptr<RawSender> sender,
        message_bridge::NoncausalOffsetEstimator *filter,
        aos::Sender<message_bridge::RemoteMessage> *remote_timestamp_sender,
        State *source_state);

    // Returns if we have read all the messages from all the logs.
    bool at_end() const {
      return timestamp_mapper_ ? timestamp_mapper_->Front() == nullptr : true;
    }

    // Unregisters everything so we can destory the event loop.
    void Deregister();

    // Sets the current TimerHandle for the replay callback.
    void set_timer_handler(TimerHandler *timer_handler) {
      timer_handler_ = timer_handler;
    }

    // Sets the next wakeup time on the replay callback.
    void Setup(monotonic_clock::time_point next_time) {
      timer_handler_->Setup(next_time);
    }

    // Sends a buffer on the provided channel index.
    bool Send(const TimestampedMessage &timestamped_message);

    // Returns a debug string for the channel merger.
    std::string DebugString() const {
      std::stringstream messages;
      size_t i = 0;
      for (const auto &message : sorted_messages_) {
        if (i < 7 || i + 7 > sorted_messages_.size()) {
          messages << "sorted_messages[" << i
                   << "]: " << std::get<0>(message).monotonic_event_time << " "
                   << configuration::StrippedChannelToString(
                          event_loop_->configuration()->channels()->Get(
                              std::get<0>(message).channel_index))
                   << "\n";
        } else if (i == 7) {
          messages << "...\n";
        }
        ++i;
      }
      if (!timestamp_mapper_) {
        return messages.str();
      }
      return messages.str() + timestamp_mapper_->DebugString();
    }

   private:
    // Log file.
    std::unique_ptr<TimestampMapper> timestamp_mapper_;

    std::deque<std::tuple<TimestampedMessage,
                          message_bridge::NoncausalOffsetEstimator *>>
        sorted_messages_;

    // Senders.
    std::vector<std::unique_ptr<RawSender>> channels_;
    std::vector<aos::Sender<message_bridge::RemoteMessage> *>
        remote_timestamp_senders_;
    // The mapping from logged channel index to sent channel index.  Needed for
    // sending out MessageHeaders.
    std::vector<int> factory_channel_index_;

    struct SentTimestamp {
      monotonic_clock::time_point monotonic_event_time =
          monotonic_clock::min_time;
      realtime_clock::time_point realtime_event_time = realtime_clock::min_time;
      uint32_t queue_index = 0xffffffff;

      // The queue index that this message *actually* was sent with.
      uint32_t actual_queue_index = 0xffffffff;
    };

    // Stores all the timestamps that have been sent on this channel.  This is
    // only done for channels which are forwarded and on the node which
    // initially sends the message.
    //
    // TODO(austin): This whole concept is a hack.  We should be able to
    // associate state with the message as it gets sorted and recover it.
    std::vector<std::unique_ptr<std::vector<SentTimestamp>>> queue_index_map_;

    // Factory (if we are in sim) that this loop was created on.
    NodeEventLoopFactory *node_event_loop_factory_ = nullptr;
    std::unique_ptr<EventLoop> event_loop_unique_ptr_;
    // Event loop.
    EventLoop *event_loop_ = nullptr;
    // And timer used to send messages.
    TimerHandler *timer_handler_;

    // Filters (or nullptr if it isn't a forwarded channel) for each channel.
    // This corresponds to the object which is shared among all the channels
    // going between 2 nodes.  The second element in the tuple indicates if this
    // is the primary direction or not.
    std::vector<message_bridge::NoncausalOffsetEstimator *> filters_;

    // List of NodeEventLoopFactorys (or nullptr if it isn't a forwarded
    // channel) which correspond to the originating node.
    std::vector<State *> channel_source_state_;

    std::map<const Node *, aos::Sender<message_bridge::RemoteMessage>>
        remote_timestamp_senders_map_;
  };

  // Node index -> State.
  std::vector<std::unique_ptr<State>> states_;

  // Creates the requested filter if it doesn't exist, regardless of whether
  // these nodes can actually communicate directly.  The second return value
  // reports if this is the primary direction or not.
  message_bridge::NoncausalOffsetEstimator *GetFilter(const Node *node_a,
                                                      const Node *node_b);

  // FILE to write offsets to (if populated).
  FILE *offset_fp_ = nullptr;
  // Timestamp of the first piece of data used for the horizontal axis on the
  // plot.
  aos::realtime_clock::time_point first_time_;

  // List of filters for a connection.  The pointer to the first node will be
  // less than the second node.
  std::map<std::tuple<const Node *, const Node *>,
           std::tuple<message_bridge::NoncausalOffsetEstimator>>
      filters_;

  // Returns the offset from the monotonic clock for a node to the distributed
  // clock.  monotonic = distributed * slope() + offset();
  double slope(int node_index) const {
    CHECK_LT(node_index, time_slope_matrix_.rows())
        << ": Got too high of a node index.";
    return time_slope_matrix_(node_index);
  }
  std::chrono::nanoseconds offset(int node_index) const {
    CHECK_LT(node_index, time_offset_matrix_.rows())
        << ": Got too high of a node index.";
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(time_offset_matrix_(node_index)));
  }

  // Updates the offset matrix solution and sets the per-node distributed
  // offsets in the factory.
  void UpdateOffsets();

  // We have 2 types of equations to do a least squares regression over to fully
  // constrain our time function.
  //
  // One is simple.  The distributed clock is the average of all the clocks.
  //   (ta + tb + tc + td) / num_nodes = t_distributed
  //
  // The second is a bit more complicated.  Our basic time conversion function
  // is:
  //   tb = ta + (ta * slope + offset)
  // We can rewrite this as follows
  //   tb - (1 + slope) * ta = offset
  //
  // From here, we have enough equations to solve for t{a,b,c,...}  We want to
  // take as an input the offsets and slope, and solve for the per-node times as
  // a function of the distributed clock.
  //
  // We need to massage our equations to make this work.  If we solve for the
  // per-node times at two set distributed clock times, we will be able to
  // recreate the linear function (we know it is linear).  We can do a similar
  // thing by breaking our equation up into:
  //
  // [1/3  1/3  1/3  ] [ta]   [t_distributed]
  // [ 1  -1-m1  0   ] [tb] = [oab]
  // [ 1    0  -1-m2 ] [tc]   [oac]
  //
  // This solves to:
  //
  // [ta]   [ a00 a01 a02]   [t_distributed]
  // [tb] = [ a10 a11 a12] * [oab]
  // [tc]   [ a20 a21 a22]   [oac]
  //
  // and can be split into:
  //
  // [ta]   [ a00 ]                   [a01 a02]
  // [tb] = [ a10 ] * t_distributed + [a11 a12] * [oab]
  // [tc]   [ a20 ]                   [a21 a22]   [oac]
  //
  // (map_matrix_ + slope_matrix_) * [ta; tb; tc] = [offset_matrix_];
  // offset_matrix_ will be in nanoseconds.
  Eigen::Matrix<mpq_class, Eigen::Dynamic, Eigen::Dynamic> map_matrix_;
  Eigen::Matrix<mpq_class, Eigen::Dynamic, Eigen::Dynamic> slope_matrix_;
  Eigen::Matrix<mpq_class, Eigen::Dynamic, 1> offset_matrix_;
  // Matrix tracking which offsets are valid.
  Eigen::Matrix<bool, Eigen::Dynamic, 1> valid_matrix_;
  // Matrix tracking the last valid matrix we used to determine connected nodes.
  Eigen::Matrix<bool, Eigen::Dynamic, 1> last_valid_matrix_;
  size_t cached_valid_node_count_ = 0;

  // [ta; tb; tc] = time_slope_matrix_ * t + time_offset_matrix;
  // t is in seconds.
  Eigen::Matrix<double, Eigen::Dynamic, 1> time_slope_matrix_;
  Eigen::Matrix<double, Eigen::Dynamic, 1> time_offset_matrix_;

  std::unique_ptr<FlatbufferDetachedBuffer<Configuration>>
      remapped_configuration_buffer_;

  std::unique_ptr<SimulatedEventLoopFactory> event_loop_factory_unique_ptr_;
  SimulatedEventLoopFactory *event_loop_factory_ = nullptr;

  // Map of channel indices to new name. The channel index will be an index into
  // logged_configuration(), and the string key will be the name of the channel
  // to send on instead of the logged channel name.
  struct RemappedChannel {
    std::string remapped_name;
    std::string new_type;
  };
  std::map<size_t, RemappedChannel> remapped_channels_;
  std::vector<MapT> maps_;

  // Number of nodes which still have data to send.  This is used to figure out
  // when to exit.
  size_t live_nodes_ = 0;

  const Configuration *remapped_configuration_ = nullptr;
  const Configuration *replay_configuration_ = nullptr;

  // If true, the replay timer will ignore any missing data.  This is used
  // during startup when we are bootstrapping everything and trying to get to
  // the start of all the log files.
  bool ignore_missing_data_ = false;

  // Whether to exit the SimulatedEventLoop when we finish reading the logs.
  bool exit_on_finish_ = true;
};

}  // namespace logger
}  // namespace aos

#endif  // AOS_EVENTS_LOGGER_H_

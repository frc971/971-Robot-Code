#ifndef AOS_EVENTS_LOGGING_LOG_READER_H_
#define AOS_EVENTS_LOGGING_LOG_READER_H_

#include <chrono>
#include <deque>
#include <string_view>
#include <tuple>
#include <vector>

#include "aos/events/event_loop.h"
#include "aos/events/logging/logfile_sorting.h"
#include "aos/events/logging/logfile_utils.h"
#include "aos/events/logging/logger_generated.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/network/message_bridge_server_generated.h"
#include "aos/network/multinode_timestamp_filter.h"
#include "aos/network/remote_message_generated.h"
#include "aos/network/timestamp_filter.h"
#include "aos/time/time.h"
#include "aos/uuid.h"
#include "flatbuffers/flatbuffers.h"

namespace aos {
namespace logger {

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
  // Registers all the callbacks to send the log file data out to an event loop
  // factory.  This does not start replaying or change the current distributed
  // time of the factory.  It does change the monotonic clocks to be right.
  void RegisterWithoutStarting(SimulatedEventLoopFactory *event_loop_factory);
  // Creates an SimulatedEventLoopFactory accessible via event_loop_factory(),
  // and then calls Register.
  void Register();
  // Registers callbacks for all the events after the log file starts.  This is
  // only useful when replaying live.
  void Register(EventLoop *event_loop);

  // Called whenever a log file starts for a node.
  void OnStart(std::function<void()> fn);
  void OnStart(const Node *node, std::function<void()> fn);
  // Called whenever a log file ends for a node.
  void OnEnd(std::function<void()> fn);
  void OnEnd(const Node *node, std::function<void()> fn);

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
  // pointers to a node in the nodes() list inside logged_configuration().
  std::vector<const Node *> LoggedNodes() const;

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
    return configuration::GetChannel(logged_configuration(), name,
                                     T::GetFullyQualifiedName(), "", node,
                                     true) != nullptr;
  }

  // Returns true if the channel exists on the node and was logged.
  template <typename T>
  bool HasLoggedChannel(std::string_view name, const Node *node = nullptr) {
    const Channel *channel =
        configuration::GetChannel(logged_configuration(), name,
                                  T::GetFullyQualifiedName(), "", node, true);
    if (channel == nullptr) return false;
    return channel->logger() != LoggerConfig::NOT_LOGGED;
  }

  SimulatedEventLoopFactory *event_loop_factory() {
    return event_loop_factory_;
  }

  std::string_view name() const { return log_files_[0].name; }

  // Set whether to exit the SimulatedEventLoopFactory when we finish reading
  // the logfile.
  void set_exit_on_finish(bool exit_on_finish) {
    exit_on_finish_ = exit_on_finish;
  }

 private:
  void Register(EventLoop *event_loop, const Node *node);

  void RegisterDuringStartup(EventLoop *event_loop, const Node *node);

  const Channel *RemapChannel(const EventLoop *event_loop, const Node *node,
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

  // Class to manage sending RemoteMessages on the provided node after the
  // correct delay.
  class RemoteMessageSender {
   public:
    RemoteMessageSender(aos::Sender<message_bridge::RemoteMessage> sender,
                        EventLoop *event_loop);
    RemoteMessageSender(RemoteMessageSender const &) = delete;
    RemoteMessageSender &operator=(RemoteMessageSender const &) = delete;

    // Sends the provided message.  If monotonic_timestamp_time is min_time,
    // send it immediately.
    void Send(
        FlatbufferDetachedBuffer<message_bridge::RemoteMessage> remote_message,
        BootTimestamp monotonic_timestamp_time, size_t source_boot_count);

   private:
    // Handles actually sending the timestamp if we were delayed.
    void SendTimestamp();
    // Handles scheduling the timer to send at the correct time.
    void ScheduleTimestamp();

    EventLoop *event_loop_;
    aos::Sender<message_bridge::RemoteMessage> sender_;
    aos::TimerHandler *timer_;

    // Time we are scheduled for, or min_time if we aren't scheduled.
    monotonic_clock::time_point scheduled_time_ = monotonic_clock::min_time;

    struct Timestamp {
      Timestamp(FlatbufferDetachedBuffer<message_bridge::RemoteMessage>
                    new_remote_message,
                monotonic_clock::time_point new_monotonic_timestamp_time)
          : remote_message(std::move(new_remote_message)),
            monotonic_timestamp_time(new_monotonic_timestamp_time) {}
      FlatbufferDetachedBuffer<message_bridge::RemoteMessage> remote_message;
      monotonic_clock::time_point monotonic_timestamp_time;
    };

    // List of messages to send. The timer works through them and then disables
    // itself automatically.
    std::deque<Timestamp> remote_timestamps_;
  };

  // State per node.
  class State {
   public:
    State(std::unique_ptr<TimestampMapper> timestamp_mapper, const Node *node);

    // Connects up the timestamp mappers.
    void AddPeer(State *peer);

    TimestampMapper *timestamp_mapper() { return timestamp_mapper_.get(); }

    // Returns the next sorted message with all the timestamps extracted and
    // matched.
    TimestampedMessage PopOldest();

    // Returns the monotonic time of the oldest message.
    BootTimestamp OldestMessageTime() const;

    size_t boot_count() const {
      // If we are replaying directly into an event loop, we can't reboot.  So
      // we will stay stuck on the 0th boot.
      if (!node_event_loop_factory_) return 0u;
      return node_event_loop_factory_->boot_count();
    }

    // Primes the queues inside State.  Should be called before calling
    // OldestMessageTime.
    void SeedSortedMessages();

    void SetupStartupTimer() {
      const monotonic_clock::time_point start_time =
          monotonic_start_time(boot_count());
      if (start_time == monotonic_clock::min_time) {
        LOG(ERROR)
            << "No start time, skipping, please figure out when this happens";
        RunOnStart();
        return;
      }
      CHECK_GT(start_time, event_loop_->monotonic_now());
      startup_timer_->Setup(start_time);
    }

    void set_startup_timer(TimerHandler *timer_handler) {
      startup_timer_ = timer_handler;
      if (startup_timer_) {
        if (event_loop_->node() != nullptr) {
          startup_timer_->set_name(absl::StrCat(
              event_loop_->node()->name()->string_view(), "_startup"));
        } else {
          startup_timer_->set_name("startup");
        }
      }
    }

    // Returns the starting time for this node.
    monotonic_clock::time_point monotonic_start_time(size_t boot_count) const {
      return timestamp_mapper_
                 ? timestamp_mapper_->monotonic_start_time(boot_count)
                 : monotonic_clock::min_time;
    }
    realtime_clock::time_point realtime_start_time(size_t boot_count) const {
      return timestamp_mapper_
                 ? timestamp_mapper_->realtime_start_time(boot_count)
                 : realtime_clock::min_time;
    }

    // Sets the node event loop factory for replaying into a
    // SimulatedEventLoopFactory.  Returns the EventLoop to use.
    void SetNodeEventLoopFactory(NodeEventLoopFactory *node_event_loop_factory);

    // Sets and gets the event loop to use.
    void set_event_loop(EventLoop *event_loop) { event_loop_ = event_loop; }
    EventLoop *event_loop() { return event_loop_; }

    const Node *node() const { return node_; }

    void Register(EventLoop *event_loop);

    void OnStart(std::function<void()> fn);
    void OnEnd(std::function<void()> fn);

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
    RemoteMessageSender *RemoteTimestampSender(const Channel *channel,
                                               const Connection *connection);

    // Converts a timestamp from the monotonic clock on this node to the
    // distributed clock.
    distributed_clock::time_point ToDistributedClock(
        monotonic_clock::time_point time) {
      return node_event_loop_factory_->ToDistributedClock(time);
    }

    // Returns the current time on the remote node which sends messages on
    // channel_index.
    BootTimestamp monotonic_remote_now(size_t channel_index) {
      State *s = channel_source_state_[channel_index];
      return BootTimestamp{
          .boot = s->boot_count(),
          .time = s->node_event_loop_factory_->monotonic_now()};
    }

    // Returns the start time of the remote for the provided channel.
    monotonic_clock::time_point monotonic_remote_start_time(
        size_t boot_count, size_t channel_index) {
      return channel_source_state_[channel_index]->monotonic_start_time(
          boot_count);
    }

    void DestroyEventLoop() { event_loop_unique_ptr_.reset(); }

    EventLoop *MakeEventLoop() {
      CHECK(!event_loop_unique_ptr_);
      event_loop_unique_ptr_ =
          node_event_loop_factory_->MakeEventLoop("log_reader");
      return event_loop_unique_ptr_.get();
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
    void SetChannel(size_t logged_channel_index, size_t factory_channel_index,
                    std::unique_ptr<RawSender> sender,
                    message_bridge::NoncausalOffsetEstimator *filter,
                    bool is_forwarded, State *source_state);

    void SetRemoteTimestampSender(size_t logged_channel_index,
                                  RemoteMessageSender *remote_timestamp_sender);

    void RunOnStart();
    void RunOnEnd();

    // Unregisters everything so we can destory the event loop.
    // TODO(austin): Is this needed?  OnShutdown should be able to serve this
    // need.
    void Deregister();

    // Sets the current TimerHandle for the replay callback.
    void set_timer_handler(TimerHandler *timer_handler) {
      timer_handler_ = timer_handler;
      if (timer_handler_) {
        if (event_loop_->node() != nullptr) {
          timer_handler_->set_name(absl::StrCat(
              event_loop_->node()->name()->string_view(), "_main"));
        } else {
          timer_handler_->set_name("main");
        }
      }
    }

    // Sets the next wakeup time on the replay callback.
    void Setup(monotonic_clock::time_point next_time) {
      timer_handler_->Setup(next_time);
    }

    // Sends a buffer on the provided channel index.
    bool Send(const TimestampedMessage &timestamped_message);

    // Returns a debug string for the channel merger.
    std::string DebugString() const {
      if (!timestamp_mapper_) {
        return "";
      }
      return timestamp_mapper_->DebugString();
    }

    void ClearRemoteTimestampSenders() {
      channel_timestamp_loggers_.clear();
      timestamp_loggers_.clear();
    }

   private:
    // Log file.
    std::unique_ptr<TimestampMapper> timestamp_mapper_;

    // Senders.
    std::vector<std::unique_ptr<RawSender>> channels_;
    std::vector<RemoteMessageSender *> remote_timestamp_senders_;
    // The mapping from logged channel index to sent channel index.  Needed for
    // sending out MessageHeaders.
    std::vector<int> factory_channel_index_;

    struct ContiguousSentTimestamp {
      // Most timestamps make it through the network, so it saves a ton of
      // memory and CPU to store the start and end, and search for valid ranges.
      // For one of the logs I looked at, we had 2 ranges for 4 days.
      //
      // Save monotonic times as well to help if a queue index ever wraps.  Odds
      // are very low, but doesn't hurt.
      //
      // The starting time and matching queue index.
      monotonic_clock::time_point starting_monotonic_event_time =
          monotonic_clock::min_time;
      uint32_t starting_queue_index = 0xffffffff;

      // Ending time and queue index.
      monotonic_clock::time_point ending_monotonic_event_time =
          monotonic_clock::max_time;
      uint32_t ending_queue_index = 0xffffffff;

      // The queue index that the first message was *actually* sent with.  The
      // queue indices are assumed to be contiguous through this range.
      uint32_t actual_queue_index = 0xffffffff;
    };

    // Stores all the timestamps that have been sent on this channel.  This is
    // only done for channels which are forwarded and on the node which
    // initially sends the message.  Compress using ranges and offsets.
    std::vector<std::unique_ptr<std::vector<ContiguousSentTimestamp>>>
        queue_index_map_;

    // Factory (if we are in sim) that this loop was created on.
    NodeEventLoopFactory *node_event_loop_factory_ = nullptr;
    std::unique_ptr<EventLoop> event_loop_unique_ptr_;
    // Event loop.
    const Node *node_ = nullptr;
    EventLoop *event_loop_ = nullptr;
    // And timer used to send messages.
    TimerHandler *timer_handler_ = nullptr;
    TimerHandler *startup_timer_ = nullptr;

    // Filters (or nullptr if it isn't a forwarded channel) for each channel.
    // This corresponds to the object which is shared among all the channels
    // going between 2 nodes.  The second element in the tuple indicates if this
    // is the primary direction or not.
    std::vector<message_bridge::NoncausalOffsetEstimator *> filters_;

    // List of NodeEventLoopFactorys (or nullptr if it isn't a forwarded
    // channel) which correspond to the originating node.
    std::vector<State *> channel_source_state_;

    // This is a cache for channel, connection mapping to the corresponding
    // sender.
    absl::btree_map<std::pair<const Channel *, const Connection *>,
                    std::shared_ptr<RemoteMessageSender>>
        channel_timestamp_loggers_;

    // Mapping from resolved RemoteMessage channel to RemoteMessage sender. This
    // is the channel that timestamps are published to.
    absl::btree_map<const Channel *, std::shared_ptr<RemoteMessageSender>>
        timestamp_loggers_;

    std::vector<std::function<void()>> on_starts_;
    std::vector<std::function<void()>> on_ends_;

    bool stopped_ = false;
    bool started_ = false;
  };

  // Node index -> State.
  std::vector<std::unique_ptr<State>> states_;

  // Creates the requested filter if it doesn't exist, regardless of whether
  // these nodes can actually communicate directly.  The second return value
  // reports if this is the primary direction or not.
  message_bridge::NoncausalOffsetEstimator *GetFilter(const Node *node_a,
                                                      const Node *node_b);

  // List of filters for a connection.  The pointer to the first node will be
  // less than the second node.
  std::unique_ptr<message_bridge::MultiNodeNoncausalOffsetEstimator> filters_;

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

#endif  // AOS_EVENTS_LOGGING_LOG_READER_H_

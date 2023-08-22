#ifndef AOS_EVENTS_LOGGING_LOG_READER_H_
#define AOS_EVENTS_LOGGING_LOG_READER_H_

#include <chrono>
#include <deque>
#include <queue>
#include <string_view>
#include <tuple>
#include <vector>

#include "flatbuffers/flatbuffers.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

#include "aos/condition.h"
#include "aos/events/event_loop.h"
#include "aos/events/event_loop_tmpl.h"
#include "aos/events/logging/config_remapper.h"
#include "aos/events/logging/logfile_sorting.h"
#include "aos/events/logging/logfile_utils.h"
#include "aos/events/logging/logger_generated.h"
#include "aos/events/logging/replay_channels.h"
#include "aos/events/logging/replay_timing_generated.h"
#include "aos/events/shm_event_loop.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/mutex/mutex.h"
#include "aos/network/message_bridge_server_generated.h"
#include "aos/network/multinode_timestamp_filter.h"
#include "aos/network/remote_message_generated.h"
#include "aos/network/timestamp_filter.h"
#include "aos/time/time.h"
#include "aos/util/threaded_queue.h"
#include "aos/uuid.h"

namespace aos {
namespace logger {

class EventNotifier;

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
//
// In general, we aim to guarantee that if you are using the LogReader
// "normally" you should be able to observe all the messages that existed on the
// live system between the start time and the end of the logfile, and that
// CHECK-failures will be generated if the LogReader cannot satisfy that
// guarantee. There are currently a few deliberate exceptions to this:
// * Any channel marked NOT_LOGGED in the configuration is known not to
//   have been logged and thus will be silently absent in log replay.
// * If an incomplete set of log files is provided to the reader (e.g.,
//   only logs logged on a single node on a multi-node system), then
//   any *individual* channel as observed on a given node will be
//   consistent, but similarly to a NOT_LOGGED channel, some data may
//   not be available.
// * At the end of a log, data for some channels/nodes may end before
//   others; during this time period, you may observe silently dropped
//   messages. This will be most obvious on uncleanly terminated logs or
//   when merging logfiles across nodes (as the logs on different nodes
//   will not finish at identical times).

// Replays all the channels in the logfile to the event loop.
class LogReader {
 public:
  // If you want to supply a new configuration that will be used for replay
  // (e.g., to change message rates, or to populate an updated schema), then
  // pass it in here. It must provide all the channels that the original logged
  // config did.
  //
  // If certain messages should not be replayed, the replay_channels param can
  // be used as an inclusive list of channels for messages to be replayed.
  //
  // The single file constructor calls SortParts internally.
  LogReader(std::string_view filename,
            const Configuration *replay_configuration = nullptr,
            const ReplayChannels *replay_channels = nullptr);
  LogReader(std::vector<LogFile> log_files,
            const Configuration *replay_configuration = nullptr,
            const ReplayChannels *replay_channels = nullptr);
  LogReader(LogFilesContainer log_files,
            const Configuration *replay_configuration = nullptr,
            const ReplayChannels *replay_channels = nullptr);
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
  // Runs the log until the last start time.  Register above is defined as:
  // Register(...) {
  //   RegisterWithoutStarting
  //   StartAfterRegister
  // }
  // This should generally be considered as a stepping stone to convert from
  // Register() to RegisterWithoutStarting() incrementally.
  void StartAfterRegister(SimulatedEventLoopFactory *event_loop_factory);

  // Creates an SimulatedEventLoopFactory accessible via event_loop_factory(),
  // and then calls Register.
  void Register();

  // Registers callbacks for all the events after the log file starts.  This is
  // only useful when replaying live.
  void Register(EventLoop *event_loop);

  // Sets a sender that should be used for tracking timing statistics. If not
  // set, no statistics will be recorded.
  void set_timing_accuracy_sender(
      const Node *node, aos::Sender<timing::ReplayTiming> timing_sender) {
    states_[configuration::GetNodeIndex(configuration(), node)]
        ->set_timing_accuracy_sender(std::move(timing_sender));
  }

  // Called whenever a log file starts for a node.
  // More precisely, this will be called on each boot at max of
  // (realtime_start_time in the logfiles, SetStartTime()). If a given boot
  // occurs entirely before the realtime_start_time, the OnStart handler will
  // never get called for that boot.
  //
  // realtime_start_time is defined below, but/ essentially is the time at which
  // message channels will start being internall consistent on a given node
  // (i.e., when the logger started). Note: If you wish to see a watcher
  // triggered for *every* message in a log, OnStart() will not be
  // sufficient--messages (possibly multiple messages) may be present on
  // channels prior to the start time. If attempting to do this, prefer to use
  // NodeEventLoopFactory::OnStart.
  void OnStart(std::function<void()> fn);
  void OnStart(const Node *node, std::function<void()> fn);
  // Called whenever a log file ends for a node on a given boot, or at the
  // realtime_end_time specified by a flag or SetEndTime().
  //
  // A log file "ends" when there are no more messages to be replayed for that
  // boot.
  //
  // If OnStart() is not called for a given boot, the OnEnd() handlers will not
  // be called either. OnEnd() handlers will not be called if the logfile for a
  // given boot has missing data that causes us to terminate replay early.
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
  // All logged channels for the specified node should be entirely available
  // after the specified time (i.e., any message that was available on the node
  // in question after the monotonic start time but before the logs end and
  // whose channel is present in any of the provided logs will either be
  // available in the log or will result in an internal CHECK-failure of the
  // LogReader if it would be skipped).
  monotonic_clock::time_point monotonic_start_time(
      const Node *node = nullptr) const;
  realtime_clock::time_point realtime_start_time(
      const Node *node = nullptr) const;

  // Sets the start and end times to replay data until for all nodes.  This
  // overrides the --start_time and --end_time flags.  The default is to replay
  // all data.
  void SetStartTime(std::string start_time);
  void SetStartTime(realtime_clock::time_point start_time);
  void SetEndTime(std::string end_time);
  void SetEndTime(realtime_clock::time_point end_time);

  // Causes the logger to publish the provided channel on a different name so
  // that replayed applications can publish on the proper channel name without
  // interference. This operates on raw channel names, without any node or
  // application specific mappings.
  void RemapLoggedChannel(std::string_view name, std::string_view type,
                          std::string_view add_prefix = "/original",
                          std::string_view new_type = "",
                          ConfigRemapper::RemapConflict conflict_handling =
                              ConfigRemapper::RemapConflict::kCascade);
  template <typename T>
  void RemapLoggedChannel(std::string_view name,
                          std::string_view add_prefix = "/original",
                          std::string_view new_type = "",
                          ConfigRemapper::RemapConflict conflict_handling =
                              ConfigRemapper::RemapConflict::kCascade) {
    RemapLoggedChannel(name, T::GetFullyQualifiedName(), add_prefix, new_type,
                       conflict_handling);
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
                          std::string_view new_type = "",
                          ConfigRemapper::RemapConflict conflict_handling =
                              ConfigRemapper::RemapConflict::kCascade);
  template <typename T>
  void RemapLoggedChannel(std::string_view name, const Node *node,
                          std::string_view add_prefix = "/original",
                          std::string_view new_type = "",
                          ConfigRemapper::RemapConflict conflict_handling =
                              ConfigRemapper::RemapConflict::kCascade) {
    RemapLoggedChannel(name, T::GetFullyQualifiedName(), node, add_prefix,
                       new_type, conflict_handling);
  }

  // Similar to RemapLoggedChannel(), but lets you specify a name for the new
  // channel without constraints. This is useful when an application has been
  // updated to use new channels but you want to support replaying old logs. By
  // default, this will not add any maps for the new channel. Use add_maps to
  // specify any maps you'd like added.
  void RenameLoggedChannel(std::string_view name, std::string_view type,
                           std::string_view new_name,
                           const std::vector<MapT> &add_maps = {});
  template <typename T>
  void RenameLoggedChannel(std::string_view name, std::string_view new_name,
                           const std::vector<MapT> &add_maps = {}) {
    RenameLoggedChannel(name, T::GetFullyQualifiedName(), new_name, add_maps);
  }
  // The following overloads are more suitable for multi-node configurations,
  // and let you rename a channel on a specific node.
  void RenameLoggedChannel(std::string_view name, std::string_view type,
                           const Node *node, std::string_view new_name,
                           const std::vector<MapT> &add_maps = {});
  template <typename T>
  void RenameLoggedChannel(std::string_view name, const Node *node,
                           std::string_view new_name,
                           const std::vector<MapT> &add_maps = {}) {
    RenameLoggedChannel(name, T::GetFullyQualifiedName(), node, new_name,
                        add_maps);
  }

  template <typename T>
  bool HasChannel(std::string_view name, const Node *node = nullptr) {
    return HasChannel(name, T::GetFullyQualifiedName(), node);
  }
  bool HasChannel(std::string_view name, std::string_view type,
                  const Node *node) {
    return configuration::GetChannel(logged_configuration(), name, type, "",
                                     node, true) != nullptr;
  }

  template <typename T>
  void MaybeRemapLoggedChannel(std::string_view name,
                               const Node *node = nullptr) {
    if (HasChannel<T>(name, node)) {
      RemapLoggedChannel<T>(name, node);
    }
  }
  template <typename T>
  void MaybeRenameLoggedChannel(std::string_view name, const Node *node,
                                std::string_view new_name,
                                const std::vector<MapT> &add_maps = {}) {
    if (HasChannel<T>(name, node)) {
      RenameLoggedChannel<T>(name, node, new_name, add_maps);
    }
  }

  // Returns true if the channel exists on the node and was logged.
  template <typename T>
  bool HasLoggedChannel(std::string_view name, const Node *node = nullptr) {
    return config_remapper_.HasOriginalChannel<T>(name, node);
  }

  // Returns a list of all the original channels from remapping.
  std::vector<const Channel *> RemappedChannels() const;

  SimulatedEventLoopFactory *event_loop_factory() {
    return event_loop_factory_;
  }

  std::string_view name() const { return log_files_.name(); }

  // Set whether to exit the SimulatedEventLoopFactory when we finish reading
  // the logfile.
  void set_exit_on_finish(bool exit_on_finish) {
    exit_on_finish_ = exit_on_finish;
  }
  bool exit_on_finish() const { return exit_on_finish_; }

  // Sets the realtime replay rate. A value of 1.0 will cause the scheduler to
  // try to play events in realtime. 0.5 will run at half speed. Use infinity
  // (the default) to run as fast as possible. This can be changed during
  // run-time.
  // Only applies when running against a SimulatedEventLoopFactory.
  void SetRealtimeReplayRate(double replay_rate);

  // Adds a callback for a channel to be called right before sending a message.
  // This allows a user to mutate a message or do any processing when a specific
  // type of message is sent on a channel. The name and type of the channel
  // corresponds to the logged_configuration's name and type.
  //
  // Note, only one callback can be registered per channel in the current
  // implementation. And, the callback is called only once one the Sender's Node
  // if the channel is forwarded.
  //
  // See multinode_logger_test for examples of usage.
  template <typename Callback>
  void AddBeforeSendCallback(std::string_view channel_name,
                             Callback &&callback) {
    CHECK(!AreStatesInitialized())
        << ": Cannot add callbacks after calling Register";

    using MessageType = typename std::remove_pointer<
        typename event_loop_internal::watch_message_type_trait<
            decltype(&Callback::operator())>::message_type>::type;

    const Channel *channel = configuration::GetChannel(
        logged_configuration(), channel_name,
        MessageType::GetFullyQualifiedName(), "", nullptr);

    CHECK(channel != nullptr)
        << ": Channel { \"name\": \"" << channel_name << "\", \"type\": \""
        << MessageType::GetFullyQualifiedName()
        << "\" } not found in config for application.";
    auto channel_index =
        configuration::ChannelIndex(logged_configuration(), channel);

    CHECK(!before_send_callbacks_[channel_index])
        << ": Before Send Callback already registered for channel "
        << ":{ \"name\": \"" << channel_name << "\", \"type\": \""
        << MessageType::GetFullyQualifiedName() << "\" }";

    before_send_callbacks_[channel_index] = [callback](void *message) {
      callback(flatbuffers::GetMutableRoot<MessageType>(
          reinterpret_cast<char *>(message)));
    };
  }

 private:
  void Register(EventLoop *event_loop, const Node *node);

  void RegisterDuringStartup(EventLoop *event_loop, const Node *node);

  const Channel *RemapChannel(const EventLoop *event_loop, const Node *node,
                              const Channel *channel);

  // Queues at least max_out_of_order_duration_ messages into channels_.
  void QueueMessages();

  // Checks if any states have their event loops initialized which indicates
  // events have been scheduled
  void CheckEventsAreNotScheduled();

  // Returns the number of nodes.
  size_t nodes_count() const {
    return !configuration::MultiNode(logged_configuration())
               ? 1u
               : logged_configuration()->nodes()->size();
  }

  // Handles when an individual node hits the realtime end time, exitting the
  // entire event loop once all nodes are stopped.
  void NoticeRealtimeEnd();

  const LogFilesContainer log_files_;

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
    // Whether we should spin up a separate thread for buffering up messages.
    // Only allowed in realtime replay--see comments on threading_ member for
    // details.
    enum class ThreadedBuffering { kYes, kNo };
    State(std::unique_ptr<TimestampMapper> timestamp_mapper,
          TimestampQueueStrategy timestamp_queue_strategy,
          message_bridge::MultiNodeNoncausalOffsetEstimator *multinode_filters,
          std::function<void()> notice_realtime_end, const Node *node,
          ThreadedBuffering threading,
          std::unique_ptr<const ReplayChannelIndices> replay_channel_indices,
          const std::vector<std::function<void(void *message)>>
              &before_send_callbacks);

    // Connects up the timestamp mappers.
    void AddPeer(State *peer);

    TimestampMapper *timestamp_mapper() { return timestamp_mapper_.get(); }

    // Returns the next sorted message with all the timestamps extracted and
    // matched.
    TimestampedMessage PopOldest();

    // Returns the monotonic time of the oldest message.
    BootTimestamp SingleThreadedOldestMessageTime();
    // Returns the monotonic time of the oldest message, handling querying the
    // separate thread of ThreadedBuffering was set.
    BootTimestamp MultiThreadedOldestMessageTime();

    size_t boot_count() const {
      // If we are replaying directly into an event loop, we can't reboot.  So
      // we will stay stuck on the 0th boot.
      if (!node_event_loop_factory_) {
        if (event_loop_ == nullptr) {
          // If boot_count is being checked after startup for any of the
          // non-primary nodes, then returning 0 may not be accurate (since
          // remote nodes *can* reboot even if the EventLoop being played to
          // can't).
          CHECK(!started_);
          CHECK(!stopped_);
        }
        return 0u;
      }
      return node_event_loop_factory_->boot_count();
    }

    // Reads all the timestamps into RAM so we don't need to manage buffering
    // them.  For logs where the timestamps are in separate files, this
    // minimizes RAM usage in the cases where the log reader decides to buffer
    // to the end of the file, or where the time estimation buffer needs to be
    // set high to sort.  This means we devote our RAM to holding lots of
    // timestamps instead of timestamps and much larger data for a shorter
    // period.  For logs where timestamps are stored with the data, this
    // triggers those files to be read twice.
    void ReadTimestamps();

    // Primes the queues inside State.  Should be called before calling
    // OldestMessageTime.
    void MaybeSeedSortedMessages();

    void SetUpStartupTimer() {
      const monotonic_clock::time_point start_time =
          monotonic_start_time(boot_count());
      if (start_time == monotonic_clock::min_time) {
        if (event_loop_->node()) {
          LOG(ERROR) << "No start time for "
                     << event_loop_->node()->name()->string_view()
                     << ", skipping.";
        } else {
          LOG(ERROR) << "No start time, skipping.";
        }

        // This is called from OnRun. There is too much complexity in supporting
        // OnStartup callbacks from inside OnRun.  Instead, schedule a timer for
        // "now", and have that do what we need.
        startup_timer_->Schedule(event_loop_->monotonic_now());
        return;
      }
      if (node_event_loop_factory_) {
        CHECK_GE(start_time + clock_offset(), event_loop_->monotonic_now());
      }
      startup_timer_->Schedule(start_time + clock_offset());
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
    void SetNodeEventLoopFactory(NodeEventLoopFactory *node_event_loop_factory,
                                 SimulatedEventLoopFactory *event_loop_factory);

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
      CHECK(node_event_loop_factory_);
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
      // TODO(james): Enable exclusive senders on LogReader to allow us to
      // ensure we are remapping channels correctly.
      event_loop_unique_ptr_ = node_event_loop_factory_->MakeEventLoop(
          "log_reader", {NodeEventLoopFactory::CheckSentTooFast::kNo,
                         NodeEventLoopFactory::ExclusiveSenders::kYes,
                         NonExclusiveChannels()});
      return event_loop_unique_ptr_.get();
    }

    distributed_clock::time_point RemoteToDistributedClock(
        size_t channel_index, monotonic_clock::time_point time) {
      CHECK(node_event_loop_factory_);
      return channel_source_state_[channel_index]
          ->node_event_loop_factory_->ToDistributedClock(time);
    }

    const Node *remote_node(size_t channel_index) {
      return channel_source_state_[channel_index]
          ->node_event_loop_factory_->node();
    }

    monotonic_clock::time_point monotonic_now() const {
      CHECK_NOTNULL(event_loop_);
      return event_loop_->monotonic_now();
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

    // Handles a logfile start event to potentially call the OnStart callbacks.
    void NotifyLogfileStart();
    // Handles a start time flag start event to potentially call the OnStart
    // callbacks.
    void NotifyFlagStart();

    // Handles a logfile end event to potentially call the OnEnd callbacks.
    void NotifyLogfileEnd();
    // Handles a end time flag start event to potentially call the OnEnd
    // callbacks.
    void NotifyFlagEnd();

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

    // Creates and registers the --start_time and --end_time event callbacks.
    void SetStartTimeFlag(realtime_clock::time_point start_time);
    void SetEndTimeFlag(realtime_clock::time_point end_time);

    // Notices the next message to update the start/end time callbacks.
    void ObserveNextMessage(monotonic_clock::time_point monotonic_event,
                            realtime_clock::time_point realtime_event);

    // Clears the start and end time flag handlers so we can delete the event
    // loop.
    void ClearTimeFlags();

    // Sets the next wakeup time on the replay callback.
    void Schedule(monotonic_clock::time_point next_time) {
      timer_handler_->Schedule(
          std::max(monotonic_now(), next_time + clock_offset()));
    }

    // Sends a buffer on the provided channel index.
    bool Send(const TimestampedMessage &&timestamped_message);

    void MaybeSetClockOffset();
    std::chrono::nanoseconds clock_offset() const { return clock_offset_; }

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

    void SetFoundLastMessage(bool val) {
      found_last_message_ = val;
      last_message_.resize(factory_channel_index_.size(), false);
    }
    bool found_last_message() const { return found_last_message_; }

    void set_last_message(size_t channel_index) {
      CHECK_LT(channel_index, last_message_.size());
      last_message_[channel_index] = true;
    }

    bool last_message(size_t channel_index) {
      CHECK_LT(channel_index, last_message_.size());
      return last_message_[channel_index];
    }

    void set_timing_accuracy_sender(
        aos::Sender<timing::ReplayTiming> timing_sender) {
      timing_statistics_sender_ = std::move(timing_sender);
      OnEnd([this]() { SendMessageTimings(); });
    }

    // If running with ThreadedBuffering::kYes, will start the processing thread
    // and queue up messages until the specified time. No-op of
    // ThreadedBuffering::kNo is set. Should only be called once.
    void QueueThreadUntil(BootTimestamp time);

   private:
    void TrackMessageSendTiming(const RawSender &sender,
                                monotonic_clock::time_point expected_send_time);
    void SendMessageTimings();
    // Log file.
    std::unique_ptr<TimestampMapper> timestamp_mapper_;
    const TimestampQueueStrategy timestamp_queue_strategy_;

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

    // Returns a list of channels which LogReader will send on but which may
    // *also* get sent on by other applications in replay.
    std::vector<
        std::pair<const aos::Channel *, NodeEventLoopFactory::ExclusiveSenders>>
    NonExclusiveChannels();

    // Stores all the timestamps that have been sent on this channel.  This is
    // only done for channels which are forwarded and on the node which
    // initially sends the message.  Compress using ranges and offsets.
    std::vector<std::unique_ptr<std::vector<ContiguousSentTimestamp>>>
        queue_index_map_;

    // Factory (if we are in sim) that this loop was created on.
    NodeEventLoopFactory *node_event_loop_factory_ = nullptr;
    SimulatedEventLoopFactory *event_loop_factory_ = nullptr;

    // Callback for when this node hits its realtime end time.
    std::function<void()> notice_realtime_end_;

    std::unique_ptr<EventLoop> event_loop_unique_ptr_;
    // Event loop.
    const Node *node_ = nullptr;
    EventLoop *event_loop_ = nullptr;
    // And timer used to send messages.
    TimerHandler *timer_handler_ = nullptr;
    TimerHandler *startup_timer_ = nullptr;

    std::unique_ptr<EventNotifier> start_event_notifier_;
    std::unique_ptr<EventNotifier> end_event_notifier_;

    // Filters (or nullptr if it isn't a forwarded channel) for each channel.
    // This corresponds to the object which is shared among all the channels
    // going between 2 nodes.  The second element in the tuple indicates if this
    // is the primary direction or not.
    std::vector<message_bridge::NoncausalOffsetEstimator *> filters_;
    message_bridge::MultiNodeNoncausalOffsetEstimator *multinode_filters_;

    // List of States (or nullptr if it isn't a forwarded channel) which
    // correspond to the originating node.
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

    // Time offset between the log's monotonic clock and the current event
    // loop's monotonic clock.  Useful when replaying logs with non-simulated
    // event loops.
    std::chrono::nanoseconds clock_offset_{0};

    std::vector<std::function<void()>> on_starts_;
    std::vector<std::function<void()>> on_ends_;

    std::atomic<bool> stopped_ = false;
    std::atomic<bool> started_ = false;

    bool found_last_message_ = false;
    std::vector<bool> last_message_;

    std::vector<timing::MessageTimingT> send_timings_;
    aos::Sender<timing::ReplayTiming> timing_statistics_sender_;

    // Protects access to any internal state after Run() is called. Designed
    // assuming that only one node is actually executing in replay.
    // Threading design:
    // * The worker passed to message_queuer_ has full ownership over all
    //   the log-reading code, timestamp filters, last_queued_message_, etc.
    // * The main thread should only have exclusive access to the replay
    //   event loop and associated features (mainly senders).
    //   It will pop an item out of the queue (which does maintain a shared_ptr
    //   reference which may also be being used by the message_queuer_ thread,
    //   but having shared_ptr's accessing the same memory from
    //   separate threads is permissible).
    // Enabling this in simulation is currently infeasible due to a lack of
    // synchronization in the MultiNodeNoncausalOffsetEstimator. Essentially,
    // when the message_queuer_ thread attempts to read/pop messages from the
    // timestamp_mapper_, it will end up calling callbacks that update the
    // internal state of the MultiNodeNoncausalOffsetEstimator. Simultaneously,
    // the event scheduler that is running in the main thread to orchestrate the
    // simulation will be querying the estimator to know what the clocks on the
    // various nodes are at, leading to potential issues.
    ThreadedBuffering threading_;
    std::optional<BootTimestamp> last_queued_message_;
    std::optional<util::ThreadedQueue<TimestampedMessage, BootTimestamp>>
        message_queuer_;

    // If a ReplayChannels was passed to LogReader, this will hold the
    // indices of the channels to replay for the Node represented by
    // the instance of LogReader::State.
    std::unique_ptr<const ReplayChannelIndices> replay_channel_indices_;
    const std::vector<std::function<void(void *message)>>
        before_send_callbacks_;
  };

  // Checks if any of the States have been constructed yet.
  // This happens during Register
  bool AreStatesInitialized() const;

  // If a ReplayChannels was passed to LogReader then creates a
  // ReplayChannelIndices for the given node. Otherwise, returns a nullptr.
  std::unique_ptr<const ReplayChannelIndices> MaybeMakeReplayChannelIndices(
      const Node *node);

  // Node index -> State.
  std::vector<std::unique_ptr<State>> states_;

  // Creates the requested filter if it doesn't exist, regardless of whether
  // these nodes can actually communicate directly.  The second return value
  // reports if this is the primary direction or not.
  message_bridge::NoncausalOffsetEstimator *GetFilter(const Node *node_a,
                                                      const Node *node_b);

  // Returns the timestamp queueing strategy to use.
  TimestampQueueStrategy ComputeTimestampQueueStrategy() const;

  // List of filters for a connection.  The pointer to the first node will be
  // less than the second node.
  std::unique_ptr<message_bridge::MultiNodeNoncausalOffsetEstimator> filters_;

  std::unique_ptr<SimulatedEventLoopFactory> event_loop_factory_unique_ptr_;
  SimulatedEventLoopFactory *event_loop_factory_ = nullptr;

  // Number of nodes which still have data to send.  This is used to figure out
  // when to exit.
  size_t live_nodes_ = 0;

  // Similar counter to live_nodes_, but for tracking which individual nodes are
  // running and have yet to hit the realtime end time, if any.
  size_t live_nodes_with_realtime_time_end_ = 0;

  const Configuration *replay_configuration_ = nullptr;

  // If a ReplayChannels was passed to LogReader, this will hold the
  // name and type of channels to replay which is used when creating States.
  const ReplayChannels *replay_channels_ = nullptr;

  // The callbacks that will be called before sending a message indexed by the
  // channel index from the logged_configuration
  std::vector<std::function<void(void *message)>> before_send_callbacks_;

  // If true, the replay timer will ignore any missing data.  This is used
  // during startup when we are bootstrapping everything and trying to get to
  // the start of all the log files.
  bool ignore_missing_data_ = false;

  // Whether to exit the SimulatedEventLoop when we finish reading the logs.
  bool exit_on_finish_ = true;

  realtime_clock::time_point start_time_ = realtime_clock::min_time;
  realtime_clock::time_point end_time_ = realtime_clock::max_time;
  ConfigRemapper config_remapper_;
};

}  // namespace logger
}  // namespace aos

#endif  // AOS_EVENTS_LOGGING_LOG_READER_H_

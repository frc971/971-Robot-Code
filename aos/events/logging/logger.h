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
#include "aos/events/logging/logfile_utils.h"
#include "aos/events/logging/logger_generated.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/network/message_bridge_server_generated.h"
#include "aos/network/timestamp_filter.h"
#include "aos/time/time.h"
#include "flatbuffers/flatbuffers.h"
#include "third_party/gmp/gmpxx.h"

namespace aos {
namespace logger {

class LogNamer {
 public:
  LogNamer(const Node *node) : node_(node) { nodes_.emplace_back(node_); }
  virtual ~LogNamer() {}

  virtual void WriteHeader(
      const aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader> &header,
      const Node *node) = 0;
  virtual DetachedBufferWriter *MakeWriter(const Channel *channel) = 0;

  virtual DetachedBufferWriter *MakeTimestampWriter(const Channel *channel) = 0;
  virtual DetachedBufferWriter *MakeForwardedTimestampWriter(
      const Channel *channel, const Node *node) = 0;
  virtual void Rotate(
      const Node *node,
      const aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader>
          &header) = 0;
  const std::vector<const Node *> &nodes() const { return nodes_; }

  const Node *node() const { return node_; }

 protected:
  const Node *const node_;
  std::vector<const Node *> nodes_;
};

class LocalLogNamer : public LogNamer {
 public:
  LocalLogNamer(std::string_view base_name, const Node *node)
      : LogNamer(node), base_name_(base_name), data_writer_(OpenDataWriter()) {}

  void WriteHeader(
      const aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader> &header,
      const Node *node) override {
    CHECK_EQ(node, this->node());
    data_writer_->WriteSizedFlatbuffer(header.full_span());
  }

  DetachedBufferWriter *MakeWriter(const Channel *channel) override {
    CHECK(configuration::ChannelIsSendableOnNode(channel, node()));
    return data_writer_.get();
  }

  void Rotate(const Node *node,
              const aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader>
                  &header) override {
    CHECK(node == this->node());
    ++part_number_;
    *data_writer_ = std::move(*OpenDataWriter());
    data_writer_->WriteSizedFlatbuffer(header.full_span());
  }

  DetachedBufferWriter *MakeTimestampWriter(const Channel *channel) override {
    CHECK(configuration::ChannelIsReadableOnNode(channel, node_))
        << ": Message is not delivered to this node.";
    CHECK(node_ != nullptr) << ": Can't log timestamps in a single node world";
    CHECK(configuration::ConnectionDeliveryTimeIsLoggedOnNode(channel, node_,
                                                              node_))
        << ": Delivery times aren't logged for this channel on this node.";
    return data_writer_.get();
  }

  DetachedBufferWriter *MakeForwardedTimestampWriter(
      const Channel * /*channel*/, const Node * /*node*/) override {
    LOG(FATAL) << "Can't log forwarded timestamps in a singe log file.";
    return nullptr;
  }

 private:
  std::unique_ptr<DetachedBufferWriter> OpenDataWriter() {
    return std::make_unique<DetachedBufferWriter>(
        absl::StrCat(base_name_, ".part", part_number_, ".bfbs"));
  }
  const std::string base_name_;
  size_t part_number_ = 0;
  std::unique_ptr<DetachedBufferWriter> data_writer_;
};

// TODO(austin): Split naming files from making files so we can re-use the
// naming code to predict the log file names for a provided base name.
class MultiNodeLogNamer : public LogNamer {
 public:
  MultiNodeLogNamer(std::string_view base_name,
                    const Configuration *configuration, const Node *node)
      : LogNamer(node),
        base_name_(base_name),
        configuration_(configuration),
        data_writer_(OpenDataWriter()) {}

  // Writes the header to all log files for a specific node.  This function
  // needs to be called after all the writers are created.
  void WriteHeader(
      const aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader> &header,
      const Node *node) override;

  void Rotate(const Node *node,
              const aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader>
                  &header) override;

  // Makes a data logger for a specific channel.
  DetachedBufferWriter *MakeWriter(const Channel *channel) override {
    // See if we can read the data on this node at all.
    const bool is_readable =
        configuration::ChannelIsReadableOnNode(channel, this->node());
    if (!is_readable) {
      return nullptr;
    }

    // Then, see if we are supposed to log the data here.
    const bool log_message =
        configuration::ChannelMessageIsLoggedOnNode(channel, this->node());

    if (!log_message) {
      return nullptr;
    }

    // Now, sort out if this is data generated on this node, or not.  It is
    // generated if it is sendable on this node.
    if (configuration::ChannelIsSendableOnNode(channel, this->node())) {
      return data_writer_.get();
    }

    // Ok, we have data that is being forwarded to us that we are supposed to
    // log.  It needs to be logged with send timestamps, but be sorted enough
    // to be able to be processed.
    CHECK(data_writers_.find(channel) == data_writers_.end());

    // Track that this node is being logged.
    const Node *source_node = configuration::GetNode(
        configuration_, channel->source_node()->string_view());

    if (std::find(nodes_.begin(), nodes_.end(), source_node) == nodes_.end()) {
      nodes_.emplace_back(source_node);
    }

    DataWriter data_writer;
    data_writer.node = source_node;
    data_writer.rotate = [this](const Channel *channel,
                                DataWriter *data_writer) {
      OpenWriter(channel, data_writer);
    };
    data_writer.rotate(channel, &data_writer);

    return data_writers_.insert(std::make_pair(channel, std::move(data_writer)))
        .first->second.writer.get();
  }

  DetachedBufferWriter *MakeForwardedTimestampWriter(
      const Channel *channel, const Node *node) override {
    // See if we can read the data on this node at all.
    const bool is_readable =
        configuration::ChannelIsReadableOnNode(channel, this->node());
    CHECK(is_readable) << ": "
                       << configuration::CleanedChannelToString(channel);

    CHECK(data_writers_.find(channel) == data_writers_.end());

    if (std::find(nodes_.begin(), nodes_.end(), node) == nodes_.end()) {
      nodes_.emplace_back(node);
    }

    DataWriter data_writer;
    data_writer.node = node;
    data_writer.rotate = [this](const Channel *channel,
                                DataWriter *data_writer) {
      OpenForwardedTimestampWriter(channel, data_writer);
    };
    data_writer.rotate(channel, &data_writer);

    return data_writers_.insert(std::make_pair(channel, std::move(data_writer)))
        .first->second.writer.get();
  }

  // Makes a timestamp (or timestamp and data) logger for a channel and
  // forwarding connection.
  DetachedBufferWriter *MakeTimestampWriter(const Channel *channel) override {
    const bool log_delivery_times =
        (this->node() == nullptr)
            ? false
            : configuration::ConnectionDeliveryTimeIsLoggedOnNode(
                  channel, this->node(), this->node());
    if (!log_delivery_times) {
      return nullptr;
    }

    return data_writer_.get();
  }

  const std::vector<const Node *> &nodes() const { return nodes_; }

 private:
  // Files to write remote data to.  We want one per channel.  Maps the channel
  // to the writer, Node, and part number.
  struct DataWriter {
    std::unique_ptr<DetachedBufferWriter> writer = nullptr;
    const Node *node;
    size_t part_number = 0;
    std::function<void(const Channel *, DataWriter *)> rotate;
  };

  void OpenForwardedTimestampWriter(const Channel *channel,
                                    DataWriter *data_writer) {
    std::string filename =
        absl::StrCat(base_name_, "_timestamps", channel->name()->string_view(),
                     "/", channel->type()->string_view(), ".part",
                     data_writer->part_number, ".bfbs");

    if (!data_writer->writer) {
      data_writer->writer = std::make_unique<DetachedBufferWriter>(filename);
    } else {
      *data_writer->writer = DetachedBufferWriter(filename);
    }
  }

  void OpenWriter(const Channel *channel, DataWriter *data_writer) {
    const std::string filename = absl::StrCat(
        base_name_, "_", channel->source_node()->string_view(), "_data",
        channel->name()->string_view(), "/", channel->type()->string_view(),
        ".part", data_writer->part_number, ".bfbs");
    if (!data_writer->writer) {
      data_writer->writer = std::make_unique<DetachedBufferWriter>(filename);
    } else {
      *data_writer->writer = DetachedBufferWriter(filename);
    }
  }

  std::unique_ptr<DetachedBufferWriter> OpenDataWriter() {
    return std::make_unique<DetachedBufferWriter>(
        absl::StrCat(base_name_, "_", node()->name()->string_view(),
                     "_data.part", part_number_, ".bfbs"));
  }

  const std::string base_name_;
  const Configuration *const configuration_;

  size_t part_number_ = 0;

  // File to write both delivery timestamps and local data to.
  std::unique_ptr<DetachedBufferWriter> data_writer_;

  std::map<const Channel *, DataWriter> data_writers_;
};

// Logs all channels available in the event loop to disk every 100 ms.
// Start by logging one message per channel to capture any state and
// configuration that is sent rately on a channel and would affect execution.
class Logger {
 public:
  Logger(std::string_view base_name, EventLoop *event_loop,
         std::chrono::milliseconds polling_period =
             std::chrono::milliseconds(100));
  Logger(std::unique_ptr<LogNamer> log_namer, EventLoop *event_loop,
         std::chrono::milliseconds polling_period =
             std::chrono::milliseconds(100));

  // Rotates the log file(s), triggering new part files to be written for each
  // log file.
  void Rotate();

 private:
  void WriteHeader();
  aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader> MakeHeader(
      const Node *node);

  bool MaybeUpdateTimestamp(
      const Node *node, int node_index,
      aos::monotonic_clock::time_point monotonic_start_time,
      aos::realtime_clock::time_point realtime_start_time);

  void DoLogData();

  void WriteMissingTimestamps();

  void StartLogging();

  // Fetches from each channel until all the data is logged.
  void LogUntil(monotonic_clock::time_point t);

  EventLoop *event_loop_;
  std::unique_ptr<LogNamer> log_namer_;

  // Structure to track both a fetcher, and if the data fetched has been
  // written.  We may want to delay writing data to disk so that we don't let
  // data get too far out of order when written to disk so we can avoid making
  // it too hard to sort when reading.
  struct FetcherStruct {
    std::unique_ptr<RawFetcher> fetcher;
    bool written = false;

    int channel_index = -1;

    LogType log_type = LogType::kLogMessage;

    DetachedBufferWriter *writer = nullptr;
    DetachedBufferWriter *timestamp_writer = nullptr;
    DetachedBufferWriter *contents_writer = nullptr;
    const Node *writer_node = nullptr;
    const Node *timestamp_node = nullptr;
    int node_index = 0;
  };

  std::vector<FetcherStruct> fetchers_;
  TimerHandler *timer_handler_;

  // Period to poll the channels.
  const std::chrono::milliseconds polling_period_;

  // Last time that data was written for all channels to disk.
  monotonic_clock::time_point last_synchronized_time_;

  monotonic_clock::time_point monotonic_start_time_;
  realtime_clock::time_point realtime_start_time_;

  // Max size that the header has consumed.  This much extra data will be
  // reserved in the builder to avoid reallocating.
  size_t max_header_size_ = 0;

  // Fetcher for all the statistics from all the nodes.
  aos::Fetcher<message_bridge::ServerStatistics> server_statistics_fetcher_;

  // Sets the start time for a specific node.
  void SetStartTime(size_t node_index,
                    aos::monotonic_clock::time_point monotonic_start_time,
                    aos::realtime_clock::time_point realtime_start_time);

  struct NodeState {
    aos::monotonic_clock::time_point monotonic_start_time =
        aos::monotonic_clock::min_time;
    aos::realtime_clock::time_point realtime_start_time =
        aos::realtime_clock::min_time;

    aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader> log_file_header =
        aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader>::Empty();
  };
  std::vector<NodeState> node_state_;
};

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
  // Log filenames are in the following format:
  //
  //   {
  //     {log1_part0, log1_part1, ...},
  //     {log2}
  //   }
  // The inner vector is a list of log file chunks which form up a log file.
  // The outer vector is a list of log files with subsets of the messages, or
  // messages from different nodes.
  //
  // If the outer vector isn't provided, it is assumed to be of size 1.
  LogReader(std::string_view filename,
            const Configuration *replay_configuration = nullptr);
  LogReader(const std::vector<std::string> &filenames,
            const Configuration *replay_configuration = nullptr);
  LogReader(const std::vector<std::vector<std::string>> &filenames,
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

  // Returns the configuration from the log file.
  const Configuration *logged_configuration() const;
  // Returns the configuration being used for replay.
  // The pointer is invalidated whenever RemapLoggedChannel is called.
  const Configuration *configuration() const;

  // Returns the nodes that this log file was created on.  This is a list of
  // pointers to a node in the nodes() list inside configuration().  The
  // pointers here are invalidated whenever RemapLoggedChannel is called.
  std::vector<const Node *> Nodes() const;

  // Returns the starting timestamp for the log file.
  monotonic_clock::time_point monotonic_start_time(const Node *node = nullptr);
  realtime_clock::time_point realtime_start_time(const Node *node = nullptr);

  // Causes the logger to publish the provided channel on a different name so
  // that replayed applications can publish on the proper channel name without
  // interference. This operates on raw channel names, without any node or
  // application specific mappings.
  void RemapLoggedChannel(std::string_view name, std::string_view type,
                          std::string_view add_prefix = "/original");
  template <typename T>
  void RemapLoggedChannel(std::string_view name,
                          std::string_view add_prefix = "/original") {
    RemapLoggedChannel(name, T::GetFullyQualifiedName(), add_prefix);
  }

  template <typename T>
  bool HasChannel(std::string_view name) {
    return configuration::GetChannel(log_file_header()->configuration(), name,
                                     T::GetFullyQualifiedName(), "",
                                     nullptr) != nullptr;
  }

  SimulatedEventLoopFactory *event_loop_factory() {
    return event_loop_factory_;
  }

  const LogFileHeader *log_file_header() const {
    return &log_file_header_.message();
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

  const std::vector<std::vector<std::string>> filenames_;

  // This is *a* log file header used to provide the logged config.  The rest of
  // the header is likely distracting.
  FlatbufferVector<LogFileHeader> log_file_header_;

  // Returns [ta; tb; ...] = tuple[0] * t + tuple[1]
  std::tuple<Eigen::Matrix<double, Eigen::Dynamic, 1>,
             Eigen::Matrix<double, Eigen::Dynamic, 1>>
  SolveOffsets();

  void LogFit(std::string_view prefix);

  // State per node.
  class State {
   public:
    State(std::unique_ptr<ChannelMerger> channel_merger);

    // Returns the timestamps, channel_index, and message from a channel.
    // update_time (will be) set to true when popping this message causes the
    // filter to change the time offset estimation function.
    std::tuple<TimestampMerger::DeliveryTimestamp, int,
               FlatbufferVector<MessageHeader>>
    PopOldest(bool *update_time);

    // Returns the monotonic time of the oldest message.
    monotonic_clock::time_point OldestMessageTime() const;

    // Primes the queues inside State.  Should be called before calling
    // OldestMessageTime.
    void SeedSortedMessages();

    // Returns the starting time for this node.
    monotonic_clock::time_point monotonic_start_time() const {
      return channel_merger_->monotonic_start_time();
    }
    realtime_clock::time_point realtime_start_time() const {
      return channel_merger_->realtime_start_time();
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
      return channel_target_event_loop_factory_[channel_index]->monotonic_now();
    }

    distributed_clock::time_point RemoteToDistributedClock(
        size_t channel_index, monotonic_clock::time_point time) {
      return channel_target_event_loop_factory_[channel_index]
          ->ToDistributedClock(time);
    }

    const Node *remote_node(size_t channel_index) {
      return channel_target_event_loop_factory_[channel_index]->node();
    }

    monotonic_clock::time_point monotonic_now() {
      return node_event_loop_factory_->monotonic_now();
    }

    // Sets the node we will be merging as, and returns true if there is any
    // data on it.
    bool SetNode() { return channel_merger_->SetNode(event_loop_->node()); }

    // Sets the number of channels.
    void SetChannelCount(size_t count);

    // Sets the sender, filter, and target factory for a channel.
    void SetChannel(size_t channel, std::unique_ptr<RawSender> sender,
                    message_bridge::NoncausalOffsetEstimator *filter,
                    NodeEventLoopFactory *channel_target_event_loop_factory);

    // Returns if we have read all the messages from all the logs.
    bool at_end() const { return channel_merger_->at_end(); }

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
    bool Send(size_t channel_index, const void *data, size_t size,
              aos::monotonic_clock::time_point monotonic_remote_time,
              aos::realtime_clock::time_point realtime_remote_time,
              uint32_t remote_queue_index) {
      return channels_[channel_index]->Send(data, size, monotonic_remote_time,
                                            realtime_remote_time,
                                            remote_queue_index);
    }

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
                              std::get<2>(message).message().channel_index()))
                   << "\n";
        } else if (i == 7) {
          messages << "...\n";
        }
        ++i;
      }
      return messages.str() + channel_merger_->DebugString();
    }

   private:
    // Log file.
    std::unique_ptr<ChannelMerger> channel_merger_;

    std::deque<std::tuple<TimestampMerger::DeliveryTimestamp, int,
                          FlatbufferVector<MessageHeader>,
                          message_bridge::NoncausalOffsetEstimator *>>
        sorted_messages_;

    // Senders.
    std::vector<std::unique_ptr<RawSender>> channels_;

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
    std::vector<NodeEventLoopFactory *> channel_target_event_loop_factory_;
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
  //   (ta + tb + tc + td) / num_nodex = t_distributed
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
  std::map<size_t, std::string> remapped_channels_;

  // Number of nodes which still have data to send.  This is used to figure out
  // when to exit.
  size_t live_nodes_ = 0;

  const Configuration *remapped_configuration_ = nullptr;
  const Configuration *replay_configuration_ = nullptr;

  // If true, the replay timer will ignore any missing data.  This is used
  // during startup when we are bootstrapping everything and trying to get to
  // the start of all the log files.
  bool ignore_missing_data_ = false;
};

}  // namespace logger
}  // namespace aos

#endif  // AOS_EVENTS_LOGGER_H_

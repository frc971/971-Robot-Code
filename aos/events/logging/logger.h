#ifndef AOS_EVENTS_LOGGER_H_
#define AOS_EVENTS_LOGGER_H_

#include <chrono>
#include <deque>
#include <string_view>
#include <vector>

#include "Eigen/Dense"
#include "absl/strings/str_cat.h"
#include "absl/types/span.h"
#include "aos/events/event_loop.h"
#include "aos/events/logging/logfile_utils.h"
#include "aos/events/logging/logger_generated.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/network/timestamp_filter.h"
#include "aos/time/time.h"
#include "flatbuffers/flatbuffers.h"

namespace aos {
namespace logger {

class LogNamer {
 public:
  LogNamer(const Node *node) : node_(node) { nodes_.emplace_back(node_); }
  virtual ~LogNamer() {}

  virtual void WriteHeader(flatbuffers::FlatBufferBuilder *fbb,
                           const Node *node) = 0;
  virtual DetachedBufferWriter *MakeWriter(const Channel *channel) = 0;

  virtual DetachedBufferWriter *MakeTimestampWriter(const Channel *channel) = 0;
  const std::vector<const Node *> &nodes() const { return nodes_; }

  const Node *node() const { return node_; }

 protected:
  const Node *const node_;
  std::vector<const Node *> nodes_;
};

class LocalLogNamer : public LogNamer {
 public:
  LocalLogNamer(DetachedBufferWriter *writer, const Node *node)
      : LogNamer(node), writer_(writer) {}

  ~LocalLogNamer() override { writer_->Flush(); }

  void WriteHeader(flatbuffers::FlatBufferBuilder *fbb,
                   const Node *node) override {
    CHECK_EQ(node, this->node());
    writer_->WriteSizedFlatbuffer(
        absl::Span<const uint8_t>(fbb->GetBufferPointer(), fbb->GetSize()));
  }

  DetachedBufferWriter *MakeWriter(const Channel *channel) override {
    CHECK(configuration::ChannelIsSendableOnNode(channel, node()));
    return writer_;
  }

  DetachedBufferWriter *MakeTimestampWriter(const Channel *channel) override {
    CHECK(configuration::ChannelIsReadableOnNode(channel, node_))
        << ": Message is not delivered to this node.";
    CHECK(node_ != nullptr) << ": Can't log timestamps in a single node world";
    CHECK(configuration::ConnectionDeliveryTimeIsLoggedOnNode(channel, node_,
                                                              node_))
        << ": Delivery times aren't logged for this channel on this node.";
    return writer_;
  }

 private:
  DetachedBufferWriter *writer_;
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
        data_writer_(std::make_unique<DetachedBufferWriter>(absl::StrCat(
            base_name_, "_", node->name()->string_view(), "_data.bfbs"))) {}

  // Writes the header to all log files for a specific node.  This function
  // needs to be called after all the writers are created.
  void WriteHeader(flatbuffers::FlatBufferBuilder *fbb, const Node *node) {
    if (node == this->node()) {
      data_writer_->WriteSizedFlatbuffer(
          absl::Span<const uint8_t>(fbb->GetBufferPointer(), fbb->GetSize()));
    } else {
      for (std::pair<const Channel *const,
                     std::unique_ptr<DetachedBufferWriter>> &data_writer :
           data_writers_) {
        if (configuration::ChannelIsSendableOnNode(data_writer.first, node)) {
          data_writer.second->WriteSizedFlatbuffer(absl::Span<const uint8_t>(
              fbb->GetBufferPointer(), fbb->GetSize()));
        }
      }
    }
  }

  // Makes a data logger for a specific channel.
  DetachedBufferWriter *MakeWriter(const Channel *channel) {
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
    } else {
      // Ok, we have data that is being forwarded to us that we are supposed to
      // log.  It needs to be logged with send timestamps, but be sorted enough
      // to be able to be processed.
      CHECK(data_writers_.find(channel) == data_writers_.end());

      // Track that this node is being logged.
      if (configuration::MultiNode(configuration_)) {
        const Node *source_node = configuration::GetNode(
            configuration_, channel->source_node()->string_view());
        if (std::find(nodes_.begin(), nodes_.end(), source_node) ==
            nodes_.end()) {
          nodes_.emplace_back(source_node);
        }
      }

      return data_writers_
          .insert(std::make_pair(
              channel,
              std::make_unique<DetachedBufferWriter>(absl::StrCat(
                  base_name_, "_", channel->source_node()->string_view(),
                  "_data", channel->name()->string_view(), "/",
                  channel->type()->string_view(), ".bfbs"))))
          .first->second.get();
    }
  }

  // Makes a timestamp (or timestamp and data) logger for a channel and
  // forwarding connection.
  DetachedBufferWriter *MakeTimestampWriter(const Channel *channel) {
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
  const std::string base_name_;
  const Configuration *const configuration_;

  // File to write both delivery timestamps and local data to.
  std::unique_ptr<DetachedBufferWriter> data_writer_;
  // Files to write remote data to.  We want one per channel.
  std::map<const Channel *, std::unique_ptr<DetachedBufferWriter>>
      data_writers_;
};


// Logs all channels available in the event loop to disk every 100 ms.
// Start by logging one message per channel to capture any state and
// configuration that is sent rately on a channel and would affect execution.
class Logger {
 public:
  Logger(DetachedBufferWriter *writer, EventLoop *event_loop,
         std::chrono::milliseconds polling_period =
             std::chrono::milliseconds(100));
  Logger(std::unique_ptr<LogNamer> log_namer, EventLoop *event_loop,
         std::chrono::milliseconds polling_period =
             std::chrono::milliseconds(100));

  // Rotates the log file with the new writer.  This writes out the header
  // again, but keeps going as if nothing else happened.
  void Rotate(DetachedBufferWriter *writer);
  void Rotate(std::unique_ptr<LogNamer> log_namer);

 private:
  void WriteHeader();
  void WriteHeader(const Node *node);

  void DoLogData();

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

  const std::vector<std::vector<std::string>> filenames_;

  // This is *a* log file header used to provide the logged config.  The rest of
  // the header is likely distracting.
  FlatbufferVector<LogFileHeader> log_file_header_;

  Eigen::Matrix<double, Eigen::Dynamic, 1> SolveOffsets();

  // State per node.
  struct State {
    // Log file.
    std::unique_ptr<ChannelMerger> channel_merger;
    // Senders.
    std::vector<std::unique_ptr<RawSender>> channels;

    // Factory (if we are in sim) that this loop was created on.
    NodeEventLoopFactory *node_event_loop_factory = nullptr;
    std::unique_ptr<EventLoop> event_loop_unique_ptr;
    // Event loop.
    EventLoop *event_loop = nullptr;
    // And timer used to send messages.
    TimerHandler *timer_handler;

    // Updates the timestamp filter with the timestamp.  Returns true if the
    // provided timestamp was actually a forwarding timestamp and used, and
    // false otherwise.
    bool MaybeUpdateTimestamp(
        const TimestampMerger::DeliveryTimestamp &channel_timestamp,
        int channel_index);

    // Filters (or nullptr if it isn't a forwarded channel) for each channel.
    // This corresponds to the object which is shared among all the channels
    // going between 2 nodes.  The second element in the tuple indicates if this
    // is the primary direction or not.
    std::vector<std::tuple<message_bridge::ClippedAverageFilter *, bool>>
        filters;

    // List of NodeEventLoopFactorys (or nullptr if it isn't a forwarded
    // channel) which correspond to the originating node.
    std::vector<NodeEventLoopFactory *> channel_target_event_loop_factory;
  };

  // Node index -> State.
  std::vector<std::unique_ptr<State>> states_;

  // Creates the requested filter if it doesn't exist, regardless of whether
  // these nodes can actually communicate directly.  The second return value
  // reports if this is the primary direction or not.
  std::tuple<message_bridge::ClippedAverageFilter *, bool> GetFilter(
      const Node *node_a, const Node *node_b);

  // FILE to write offsets to (if populated).
  FILE *offset_fp_ = nullptr;
  // Timestamp of the first piece of data used for the horizontal axis on the
  // plot.
  aos::realtime_clock::time_point first_time_;

  // List of filters for a connection.  The pointer to the first node will be
  // less than the second node.
  std::map<std::tuple<const Node *, const Node *>,
           message_bridge::ClippedAverageFilter>
      filters_;

  // Returns the offset from the monotonic clock for a node to the distributed
  // clock.  distributed = monotonic + offset;
  std::chrono::nanoseconds offset(int node_index) const {
    CHECK_LT(node_index, offset_matrix_.rows())
        << ": Got too high of a node index.";
    return -std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::duration<double>(offset_matrix_(node_index))) -
           base_offset_matrix_(node_index);
  }

  // Updates the offset matrix solution and sets the per-node distributed
  // offsets in the factory.
  void UpdateOffsets();

  // sample_matrix_ = map_matrix_ * offset_matrix_
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> map_matrix_;
  Eigen::Matrix<double, Eigen::Dynamic, 1> sample_matrix_;
  Eigen::Matrix<double, Eigen::Dynamic, 1> offset_matrix_;

  // Base offsets.  The actual offset is the sum of this and the offset matrix.
  // This removes some of the dynamic range challenges from the double above.
  Eigen::Matrix<std::chrono::nanoseconds, Eigen::Dynamic, 1>
      base_offset_matrix_;

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

#ifndef AOS_EVENTS_LOGGING_LOG_NAMER_H_
#define AOS_EVENTS_LOGGING_LOG_NAMER_H_

#include <functional>
#include <map>
#include <memory>
#include <string_view>
#include <vector>

#include "absl/container/btree_map.h"
#include "flatbuffers/flatbuffers.h"
#include "glog/logging.h"

#include "aos/events/logging/logfile_utils.h"
#include "aos/events/logging/logger_generated.h"
#include "aos/uuid.h"

namespace aos {
namespace logger {

class LogNamer;

// TODO(austin): Rename this back to DataWriter once all other callers are of
// the old DataWriter.
//
// Class to manage writing data to log files.  This lets us track which boot the
// written header has in it, and if the header has been written or not.
//
// The design of this class is that instead of being notified when any of the
// header data changes, it polls and owns that decision.  This makes it much
// harder to write corrupted data.  If that becomes a performance problem, we
// can DCHECK and take it out of production binaries.
class NewDataWriter {
 public:
  // Constructs a NewDataWriter.
  // log_namer is the log namer which holds the config and any other data we
  // need for our header.
  // node is the node whom's prespective we are logging from.
  // reopen is called whenever a file needs to be reopened.
  // close is called to close that file and extract any statistics.
  NewDataWriter(LogNamer *log_namer, const Node *node, const Node *logger_node,
                std::function<void(NewDataWriter *)> reopen,
                std::function<void(NewDataWriter *)> close,
                size_t max_message_size,
                std::initializer_list<StoredDataType> types);

  void UpdateMaxMessageSize(size_t new_size) {
    if (new_size > max_message_size_) {
      CHECK(!header_written_) << ": Tried to update to " << new_size << ", was "
                              << max_message_size_ << " for " << name();
      max_message_size_ = new_size;
    }
  }
  size_t max_message_size() const { return max_message_size_; }

  std::chrono::nanoseconds max_out_of_order_duration() const {
    return max_out_of_order_duration_;
  }

  NewDataWriter(NewDataWriter &&other) = default;
  aos::logger::NewDataWriter &operator=(NewDataWriter &&other) = default;
  NewDataWriter(const NewDataWriter &) = delete;
  void operator=(const NewDataWriter &) = delete;

  ~NewDataWriter();

  // Rotates the log file, delaying writing the new header until data arrives.
  void Rotate();

  // Updates all the metadata in the log file about the remote node which this
  // message is from.
  void UpdateRemote(size_t remote_node_index, const UUID &remote_node_boot_uuid,
                    monotonic_clock::time_point monotonic_remote_time,
                    monotonic_clock::time_point monotonic_event_time,
                    bool reliable,
                    monotonic_clock::time_point monotonic_timestamp_time =
                        monotonic_clock::min_time);

  // Coppies a message with the provided boot UUID.
  void CopyDataMessage(DataEncoder::Copier *copier,
                       const UUID &source_node_boot_uuid,
                       aos::monotonic_clock::time_point now,
                       aos::monotonic_clock::time_point message_time);
  void CopyTimestampMessage(DataEncoder::Copier *copier,
                            const UUID &source_node_boot_uuid,
                            aos::monotonic_clock::time_point now,
                            aos::monotonic_clock::time_point message_time);
  void CopyRemoteTimestampMessage(
      DataEncoder::Copier *copier, const UUID &source_node_boot_uuid,
      aos::monotonic_clock::time_point now,
      aos::monotonic_clock::time_point message_time);

  // Updates the current boot for the source node.  This is useful when you want
  // to queue a message that may trigger a reboot rotation, but then need to
  // update the remote timestamps.
  void UpdateBoot(const UUID &source_node_boot_uuid);

  // Returns the name of the writer. It may be a filename, but assume it is not.
  std::string_view name() const { return writer ? writer->name() : "(closed)"; }

  void Close();

  std::unique_ptr<DetachedBufferWriter> writer = nullptr;

  size_t node_index() const { return node_index_; }
  const UUID &parts_uuid() const { return parts_uuid_; }
  size_t parts_index() const { return parts_index_; }
  const Node *node() const { return node_; }

  // Datastructure used to capture all the information about a remote node.
  struct State {
    // Boot UUID of the node.
    UUID boot_uuid = UUID::Zero();
    // Timestamp on the remote monotonic clock of the oldest message sent to
    // node_index_.
    monotonic_clock::time_point oldest_remote_monotonic_timestamp =
        monotonic_clock::max_time;
    // Timestamp on the local monotonic clock of the message in
    // oldest_remote_monotonic_timestamp.
    monotonic_clock::time_point oldest_local_monotonic_timestamp =
        monotonic_clock::max_time;
    // Timestamp on the remote monotonic clock of the oldest message sent to
    // node_index_, excluding messages forwarded with time_to_live() == 0.
    monotonic_clock::time_point oldest_remote_unreliable_monotonic_timestamp =
        monotonic_clock::max_time;
    // Timestamp on the local monotonic clock of the message in
    // oldest_local_unreliable_monotonic_timestamp.
    monotonic_clock::time_point oldest_local_unreliable_monotonic_timestamp =
        monotonic_clock::max_time;

    // Timestamp on the remote monotonic clock of the oldest message sent to
    // node_index_, only including messages forwarded with time_to_live() == 0.
    monotonic_clock::time_point oldest_remote_reliable_monotonic_timestamp =
        monotonic_clock::max_time;
    // Timestamp on the local monotonic clock of the message in
    // oldest_local_reliable_monotonic_timestamp.
    monotonic_clock::time_point oldest_local_reliable_monotonic_timestamp =
        monotonic_clock::max_time;

    // Timestamp on the remote monotonic clock of the oldest message timestamp
    // sent back to logger_node_index_.  The remote here will be the node this
    // part is from the perspective of, ie node_index_.
    monotonic_clock::time_point
        oldest_logger_remote_unreliable_monotonic_timestamp =
            monotonic_clock::max_time;
    // The time on the monotonic clock of the logger when this timestamp made it
    // back to the logger (logger_node_index_).
    monotonic_clock::time_point
        oldest_logger_local_unreliable_monotonic_timestamp =
            monotonic_clock::max_time;
  };

 private:
  // Signals that a node has rebooted.
  void Reboot(const UUID &source_node_boot_uuid);

  void CopyMessage(DataEncoder::Copier *copier,
                   const UUID &source_node_boot_uuid,
                   aos::monotonic_clock::time_point now,
                   aos::monotonic_clock::time_point message_time);

  void QueueHeader(
      aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader> &&header);

  aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader> MakeHeader();

  monotonic_clock::time_point monotonic_start_time_ = monotonic_clock::min_time;

  const Node *node_ = nullptr;
  size_t node_index_ = 0;
  size_t logger_node_index_ = 0;
  LogNamer *log_namer_;
  UUID parts_uuid_ = UUID::Random();
  size_t parts_index_ = 0;

  std::function<void(NewDataWriter *)> reopen_;
  std::function<void(NewDataWriter *)> close_;
  bool header_written_ = false;

  std::vector<State> state_;

  size_t max_message_size_;

  // Each data writer logs the channels for that node, i.e.
  // each data writer writes one file. We may encounter messages which
  // violate the max out of order duration specified in the header of that file.
  // Rotate the data writer and start a new part for that particular file.
  // This shouldn't affect the headers of other data writers, so make this
  // a property of individual data writer instead of the overall log.
  std::chrono::nanoseconds max_out_of_order_duration_;

  // Monotonic time point of the latest message we've logged so far, i.e
  // Message X - time Z
  // Message Y - time Z + 1
  // newest_message_time_ = Z + 1 (even if X was logged after Y)
  //
  // Since the messages can be logged out of order, this helps determine if
  // max out of order duration was violated.
  monotonic_clock::time_point newest_message_time_ = monotonic_clock::min_time;

  // An array with a bool for each value of StoredDataType representing if that
  // data type is allowed to be logged by this object.
  std::array<bool, static_cast<size_t>(StoredDataType::MAX) + 1>
      allowed_data_types_;
};

// Interface describing how to name, track, and add headers to log file parts.
class LogNamer {
 public:
  // Constructs a LogNamer with the primary node (ie the one the logger runs on)
  // being node.
  LogNamer(const aos::Configuration *configuration, EventLoop *event_loop,
           const aos::Node *node)
      : event_loop_(event_loop),
        configuration_(configuration),
        node_(node),
        logger_node_index_(configuration::GetNodeIndex(configuration_, node_)) {
    nodes_.emplace_back(node_);
  }
  virtual ~LogNamer() = default;

  // Returns a writer for writing data from messages on this channel (on the
  // primary node).
  //
  // The returned pointer will stay valid across rotations, but the object it
  // points to will be assigned to.
  virtual NewDataWriter *MakeWriter(const Channel *channel) = 0;

  // Returns a writer for writing timestamps from messages on this channel (on
  // the primary node).
  //
  // The returned pointer will stay valid across rotations, but the object it
  // points to will be assigned to.
  virtual NewDataWriter *MakeTimestampWriter(const Channel *channel) = 0;

  // Returns a writer for writing timestamps delivered over the special
  // /aos/remote_timestamps/* channels.  node is the node that the timestamps
  // are forwarded back from (to the primary node).
  //
  // The returned pointer will stay valid across rotations, but the object it
  // points to will be assigned to.
  virtual NewDataWriter *MakeForwardedTimestampWriter(const Channel *channel,
                                                      const Node *node) = 0;

  // Rotates all log files for the provided node.
  virtual void Rotate(const Node *node) = 0;

  // Returns all the nodes that data is being written for.
  const std::vector<const Node *> &nodes() const { return nodes_; }

  // Closes all existing log data writers. No more data may be written after
  // this.
  virtual WriteCode Close() = 0;

  // Returns the node the logger is running on.
  const Node *node() const { return node_; }
  const UUID &logger_node_boot_uuid() const { return logger_node_boot_uuid_; }
  size_t logger_node_index() const { return logger_node_index_; }

  // Writes out the nested Configuration object to the config file location.
  virtual void WriteConfiguration(
      aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader> *header,
      std::string_view config_sha256) = 0;

  void SetHeaderTemplate(
      aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader> header) {
    header_ = std::move(header);
    logger_node_boot_uuid_ =
        UUID::FromString(header_.message().logger_node_boot_uuid());
  }

  void ClearStartTimes() { node_states_.clear(); }

  void SetStartTimes(size_t node_index, const UUID &boot_uuid,
                     monotonic_clock::time_point monotonic_start_time,
                     realtime_clock::time_point realtime_start_time,
                     monotonic_clock::time_point logger_monotonic_start_time,
                     realtime_clock::time_point logger_realtime_start_time) {
    VLOG(1) << "Setting node " << node_index << " to start time "
            << monotonic_start_time << " rt " << realtime_start_time << " UUID "
            << boot_uuid;
    NodeState *node_state = GetNodeState(node_index, boot_uuid);
    node_state->monotonic_start_time = monotonic_start_time;
    node_state->realtime_start_time = realtime_start_time;
    node_state->logger_monotonic_start_time = logger_monotonic_start_time;
    node_state->logger_realtime_start_time = logger_realtime_start_time;
  }

  monotonic_clock::time_point monotonic_start_time(size_t node_index,
                                                   const UUID &boot_uuid) {
    DCHECK_NE(boot_uuid, UUID::Zero());

    NodeState *node_state = GetNodeState(node_index, boot_uuid);
    return node_state->monotonic_start_time;
  }

  // This returns the initial out of order duration set in the header template
  // by the logger based on polling period. It may be different than the actual
  // duration used by the data writer.
  std::chrono::nanoseconds base_max_out_of_order_duration() const {
    return std::chrono::nanoseconds(
        header_.message().max_out_of_order_duration());
  }

 protected:
  // Structure with state per node about times and such.
  struct NodeState {
    // Time when this node started logging.
    monotonic_clock::time_point monotonic_start_time =
        monotonic_clock::min_time;
    realtime_clock::time_point realtime_start_time = realtime_clock::min_time;

    // Corresponding time on the logger node when it started logging.
    monotonic_clock::time_point logger_monotonic_start_time =
        monotonic_clock::min_time;
    realtime_clock::time_point logger_realtime_start_time =
        realtime_clock::min_time;
  };

  // Creates a new header by copying fields out of the template and combining
  // them with the arguments provided.
  aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader> MakeHeader(
      size_t node_index, const std::vector<NewDataWriter::State> &state,
      const UUID &parts_uuid, int parts_index,
      std::chrono::nanoseconds max_out_of_order_duration,
      const std::array<bool, static_cast<size_t>(StoredDataType::MAX) + 1>
          &allowed_data_types);

  EventLoop *event_loop_;
  const Configuration *const configuration_;
  const Node *const node_;
  const size_t logger_node_index_;
  UUID logger_node_boot_uuid_;
  std::vector<const Node *> nodes_;

  friend NewDataWriter;

  // Returns the start/stop time state structure for a node and boot.  We can
  // have data from multiple boots, and it makes sense to reuse the start/stop
  // times if we get data from the same boot again.
  NodeState *GetNodeState(size_t node_index, const UUID &boot_uuid);

  absl::btree_map<std::pair<size_t, UUID>, NodeState> node_states_;

  aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader> header_ =
      aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader>::Empty();
};

// Log namer which uses a config to name a bunch of files.
class MultiNodeLogNamer : public LogNamer {
 public:
  MultiNodeLogNamer(std::unique_ptr<LogBackend> log_backend,
                    EventLoop *event_loop);
  MultiNodeLogNamer(std::unique_ptr<LogBackend> log_backend,
                    const Configuration *configuration, EventLoop *event_loop,
                    const Node *node);
  ~MultiNodeLogNamer() override;

  // Sets the function for creating encoders.  The argument is the max message
  // size (including headers) that will be written into this encoder.
  //
  // Defaults to just creating DummyEncoders.
  void set_encoder_factory(
      std::function<std::unique_ptr<DataEncoder>(size_t)> encoder_factory) {
    encoder_factory_ = std::move(encoder_factory);
  }

  // Sets an additional file extension.
  //
  // Defaults to nothing.
  void set_extension(std::string_view extension) { extension_ = extension; }

  // A list of all the filenames we've written.
  //
  // This only includes the part after base_name().
  const std::vector<std::string> &all_filenames() const {
    return all_filenames_;
  }

  void Rotate(const Node *node) override;

  void WriteConfiguration(
      aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader> *header,
      std::string_view config_sha256) override;

  NewDataWriter *MakeWriter(const Channel *channel) override;

  NewDataWriter *MakeForwardedTimestampWriter(const Channel *channel,
                                              const Node *node) override;

  NewDataWriter *MakeTimestampWriter(const Channel *channel) override;

  // Indicates that at least one file ran out of space. Once this happens, we
  // stop trying to open new files, to avoid writing any files with holes from
  // previous parts.
  //
  // Besides this function, this object will silently stop logging data when
  // this occurs. If you want to ensure log files are complete, you must call
  // this method.
  bool ran_out_of_space() const {
    return accumulate_data_writers<bool>(
        ran_out_of_space_, [](bool x, const NewDataWriter &data_writer) {
          CHECK_NOTNULL(data_writer.writer);
          return x ||
                 (data_writer.writer && data_writer.writer->ran_out_of_space());
        });
  }

  // Returns the maximum total_bytes() value for all existing
  // DetachedBufferWriters.
  //
  // Returns 0 if no files are open.
  size_t maximum_total_bytes() const {
    return accumulate_data_writers<size_t>(
        0, [](size_t x, const NewDataWriter &data_writer) {
          CHECK_NOTNULL(data_writer.writer);
          return std::max(x, data_writer.writer->total_bytes());
        });
  }

  // Closes all existing log files. No more data may be written after this.
  //
  // This may set ran_out_of_space().
  WriteCode Close() override;

  // Accessors for various statistics. See the identically-named methods in
  // DetachedBufferWriter for documentation. These are aggregated across all
  // past and present DetachedBufferWriters.
  std::chrono::nanoseconds max_write_time() const {
    return accumulate_data_writers(
        max_write_time_,
        [](std::chrono::nanoseconds x, const NewDataWriter &data_writer) {
          CHECK_NOTNULL(data_writer.writer);
          return std::max(
              x, data_writer.writer->WriteStatistics()->max_write_time());
        });
  }
  int max_write_time_bytes() const {
    return std::get<0>(accumulate_data_writers(
        std::make_tuple(max_write_time_bytes_, max_write_time_),
        [](std::tuple<int, std::chrono::nanoseconds> x,
           const NewDataWriter &data_writer) {
          CHECK_NOTNULL(data_writer.writer);
          if (data_writer.writer->WriteStatistics()->max_write_time() >
              std::get<1>(x)) {
            return std::make_tuple(
                data_writer.writer->WriteStatistics()->max_write_time_bytes(),
                data_writer.writer->WriteStatistics()->max_write_time());
          }
          return x;
        }));
  }
  int max_write_time_messages() const {
    return std::get<0>(accumulate_data_writers(
        std::make_tuple(max_write_time_messages_, max_write_time_),
        [](std::tuple<int, std::chrono::nanoseconds> x,
           const NewDataWriter &data_writer) {
          CHECK_NOTNULL(data_writer.writer);
          if (data_writer.writer->WriteStatistics()->max_write_time() >
              std::get<1>(x)) {
            return std::make_tuple(
                data_writer.writer->WriteStatistics()
                    ->max_write_time_messages(),
                data_writer.writer->WriteStatistics()->max_write_time());
          }
          return x;
        }));
  }
  std::chrono::nanoseconds total_write_time() const {
    return accumulate_data_writers(
        total_write_time_,
        [](std::chrono::nanoseconds x, const NewDataWriter &data_writer) {
          CHECK_NOTNULL(data_writer.writer);
          return x + data_writer.writer->WriteStatistics()->total_write_time();
        });
  }
  int total_write_count() const {
    return accumulate_data_writers(
        total_write_count_, [](int x, const NewDataWriter &data_writer) {
          CHECK_NOTNULL(data_writer.writer);
          return x + data_writer.writer->WriteStatistics()->total_write_count();
        });
  }
  int total_write_messages() const {
    return accumulate_data_writers(
        total_write_messages_, [](int x, const NewDataWriter &data_writer) {
          return x +
                 data_writer.writer->WriteStatistics()->total_write_messages();
        });
  }
  int total_write_bytes() const {
    return accumulate_data_writers(
        total_write_bytes_, [](int x, const NewDataWriter &data_writer) {
          CHECK_NOTNULL(data_writer.writer);
          return x + data_writer.writer->WriteStatistics()->total_write_bytes();
        });
  }

  void ResetStatistics();

 protected:
  // TODO (Alexei): consider to move ownership of log_namer to concrete sub
  // class and make log_backend_ raw pointer.
  LogBackend *log_backend() { return log_backend_.get(); }
  const LogBackend *log_backend() const { return log_backend_.get(); }

  // Returns the data writer or timestamp writer if we find one for the provided
  // node.
  NewDataWriter *FindNodeDataWriter(const Node *node, size_t max_message_size);
  NewDataWriter *FindNodeTimestampWriter(const Node *node,
                                         size_t max_message_size);

  // Saves the data writer or timestamp writer for the provided node.
  NewDataWriter *AddNodeDataWriter(const Node *node, NewDataWriter &&writer);
  NewDataWriter *AddNodeTimestampWriter(const Node *node,
                                        NewDataWriter &&writer);

  void CloseWriter(std::unique_ptr<DetachedBufferWriter> *writer_pointer);

  void CreateBufferWriter(std::string_view path, size_t max_message_size,
                          std::unique_ptr<DetachedBufferWriter> *destination);

  std::string extension_;

 private:
  // Opens up a writer for timestamps forwarded back.
  void OpenForwardedTimestampWriter(const Node *source_node,
                                    NewDataWriter *data_writer);

  // Opens up a writer for remote data.
  void OpenDataWriter(const Node *source_node, NewDataWriter *data_writer);
  void OpenTimestampWriter(NewDataWriter *data_writer);

  // Tracks the node in nodes_.
  void NoticeNode(const Node *source_node);

  // A version of std::accumulate which operates over all of our DataWriters.
  template <typename T, typename BinaryOperation>
  T accumulate_data_writers(T t, BinaryOperation op) const {
    for (const std::pair<const Node *const, NewDataWriter> &data_writer :
         node_data_writers_) {
      if (data_writer.second.writer != nullptr) {
        t = op(std::move(t), data_writer.second);
      }
    }
    for (const std::pair<const Node *const, NewDataWriter> &data_writer :
         node_timestamp_writers_) {
      if (data_writer.second.writer != nullptr) {
        t = op(std::move(t), data_writer.second);
      }
    }
    return t;
  }

  std::unique_ptr<LogBackend> log_backend_;

  bool ran_out_of_space_ = false;
  std::vector<std::string> all_filenames_;

  std::function<std::unique_ptr<DataEncoder>(size_t)> encoder_factory_;

  // Storage for statistics from previously-rotated DetachedBufferWriters.
  std::chrono::nanoseconds max_write_time_ = std::chrono::nanoseconds::zero();
  int max_write_time_bytes_ = -1;
  int max_write_time_messages_ = -1;
  std::chrono::nanoseconds total_write_time_ = std::chrono::nanoseconds::zero();
  int total_write_count_ = 0;
  int total_write_messages_ = 0;
  int total_write_bytes_ = 0;

  // Data writer per remote node.
  std::map<const Node *, NewDataWriter> node_data_writers_;
  // Remote timestamp writers per node.
  std::map<const Node *, NewDataWriter> node_timestamp_writers_;
};

// This is specialized log namer that deals with directory centric log events.
class MultiNodeFilesLogNamer : public MultiNodeLogNamer {
 public:
  MultiNodeFilesLogNamer(std::string_view base_name, EventLoop *event_loop)
      : MultiNodeLogNamer(
            std::make_unique<RenamableFileBackend>(base_name, false),
            event_loop) {}

  MultiNodeFilesLogNamer(std::string_view base_name,
                         const Configuration *configuration,
                         EventLoop *event_loop, const Node *node)
      : MultiNodeLogNamer(
            std::make_unique<RenamableFileBackend>(base_name, false),
            configuration, event_loop, node) {}

  MultiNodeFilesLogNamer(EventLoop *event_loop,
                         std::unique_ptr<RenamableFileBackend> backend)
      : MultiNodeLogNamer(std::move(backend), event_loop) {}

  ~MultiNodeFilesLogNamer() override = default;

  std::string_view base_name() const {
    return renamable_file_backend()->base_name();
  }

  // Rotate should be called at least once in between calls to set_base_name.
  // Otherwise, temporary files will not be recoverable.
  // Rotate is called by Logger::RenameLogBase, which is currently the only user
  // of this method.
  // Only renaming the folder is supported, not the file base name.
  void set_base_name(std::string_view base_name) {
    renamable_file_backend()->RenameLogBase(base_name);
  }

  // When enabled, this will write files under names beginning
  // with the .tmp suffix, and then rename them to the desired name after
  // they are fully written.
  //
  // This is useful to enable incremental copying of the log files.
  //
  // Defaults to writing directly to the final filename.
  void EnableTempFiles() { renamable_file_backend()->EnableTempFiles(); }

 private:
  RenamableFileBackend *renamable_file_backend() {
    return reinterpret_cast<RenamableFileBackend *>(log_backend());
  }
  const RenamableFileBackend *renamable_file_backend() const {
    return reinterpret_cast<const RenamableFileBackend *>(log_backend());
  }
};

// Class which dumps all data from each node into a single file per node.  This
// is mostly interesting for testing.
class MinimalFileMultiNodeLogNamer : public MultiNodeFilesLogNamer {
 public:
  MinimalFileMultiNodeLogNamer(std::string_view base_name,
                               EventLoop *event_loop)
      : MultiNodeFilesLogNamer(base_name, event_loop) {}
  MinimalFileMultiNodeLogNamer(std::string_view base_name,
                               const Configuration *configuration,
                               EventLoop *event_loop, const Node *node)
      : MultiNodeFilesLogNamer(base_name, configuration, event_loop, node) {}

  NewDataWriter *MakeWriter(const Channel *channel) override;

  NewDataWriter *MakeForwardedTimestampWriter(const Channel *channel,
                                              const Node *node) override;

  NewDataWriter *MakeTimestampWriter(const Channel *channel) override;

 private:
  // Names the data writer.
  void OpenNodeWriter(const Node *source_node, NewDataWriter *data_writer);
};

}  // namespace logger
}  // namespace aos

#endif  // AOS_EVENTS_LOGGING_LOG_NAMER_H_

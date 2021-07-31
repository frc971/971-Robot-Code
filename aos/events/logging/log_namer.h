#ifndef AOS_EVENTS_LOGGING_LOG_NAMER_H_
#define AOS_EVENTS_LOGGING_LOG_NAMER_H_

#include <functional>
#include <map>
#include <memory>
#include <string_view>
#include <vector>

#include "aos/events/logging/logfile_utils.h"
#include "aos/events/logging/logger_generated.h"
#include "aos/uuid.h"
#include "flatbuffers/flatbuffers.h"

namespace aos {
namespace logger {

class LogNamer;

// TODO(austin): Rename this back to DataWriter once all other callers are of
// the old DataWriter.
//
// Class to manage writing data to log files.  This lets us track which boot the
// written header has in it, and if the header has been written or not.
class NewDataWriter {
 public:
  // Constructs a NewDataWriter.
  // log_namer is the log namer which holds the config and any other data we
  // need for our header.
  // node is the node whom's prespective we are logging from.
  // reopen is called whenever a file needs to be reopened.
  // close is called to close that file and extract any statistics.
  NewDataWriter(LogNamer *log_namer, const Node *node,
                std::function<void(NewDataWriter *)> reopen,
                std::function<void(NewDataWriter *)> close);

  NewDataWriter(NewDataWriter &&other) = default;
  aos::logger::NewDataWriter &operator=(NewDataWriter &&other) = default;
  NewDataWriter(const NewDataWriter &) = delete;
  void operator=(const NewDataWriter &) = delete;

  ~NewDataWriter();

  // Rotates the log file, delaying writing the new header until data arrives.
  void Rotate();

  // TODO(austin): Copy header and add all UUIDs and such when available
  // whenever data is written.
  //
  // TODO(austin): Add known timestamps for each node every time we cycle a log
  // for sorting.

  // Queues up a message with the provided boot UUID.
  void QueueSizedFlatbuffer(flatbuffers::FlatBufferBuilder *fbb,
                            const UUID &source_node_boot_uuid,
                            aos::monotonic_clock::time_point now);

  // Returns the filename of the writer.
  std::string_view filename() const { return writer->filename(); }

  // Signals that a node has rebooted.
  void Reboot();

  void Close();

  std::unique_ptr<DetachedBufferWriter> writer = nullptr;

  size_t node_index() const { return node_index_; }
  const UUID &parts_uuid() const { return parts_uuid_; }
  size_t parts_index() const { return parts_index_; }
  const Node *node() const { return node_; }

 private:
  void QueueHeader(
      aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader> &&header);

  const Node *const node_ = nullptr;
  size_t node_index_ = 0;
  LogNamer *log_namer_;
  UUID parts_uuid_ = UUID::Random();
  size_t parts_index_ = 0;

  std::function<void(NewDataWriter *)> reopen_;
  std::function<void(NewDataWriter *)> close_;
  bool header_written_ = false;
  UUID source_node_boot_uuid_ = UUID::Zero();
};

// Interface describing how to name, track, and add headers to log file parts.
class LogNamer {
 public:
  // Constructs a LogNamer with the primary node (ie the one the logger runs on)
  // being node.
  LogNamer(const Configuration *configuration, const Node *node)
      : configuration_(configuration), node_(node) {
    nodes_.emplace_back(node_);
    node_states_.resize(configuration::NodesCount(configuration_));
  }
  virtual ~LogNamer() {}

  virtual std::string_view base_name() const = 0;

  // Rotate should be called at least once in between calls to set_base_name.
  // Otherwise temporary files will not be recoverable.
  // Rotate is called by Logger::RenameLogBase, which is currently the only user
  // of this method.
  // Only renaming the folder is supported, not the file base name.
  virtual void set_base_name(std::string_view base_name) = 0;

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

  // Returns the node the logger is running on.
  const Node *node() const { return node_; }

  // Writes out the nested Configuration object to the config file location.
  virtual void WriteConfiguration(
      aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader> *header,
      std::string_view config_sha256) = 0;

  void SetHeaderTemplate(
      aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader> header) {
    header_ = std::move(header);
  }

  void SetStartTimes(size_t node_index,
                     monotonic_clock::time_point monotonic_start_time,
                     realtime_clock::time_point realtime_start_time,
                     monotonic_clock::time_point logger_monotonic_start_time,
                     realtime_clock::time_point logger_realtime_start_time) {
    node_states_[node_index].monotonic_start_time = monotonic_start_time;
    node_states_[node_index].realtime_start_time = realtime_start_time;
    node_states_[node_index].logger_monotonic_start_time =
        logger_monotonic_start_time;
    node_states_[node_index].logger_realtime_start_time =
        logger_realtime_start_time;

    // TODO(austin): Track that the header has changed and needs to be
    // rewritten down here rather than up in log_writer.
  }

  monotonic_clock::time_point monotonic_start_time(size_t node_index) const {
    return node_states_[node_index].monotonic_start_time;
  }

 protected:
  // Creates a new header by copying fields out of the template and combining
  // them with the arguments provided.
  aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader> MakeHeader(
      size_t node_index, const UUID &source_node_boot_uuid,
      const UUID &parts_uuid, int parts_index) const;

  const Configuration *const configuration_;
  const Node *const node_;
  std::vector<const Node *> nodes_;

  friend NewDataWriter;

  // Structure with state per node about times and such.
  // TODO(austin): some of this lives better in NewDataWriter once we move
  // ownership of deciding when to write headers into LogNamer.
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
  std::vector<NodeState> node_states_;

  aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader> header_ =
      aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader>::Empty();
};

// Local log namer is a simple version which only names things
// "base_name.part#.bfbs" and increments the part number.  It doesn't support
// any other log type.
class LocalLogNamer : public LogNamer {
 public:
  LocalLogNamer(std::string_view base_name, const Configuration *configuration,
                const Node *node)
      : LogNamer(configuration, node),
        base_name_(base_name),
        data_writer_(this, node,
                     [this](NewDataWriter *writer) {
                       writer->writer = std::make_unique<DetachedBufferWriter>(
                           absl::StrCat(base_name_, ".part",
                                        writer->parts_index(), ".bfbs"),
                           std::make_unique<aos::logger::DummyEncoder>());
                     },
                     [](NewDataWriter * /*writer*/) {}) {}

  LocalLogNamer(const LocalLogNamer &) = delete;
  LocalLogNamer(LocalLogNamer &&) = delete;
  LocalLogNamer &operator=(const LocalLogNamer &) = delete;
  LocalLogNamer &operator=(LocalLogNamer &&) = delete;

  ~LocalLogNamer() override = default;

  std::string_view base_name() const final { return base_name_; }

  void set_base_name(std::string_view base_name) final {
    base_name_ = base_name;
  }

  NewDataWriter *MakeWriter(const Channel *channel) override;

  void Rotate(const Node *node) override;

  NewDataWriter *MakeTimestampWriter(const Channel *channel) override;

  NewDataWriter *MakeForwardedTimestampWriter(const Channel * /*channel*/,
                                              const Node * /*node*/) override;

  void WriteConfiguration(
      aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader> *header,
      std::string_view config_sha256) override;

 private:
  std::string base_name_;

  NewDataWriter data_writer_;
};

// Log namer which uses a config and a base name to name a bunch of files.
class MultiNodeLogNamer : public LogNamer {
 public:
  MultiNodeLogNamer(std::string_view base_name,
                    const Configuration *configuration, const Node *node);
  ~MultiNodeLogNamer() override;

  std::string_view base_name() const final { return base_name_; }

  void set_base_name(std::string_view base_name) final {
    old_base_name_ = base_name_;
    base_name_ = base_name;
  }

  // If temp_suffix is set, then this will write files under names beginning
  // with the specified suffix, and then rename them to the desired name after
  // they are fully written.
  //
  // This is useful to enable incremental copying of the log files.
  //
  // Defaults to writing directly to the final filename.
  void set_temp_suffix(std::string_view temp_suffix) {
    temp_suffix_ = temp_suffix;
  }

  // Sets the function for creating encoders.
  //
  // Defaults to just creating DummyEncoders.
  void set_encoder_factory(
      std::function<std::unique_ptr<DetachedBufferEncoder>()> encoder_factory) {
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
          return std::max(x, data_writer.writer->total_bytes());
        });
  }

  // Closes all existing log files. No more data may be written after this.
  //
  // This may set ran_out_of_space().
  void Close();

  // Accessors for various statistics. See the identically-named methods in
  // DetachedBufferWriter for documentation. These are aggregated across all
  // past and present DetachedBufferWriters.
  std::chrono::nanoseconds max_write_time() const {
    return accumulate_data_writers(
        max_write_time_,
        [](std::chrono::nanoseconds x, const NewDataWriter &data_writer) {
          return std::max(x, data_writer.writer->max_write_time());
        });
  }
  int max_write_time_bytes() const {
    return std::get<0>(accumulate_data_writers(
        std::make_tuple(max_write_time_bytes_, max_write_time_),
        [](std::tuple<int, std::chrono::nanoseconds> x,
           const NewDataWriter &data_writer) {
          if (data_writer.writer->max_write_time() > std::get<1>(x)) {
            return std::make_tuple(data_writer.writer->max_write_time_bytes(),
                                   data_writer.writer->max_write_time());
          }
          return x;
        }));
  }
  int max_write_time_messages() const {
    return std::get<0>(accumulate_data_writers(
        std::make_tuple(max_write_time_messages_, max_write_time_),
        [](std::tuple<int, std::chrono::nanoseconds> x,
           const NewDataWriter &data_writer) {
          if (data_writer.writer->max_write_time() > std::get<1>(x)) {
            return std::make_tuple(
                data_writer.writer->max_write_time_messages(),
                data_writer.writer->max_write_time());
          }
          return x;
        }));
  }
  std::chrono::nanoseconds total_write_time() const {
    return accumulate_data_writers(
        total_write_time_,
        [](std::chrono::nanoseconds x, const NewDataWriter &data_writer) {
          return x + data_writer.writer->total_write_time();
        });
  }
  int total_write_count() const {
    return accumulate_data_writers(
        total_write_count_, [](int x, const NewDataWriter &data_writer) {
          return x + data_writer.writer->total_write_count();
        });
  }
  int total_write_messages() const {
    return accumulate_data_writers(
        total_write_messages_, [](int x, const NewDataWriter &data_writer) {
          return x + data_writer.writer->total_write_messages();
        });
  }
  int total_write_bytes() const {
    return accumulate_data_writers(
        total_write_bytes_, [](int x, const NewDataWriter &data_writer) {
          return x + data_writer.writer->total_write_bytes();
        });
  }

  void ResetStatistics();

 private:
  // Opens up a writer for timestamps forwarded back.
  void OpenForwardedTimestampWriter(const Channel *channel,
                                    NewDataWriter *data_writer);

  // Opens up a writer for remote data.
  void OpenWriter(const Channel *channel, NewDataWriter *data_writer);

  // Opens the main data writer file for this node responsible for data_writer_.
  void OpenDataWriter();

  void CreateBufferWriter(std::string_view path,
                          std::unique_ptr<DetachedBufferWriter> *destination);

  void RenameTempFile(DetachedBufferWriter *destination);

  void CloseWriter(std::unique_ptr<DetachedBufferWriter> *writer_pointer);

  // A version of std::accumulate which operates over all of our DataWriters.
  template <typename T, typename BinaryOperation>
  T accumulate_data_writers(T t, BinaryOperation op) const {
    for (const std::pair<const Channel *const, NewDataWriter> &data_writer :
         data_writers_) {
      if (!data_writer.second.writer) continue;
      t = op(std::move(t), data_writer.second);
    }
    if (data_writer_) {
      t = op(std::move(t), *data_writer_);
    }
    return t;
  }

  std::string base_name_;
  std::string old_base_name_;

  bool ran_out_of_space_ = false;
  std::vector<std::string> all_filenames_;

  std::string temp_suffix_;
  std::function<std::unique_ptr<DetachedBufferEncoder>()> encoder_factory_ =
      []() { return std::make_unique<DummyEncoder>(); };
  std::string extension_;

  // Storage for statistics from previously-rotated DetachedBufferWriters.
  std::chrono::nanoseconds max_write_time_ = std::chrono::nanoseconds::zero();
  int max_write_time_bytes_ = -1;
  int max_write_time_messages_ = -1;
  std::chrono::nanoseconds total_write_time_ = std::chrono::nanoseconds::zero();
  int total_write_count_ = 0;
  int total_write_messages_ = 0;
  int total_write_bytes_ = 0;

  // File to write both delivery timestamps and local data to.
  std::unique_ptr<NewDataWriter> data_writer_;

  std::map<const Channel *, NewDataWriter> data_writers_;
};

}  // namespace logger
}  // namespace aos

#endif  // AOS_EVENTS_LOGGING_LOG_NAMER_H_

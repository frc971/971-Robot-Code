#ifndef AOS_EVENTS_LOGGING_LOG_NAMER_H_
#define AOS_EVENTS_LOGGING_LOG_NAMER_H_

#include <functional>
#include <map>
#include <memory>
#include <string_view>
#include <vector>

#include "aos/events/logging/logfile_utils.h"
#include "aos/events/logging/logger_generated.h"
#include "aos/events/logging/uuid.h"
#include "flatbuffers/flatbuffers.h"

namespace aos {
namespace logger {

// Interface describing how to name, track, and add headers to log file parts.
class LogNamer {
 public:
  // Constructs a LogNamer with the primary node (ie the one the logger runs on)
  // being node.
  LogNamer(const Node *node) : node_(node) { nodes_.emplace_back(node_); }
  virtual ~LogNamer() {}

  // Writes the header to all log files for a specific node.  This function
  // needs to be called after all the writers are created.
  //
  // Modifies header to contain the uuid and part number for each writer as it
  // writes it.  Since this is done unconditionally, it does not restore the
  // previous value at the end.
  virtual void WriteHeader(
      aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader> *header,
      const Node *node) = 0;

  // Returns a writer for writing data from messages on this channel (on the
  // primary node).
  //
  // The returned pointer will stay valid across rotations, but the object it
  // points to will be assigned to.
  virtual DetachedBufferWriter *MakeWriter(const Channel *channel) = 0;

  // Returns a writer for writing timestamps from messages on this channel (on
  // the primary node).
  //
  // The returned pointer will stay valid across rotations, but the object it
  // points to will be assigned to.
  virtual DetachedBufferWriter *MakeTimestampWriter(const Channel *channel) = 0;

  // Returns a writer for writing timestamps delivered over the special
  // /aos/remote_timestamps/* channels.  node is the node that the timestamps
  // are forwarded back from (to the primary node).
  //
  // The returned pointer will stay valid across rotations, but the object it
  // points to will be assigned to.
  virtual DetachedBufferWriter *MakeForwardedTimestampWriter(
      const Channel *channel, const Node *node) = 0;

  // Rotates all log files for the provided node.  The provided header will be
  // modified and written per WriteHeader above.
  virtual void Rotate(
      const Node *node,
      aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader> *header) = 0;

  // Reboots all log files for the provided node.  The provided header will be
  // modified and written per WriteHeader above.  Resets any parts UUIDs.
  virtual void Reboot(
      const Node *node,
      aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader> *header) = 0;

  // Returns all the nodes that data is being written for.
  const std::vector<const Node *> &nodes() const { return nodes_; }

  // Returns the node the logger is running on.
  const Node *node() const { return node_; }

 protected:
  // Modifies the header to have the provided UUID and part id.
  void UpdateHeader(
      aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader> *header,
      const UUID &uuid, int part_id) const;

  const Node *const node_;
  std::vector<const Node *> nodes_;
};

// Local log namer is a simple version which only names things
// "base_name.part#.bfbs" and increments the part number.  It doesn't support
// any other log type.
class LocalLogNamer : public LogNamer {
 public:
  LocalLogNamer(std::string_view base_name, const Node *node)
      : LogNamer(node),
        base_name_(base_name),
        uuid_(UUID::Random()),
        data_writer_(OpenDataWriter()) {}
  ~LocalLogNamer() override = default;

  void WriteHeader(
      aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader> *header,
      const Node *node) override;

  DetachedBufferWriter *MakeWriter(const Channel *channel) override;

  void Rotate(const Node *node,
              aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader> *header)
      override;

  void Reboot(const Node *node,
              aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader> *header)
      override;

  DetachedBufferWriter *MakeTimestampWriter(const Channel *channel) override;

  DetachedBufferWriter *MakeForwardedTimestampWriter(
      const Channel * /*channel*/, const Node * /*node*/) override;

 private:
  // Creates a new data writer with the new part number.
  std::unique_ptr<DetachedBufferWriter> OpenDataWriter() {
    return std::make_unique<DetachedBufferWriter>(
        absl::StrCat(base_name_, ".part", part_number_, ".bfbs"),
        std::make_unique<aos::logger::DummyEncoder>());
  }

  const std::string base_name_;
  const UUID uuid_;
  size_t part_number_ = 0;
  std::unique_ptr<DetachedBufferWriter> data_writer_;
};

// Log namer which uses a config and a base name to name a bunch of files.
class MultiNodeLogNamer : public LogNamer {
 public:
  MultiNodeLogNamer(std::string_view base_name,
                    const Configuration *configuration, const Node *node);
  ~MultiNodeLogNamer() override;

  std::string_view base_name() const { return base_name_; }

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

  void WriteHeader(
      aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader> *header,
      const Node *node) override;

  void Rotate(const Node *node,
              aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader> *header)
      override;

  void Reboot(const Node *node,
              aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader> *header)
      override;

  DetachedBufferWriter *MakeWriter(const Channel *channel) override;

  DetachedBufferWriter *MakeForwardedTimestampWriter(const Channel *channel,
                                                     const Node *node) override;

  DetachedBufferWriter *MakeTimestampWriter(const Channel *channel) override;

  // Indicates that at least one file ran out of space. Once this happens, we
  // stop trying to open new files, to avoid writing any files with holes from
  // previous parts.
  //
  // Besides this function, this object will silently stop logging data when
  // this occurs. If you want to ensure log files are complete, you must call
  // this method.
  bool ran_out_of_space() const {
    return accumulate_data_writers<bool>(
        ran_out_of_space_, [](bool x, const DataWriter &data_writer) {
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
        0, [](size_t x, const DataWriter &data_writer) {
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
        [](std::chrono::nanoseconds x, const DataWriter &data_writer) {
          return std::max(x, data_writer.writer->max_write_time());
        });
  }
  int max_write_time_bytes() const {
    return std::get<0>(accumulate_data_writers(
        std::make_tuple(max_write_time_bytes_, max_write_time_),
        [](std::tuple<int, std::chrono::nanoseconds> x,
           const DataWriter &data_writer) {
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
           const DataWriter &data_writer) {
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
        [](std::chrono::nanoseconds x, const DataWriter &data_writer) {
          return x + data_writer.writer->total_write_time();
        });
  }
  int total_write_count() const {
    return accumulate_data_writers(
        total_write_count_, [](int x, const DataWriter &data_writer) {
          return x + data_writer.writer->total_write_count();
        });
  }
  int total_write_messages() const {
    return accumulate_data_writers(
        total_write_messages_, [](int x, const DataWriter &data_writer) {
          return x + data_writer.writer->total_write_messages();
        });
  }
  int total_write_bytes() const {
    return accumulate_data_writers(
        total_write_bytes_, [](int x, const DataWriter &data_writer) {
          return x + data_writer.writer->total_write_bytes();
        });
  }

  void ResetStatistics();

 private:
  // Files to write remote data to.  We want one per channel.  Maps the channel
  // to the writer, Node, and part number.
  struct DataWriter {
    std::unique_ptr<DetachedBufferWriter> writer = nullptr;
    const Node *node;
    size_t part_number = 0;
    UUID uuid = UUID::Random();
    std::function<void(const Channel *, DataWriter *)> rotate;
  };

  // Implements Rotate and Reboot, controlled by the 'reboot' flag.  The only
  // difference between the two is if DataWriter::uuid is reset or not.
  void DoRotate(
      const Node *node,
      aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader> *header,
      bool reboot);

  // Opens up a writer for timestamps forwarded back.
  void OpenForwardedTimestampWriter(const Channel *channel,
                                    DataWriter *data_writer);

  // Opens up a writer for remote data.
  void OpenWriter(const Channel *channel, DataWriter *data_writer);

  // Opens the main data writer file for this node responsible for data_writer_.
  void OpenDataWriter();

  void CreateBufferWriter(std::string_view path,
                          std::unique_ptr<DetachedBufferWriter> *destination);

  void RenameTempFile(DetachedBufferWriter *destination);

  void CloseWriter(std::unique_ptr<DetachedBufferWriter> *writer_pointer);

  // A version of std::accumulate which operates over all of our DataWriters.
  template <typename T, typename BinaryOperation>
  T accumulate_data_writers(T t, BinaryOperation op) const {
    for (const std::pair<const Channel *const, DataWriter> &data_writer :
         data_writers_) {
      t = op(std::move(t), data_writer.second);
    }
    if (data_writer_.writer) {
      t = op(std::move(t), data_writer_);
    }
    return t;
  }

  const std::string base_name_;
  const Configuration *const configuration_;

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
  DataWriter data_writer_;

  std::map<const Channel *, DataWriter> data_writers_;
};

}  // namespace logger
}  // namespace aos

#endif  // AOS_EVENTS_LOGGING_LOG_NAMER_H_

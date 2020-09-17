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

  void WriteHeader(
      aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader> *header,
      const Node *node) override;

  DetachedBufferWriter *MakeWriter(const Channel *channel) override;

  void Rotate(const Node *node,
              aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader> *header)
      override;

  DetachedBufferWriter *MakeTimestampWriter(const Channel *channel) override;

  DetachedBufferWriter *MakeForwardedTimestampWriter(
      const Channel * /*channel*/, const Node * /*node*/) override;

 private:
  // Creates a new data writer with the new part number.
  std::unique_ptr<DetachedBufferWriter> OpenDataWriter() {
    return std::make_unique<DetachedBufferWriter>(
        absl::StrCat(base_name_, ".part", part_number_, ".bfbs"));
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

  void WriteHeader(
      aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader> *header,
      const Node *node) override;

  void Rotate(const Node *node,
              aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader> *header)
      override;

  DetachedBufferWriter *MakeWriter(const Channel *channel) override;

  DetachedBufferWriter *MakeForwardedTimestampWriter(
      const Channel *channel, const Node *node) override;

  DetachedBufferWriter *MakeTimestampWriter(const Channel *channel) override;

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

  // Opens up a writer for timestamps forwarded back.
  void OpenForwardedTimestampWriter(const Channel *channel,
                                    DataWriter *data_writer);

  // Opens up a writer for remote data.
  void OpenWriter(const Channel *channel, DataWriter *data_writer);

  // Opens the main data writer file for this node responsible for data_writer_.
  std::unique_ptr<DetachedBufferWriter> OpenDataWriter();

  const std::string base_name_;
  const Configuration *const configuration_;
  const UUID uuid_;

  size_t part_number_ = 0;

  // File to write both delivery timestamps and local data to.
  std::unique_ptr<DetachedBufferWriter> data_writer_;

  std::map<const Channel *, DataWriter> data_writers_;
};

}  // namespace logger
}  // namespace aos

#endif  // AOS_EVENTS_LOGGING_LOG_NAMER_H_

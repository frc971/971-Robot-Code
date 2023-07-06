#ifndef AOS_UTIL_MCAP_LOGGER_H_
#define AOS_UTIL_MCAP_LOGGER_H_

#include "single_include/nlohmann/json.hpp"

#include "aos/configuration_generated.h"
#include "aos/events/event_loop.h"
#include "aos/fast_string_builder.h"
#include "aos/flatbuffer_utils.h"

namespace aos {

// Produces a JSON Schema (https://json-schema.org/) for a given flatbuffer
// type. If recursion_level is set, will include a $schema attribute indicating
// the schema definition being used (this is used to allow for recursion).
//
// Note that this is pretty bare-bones, so, e.g., we don't distinguish between
// structs and tables when generating the JSON schema, so we don't bother to
// mark struct fields as required.
enum class JsonSchemaRecursion {
  kTopLevel,
  kNested,
};
nlohmann::json JsonSchemaForFlatbuffer(
    const FlatbufferType &type,
    JsonSchemaRecursion recursion_level = JsonSchemaRecursion::kTopLevel);

// Returns the shortest possible alias for the specified channel on the
// specified node/application.
std::string ShortenedChannelName(const aos::Configuration *config,
                                 const aos::Channel *channel,
                                 std::string_view application_name,
                                 const aos::Node *node);

// Generates an MCAP file, per the specification at
// https://github.com/foxglove/mcap/tree/main/docs/specification
// This currently generates an uncompressed logfile with full message indexing
// available, to be able to support Foxglove fully.
class McapLogger {
 public:
  // Whether to serialize the messages into the MCAP file as JSON or
  // flatbuffers.
  enum class Serialization {
    kJson,
    kFlatbuffer,
  };
  // Whether to attempt to shorten channel names.
  enum class CanonicalChannelNames {
    // Just use the full, unambiguous, channel names.
    kCanonical,
    // Use GetChannelAliases() to determine the shortest possible name for the
    // channel for the current node, and use that in the MCAP file. This makes
    // it so that the channels in the resulting file are more likely to match
    // the channel names that are used in "real" applications.
    kShortened,
  };
  // Chunk compression to use in the MCAP file.
  enum class Compression {
    kNone,
    kLz4,
  };
  McapLogger(EventLoop *event_loop, const std::string &output_path,
             Serialization serialization,
             CanonicalChannelNames canonical_channels, Compression compression);
  ~McapLogger();

 private:
  enum class OpCode {
    kHeader = 0x01,
    kFooter = 0x02,
    kSchema = 0x03,
    kChannel = 0x04,
    kMessage = 0x05,
    kChunk = 0x06,
    kMessageIndex = 0x07,
    kChunkIndex = 0x08,
    kAttachment = 0x09,
    kAttachmentIndex = 0x0A,
    kStatistics = 0x0B,
    kMetadata = 0x0C,
    kMetadataIndex = 0x0D,
    kSummaryOffset = 0x0E,
    kDataEnd = 0x0F,
  };
  // Stores information associated with a SummaryOffset entry (an offset to the
  // start of a section within Summary section, which allows readers to quickly
  // find all the indices/channel definitions/etc. for a given log).
  struct SummaryOffset {
    OpCode op_code;
    // Offset from the start of the file.
    uint64_t offset;
    // Total length of the section, in bytes.
    uint64_t size;
  };
  // Information needed to build a ChunkIndex entry.
  struct ChunkIndex {
    // Earliest and latest message times within the Chunk being referenced.
    aos::monotonic_clock::time_point start_time;
    aos::monotonic_clock::time_point end_time;
    // Offset from the start of the file to the start of the relevant Chunk.
    uint64_t offset;
    // Total size of the Chunk, in bytes.
    uint64_t chunk_size;
    // Total uncompressed size of the records portion of the Chunk, in bytes.
    uint64_t records_size;
    // Total size of the records portion of the Chunk, when compressed
    uint64_t records_size_compressed;
    // Mapping of channel IDs to the MessageIndex entry for that channel within
    // the referenced Chunk. The MessageIndex is referenced by an offset from
    // the start of the file.
    std::map<uint16_t, uint64_t> message_index_offsets;
    // Total size, in bytes, of all the MessageIndex entries for this Chunk
    // together (note that they are required to be contiguous).
    uint64_t message_index_size;
    // Compression used in this Chunk.
    Compression compression;
  };
  // Maintains the state of a single Chunk. In order to maximize read
  // performance, we currently maintain separate chunks for each channel so
  // that, in order to read a given channel, only data associated with that
  // channel nead be read.
  struct ChunkStatus {
    // Buffer containing serialized message data for the currently-being-built
    // chunk.
    std::stringstream data;
    // Earliest message observed in this chunk.
    std::optional<aos::monotonic_clock::time_point> earliest_message;
    // Latest message observed in this chunk.
    std::optional<aos::monotonic_clock::time_point> latest_message;
    // MessageIndex's for each message. The std::map is indexed by channel ID.
    // The vector is then a series of pairs of (timestamp, offset from start of
    // data).
    // Note that currently this will only ever have one entry, for the channel
    // that this chunk corresponds to. However, the standard provides for there
    // being more than one channel per chunk and so we still have some code that
    // supports that.
    std::map<uint16_t, std::vector<std::pair<uint64_t, uint64_t>>>
        message_indices;
  };
  enum class RegisterHandlers { kYes, kNo };
  // Helpers to write each type of relevant record.
  void WriteMagic();
  void WriteHeader();
  void WriteFooter(uint64_t summary_offset, uint64_t summary_offset_offset);
  void WriteDataEnd();
  void WriteSchema(const uint16_t id, const aos::Channel *channel);
  void WriteChannel(const uint16_t id, const uint16_t schema_id,
                    const aos::Channel *channel,
                    std::string_view override_name = "");
  void WriteMessage(uint16_t channel_id, const Channel *channel,
                    const Context &context, ChunkStatus *chunk);
  void WriteChunk(ChunkStatus *chunk);

  // Writes out the special configuration channel. This gets called right before
  // the first actual message is written so that we can have a reasonable
  // monotonic clock time.
  void WriteConfigurationMessage();

  // The helpers for writing records which appear in the Summary section will
  // return SummaryOffset's so that they can be referenced in the SummaryOffset
  // section.
  SummaryOffset WriteChunkIndices();
  SummaryOffset WriteStatistics();
  std::vector<SummaryOffset> WriteSchemasAndChannels(
      RegisterHandlers register_handlers);
  void WriteSummaryOffset(const SummaryOffset &offset);

  // Writes an MCAP record to the output file.
  void WriteRecord(OpCode op, std::string_view record, std::ostream *ostream);
  void WriteRecord(OpCode op, std::string_view record) {
    WriteRecord(op, record, &output_);
  }
  // Adds an MCAP-spec string/byte-array/map/array of pairs/fixed-size integer
  // to a buffer.
  static void AppendString(FastStringBuilder *builder, std::string_view string);
  static void AppendBytes(FastStringBuilder *builder, std::string_view bytes);
  static void AppendChannelMap(FastStringBuilder *builder,
                               const std::map<uint16_t, uint64_t> &map);
  static void AppendMessageIndices(
      FastStringBuilder *builder,
      const std::vector<std::pair<uint64_t, uint64_t>> &messages);
  static void AppendInt16(FastStringBuilder *builder, uint16_t val);
  static void AppendInt32(FastStringBuilder *builder, uint32_t val);
  static void AppendInt64(FastStringBuilder *builder, uint64_t val);

  aos::EventLoop *event_loop_;
  std::ofstream output_;
  const Serialization serialization_;
  const CanonicalChannelNames canonical_channels_;
  const Compression compression_;
  size_t total_message_bytes_ = 0;
  std::map<const Channel *, size_t> total_channel_bytes_;
  FastStringBuilder string_builder_;

  // Earliest message observed in this logfile.
  std::optional<aos::monotonic_clock::time_point> earliest_message_;
  // Latest message observed in this logfile.
  aos::monotonic_clock::time_point latest_message_ =
      aos::monotonic_clock::min_time;
  // Count of all messages on each channel, indexed by channel ID.
  std::map<uint16_t, uint64_t> message_counts_;
  std::map<uint16_t, std::unique_ptr<RawFetcher>> fetchers_;
  // All currently-being-built chunks. Indexed by channel ID. This is used to
  // segregate channels into separate chunks to support more efficient reading.
  std::map<uint16_t, ChunkStatus> current_chunks_;
  // ChunkIndex's for all fully written Chunks.
  std::vector<ChunkIndex> chunk_indices_;

  // Metadata associated with the fake "configuration" channel that we create in
  // order to ensure that foxglove extensions/users have access to the full
  // configuration.
  uint16_t configuration_id_ = 0;
  FlatbufferDetachedBuffer<Channel> configuration_channel_;
  FlatbufferDetachedBuffer<Configuration> configuration_;
  bool wrote_configuration_ = false;

  // Memory buffer to use for compressing data.
  std::vector<uint8_t> compression_buffer_;
};
}  // namespace aos
#endif  // AOS_UTIL_MCAP_LOGGER_H_

#include "aos/util/mcap_logger.h"

#include "absl/strings/str_replace.h"
#include "single_include/nlohmann/json.hpp"

namespace aos {
namespace {
// Amount of data to allow in each chunk before creating a new chunk.
constexpr size_t kChunkSize = 10000000;
}

nlohmann::json JsonSchemaForFlatbuffer(const FlatbufferType &type,
                                       JsonSchemaRecursion recursion_level) {
  nlohmann::json schema;
  if (recursion_level == JsonSchemaRecursion::kTopLevel) {
    schema["$schema"] = "https://json-schema.org/draft/2020-12/schema";
  }
  schema["type"] = "object";
  nlohmann::json properties;
  for (int index = 0; index < type.NumberFields(); ++index) {
    nlohmann::json field;
    const bool is_array = type.FieldIsRepeating(index);
    if (type.FieldIsSequence(index)) {
      // For sub-tables/structs, just recurse.
      nlohmann::json subtype = JsonSchemaForFlatbuffer(
          type.FieldType(index), JsonSchemaRecursion::kNested);
      if (is_array) {
        field["type"] = "array";
        field["items"] = subtype;
      } else {
        field = subtype;
      }
    } else {
      std::string elementary_type;
      switch (type.FieldElementaryType(index)) {
        case flatbuffers::ET_UTYPE:
        case flatbuffers::ET_CHAR:
        case flatbuffers::ET_UCHAR:
        case flatbuffers::ET_SHORT:
        case flatbuffers::ET_USHORT:
        case flatbuffers::ET_INT:
        case flatbuffers::ET_UINT:
        case flatbuffers::ET_LONG:
        case flatbuffers::ET_ULONG:
        case flatbuffers::ET_FLOAT:
        case flatbuffers::ET_DOUBLE:
          elementary_type = "number";
          break;
        case flatbuffers::ET_BOOL:
          elementary_type = "boolean";
          break;
        case flatbuffers::ET_STRING:
          elementary_type = "string";
          break;
        case flatbuffers::ET_SEQUENCE:
          if (type.FieldIsEnum(index)) {
            elementary_type = "string";
          } else {
            LOG(FATAL) << "Should not encounter any sequence fields here.";
          }
          break;
      }
      if (is_array) {
        field["type"] = "array";
        field["items"]["type"] = elementary_type;
      } else {
        field["type"] = elementary_type;
      }
    }
    // the nlohmann::json [] operator needs an actual string, not just a
    // string_view :(.
    properties[std::string(type.FieldName(index))] = field;
  }
  schema["properties"] = properties;
  return schema;
}

McapLogger::McapLogger(EventLoop *event_loop, const std::string &output_path)
    : event_loop_(event_loop), output_(output_path) {
  event_loop->SkipTimingReport();
  event_loop->SkipAosLog();
  CHECK(output_);
  WriteMagic();
  WriteHeader();
  // Schemas and channels get written out both at the start and end of the file,
  // per the MCAP spec.
  WriteSchemasAndChannels(RegisterHandlers::kYes);
}

McapLogger::~McapLogger() {
  // If we have any data remaining, write one last chunk.
  if (current_chunk_.tellp() > 0) {
    WriteChunk();
  }
  WriteDataEnd();

  // Now we enter the Summary section, where we write out all the channel/index
  // information that readers need to be able to seek to arbitrary locations
  // within the log.
  const uint64_t summary_offset = output_.tellp();
  const SummaryOffset chunk_indices_offset = WriteChunkIndices();
  const SummaryOffset stats_offset = WriteStatistics();
  // Schemas/Channels need to get reproduced in the summary section for random
  // access reading.
  const std::vector<SummaryOffset> offsets =
      WriteSchemasAndChannels(RegisterHandlers::kNo);

  // Next we have the summary offset section, which references the individual
  // pieces of the summary section.
  const uint64_t summary_offset_offset = output_.tellp();

  // SummarytOffset's must all be the final thing before the footer.
  WriteSummaryOffset(chunk_indices_offset);
  WriteSummaryOffset(stats_offset);
  for (const auto &offset : offsets) {
    WriteSummaryOffset(offset);
  }

  // And finally, the footer which must itself reference the start of the
  // summary and summary offset sections.
  WriteFooter(summary_offset, summary_offset_offset);
  WriteMagic();
}

std::vector<McapLogger::SummaryOffset> McapLogger::WriteSchemasAndChannels(
    RegisterHandlers register_handlers) {
  uint16_t id = 1;
  std::map<uint16_t, const Channel *> channels;
  for (const Channel *channel : *event_loop_->configuration()->channels()) {
    if (!configuration::ChannelIsReadableOnNode(channel, event_loop_->node())) {
      continue;
    }
    channels[id] = channel;

    if (register_handlers == RegisterHandlers::kYes) {
      message_counts_[id] = 0;
      event_loop_->MakeRawWatcher(
          channel, [this, id, channel](const Context &context, const void *) {
            WriteMessage(id, channel, context, &current_chunk_);
            if (static_cast<uint64_t>(current_chunk_.tellp()) > kChunkSize) {
              WriteChunk();
            }
          });
    }
    ++id;
  }

  std::vector<SummaryOffset> offsets;

  const uint64_t schema_offset = output_.tellp();

  for (const auto &pair : channels) {
    WriteSchema(pair.first, pair.second);
  }

  const uint64_t channel_offset = output_.tellp();

  offsets.push_back(
      {OpCode::kSchema, schema_offset, channel_offset - schema_offset});

  for (const auto &pair : channels) {
    // Write out the channel entry that uses the schema (we just re-use
    // the schema ID for the channel ID, since we aren't deduplicating
    // schemas for channels that are of the same type).
    WriteChannel(pair.first, pair.first, pair.second);
  }

  offsets.push_back({OpCode::kChannel, channel_offset,
                     static_cast<uint64_t>(output_.tellp()) - channel_offset});
  return offsets;
}

void McapLogger::WriteMagic() { output_ << "\x89MCAP0\r\n"; }

void McapLogger::WriteHeader() {
  string_builder_.Reset();
  // "profile"
  AppendString(&string_builder_, "x-aos");
  // "library"
  AppendString(&string_builder_, "AOS MCAP converter");
  WriteRecord(OpCode::kHeader, string_builder_.Result());
}

void McapLogger::WriteFooter(uint64_t summary_offset,
                             uint64_t summary_offset_offset) {
  string_builder_.Reset();
  AppendInt64(&string_builder_, summary_offset);
  AppendInt64(&string_builder_, summary_offset_offset);
  // CRC32 for the Summary section, which we don't bother populating.
  AppendInt32(&string_builder_, 0);
  WriteRecord(OpCode::kFooter, string_builder_.Result());
}

void McapLogger::WriteDataEnd() {
  string_builder_.Reset();
  // CRC32 for the data, which we are too lazy to calculate.
  AppendInt32(&string_builder_, 0);
  WriteRecord(OpCode::kDataEnd, string_builder_.Result());
}

void McapLogger::WriteSchema(const uint16_t id, const aos::Channel *channel) {
  CHECK(channel->has_schema());
  std::string schema = JsonSchemaForFlatbuffer({channel->schema()}).dump();

  // Write out the schema (we don't bother deduplicating schema types):
  string_builder_.Reset();
  // Schema ID
  AppendInt16(&string_builder_, id);
  // Type name
  AppendString(&string_builder_, channel->type()->string_view());
  // Encoding
  AppendString(&string_builder_, "jsonschema");
  // Actual schema itself
  AppendString(&string_builder_, schema);
  WriteRecord(OpCode::kSchema, string_builder_.Result());
}

void McapLogger::WriteChannel(const uint16_t id, const uint16_t schema_id,
                              const aos::Channel *channel) {
  string_builder_.Reset();
  // Channel ID
  AppendInt16(&string_builder_, id);
  // Schema ID
  AppendInt16(&string_builder_, schema_id);
  // Topic name
  AppendString(&string_builder_,
               absl::StrCat(channel->name()->string_view(), " ",
                            channel->type()->string_view()));
  // Encoding
  AppendString(&string_builder_, "json");
  // Metadata (technically supposed to be a Map<string, string>)
  AppendString(&string_builder_, "");
  WriteRecord(OpCode::kChannel, string_builder_.Result());
}

void McapLogger::WriteMessage(uint16_t channel_id, const Channel *channel,
                              const Context &context, std::ostream *output) {
  CHECK_NOTNULL(context.data);

  message_counts_[channel_id]++;

  if (!earliest_message_.has_value()) {
    earliest_message_ = context.monotonic_event_time;
  }
  if (!earliest_chunk_message_.has_value()) {
    earliest_chunk_message_ = context.monotonic_event_time;
  }
  latest_message_ = context.monotonic_event_time;

  string_builder_.Reset();
  // Channel ID
  AppendInt16(&string_builder_, channel_id);
  // Queue Index
  AppendInt32(&string_builder_, context.queue_index);
  // Log time, and publish time. Since we don't log a logged time, just use
  // published time.
  // TODO(james): If we use this for multi-node logfiles, use distributed clock.
  AppendInt64(&string_builder_,
              context.monotonic_event_time.time_since_epoch().count());
  AppendInt64(&string_builder_,
              context.monotonic_event_time.time_since_epoch().count());

  CHECK(flatbuffers::Verify(*channel->schema(),
                            *channel->schema()->root_table(),
                            static_cast<const uint8_t *>(context.data),
                            static_cast<size_t>(context.size)))
      << ": Corrupted flatbuffer on " << channel->name()->c_str() << " "
      << channel->type()->c_str();

  aos::FlatbufferToJson(&string_builder_, channel->schema(),
                        static_cast<const uint8_t *>(context.data));

  message_indices_[channel_id].push_back(std::make_pair<uint64_t, uint64_t>(
      context.monotonic_event_time.time_since_epoch().count(),
      output->tellp()));

  WriteRecord(OpCode::kMessage, string_builder_.Result(), output);
}

void McapLogger::WriteRecord(OpCode op, std::string_view record,
                             std::ostream *ostream) {
  ostream->put(static_cast<char>(op));
  uint64_t record_length = record.size();
  ostream->write(reinterpret_cast<const char *>(&record_length),
                 sizeof(record_length));
  *ostream << record;
}

void McapLogger::WriteChunk() {
  string_builder_.Reset();

  CHECK(earliest_chunk_message_.has_value());
  const uint64_t chunk_offset = output_.tellp();
  AppendInt64(&string_builder_,
              earliest_chunk_message_->time_since_epoch().count());
  AppendInt64(&string_builder_, latest_message_.time_since_epoch().count());

  std::string chunk_records = current_chunk_.str();
  // Reset the chunk buffer.
  current_chunk_.str("");

  const uint64_t records_size = chunk_records.size();
  // Uncompressed chunk size.
  AppendInt64(&string_builder_, records_size);
  // Uncompressed CRC (unpopulated).
  AppendInt32(&string_builder_, 0);
  AppendString(&string_builder_, "");
  AppendBytes(&string_builder_, chunk_records);
  WriteRecord(OpCode::kChunk, string_builder_.Result());

  std::map<uint16_t, uint64_t> index_offsets;
  const uint64_t message_index_start = output_.tellp();
  for (const auto &indices : message_indices_) {
    index_offsets[indices.first] = output_.tellp();
    string_builder_.Reset();
    AppendInt16(&string_builder_, indices.first);
    AppendMessageIndices(&string_builder_, indices.second);
    WriteRecord(OpCode::kMessageIndex, string_builder_.Result());
  }
  message_indices_.clear();
  chunk_indices_.push_back(ChunkIndex{
      earliest_chunk_message_.value(), latest_message_, chunk_offset,
      message_index_start - chunk_offset, records_size, index_offsets,
      static_cast<uint64_t>(output_.tellp()) - message_index_start});
  earliest_chunk_message_.reset();
}

McapLogger::SummaryOffset McapLogger::WriteStatistics() {
  const uint64_t stats_offset = output_.tellp();
  const uint64_t message_count = std::accumulate(
      message_counts_.begin(), message_counts_.end(), 0,
      [](const uint64_t &count, const std::pair<uint16_t, uint64_t> &val) {
        return count + val.second;
      });
  string_builder_.Reset();
  AppendInt64(&string_builder_, message_count);
  // Schema count.
  AppendInt16(&string_builder_, message_counts_.size());
  // Channel count.
  AppendInt32(&string_builder_, message_counts_.size());
  // Attachment count.
  AppendInt32(&string_builder_, 0);
  // Metadata count.
  AppendInt32(&string_builder_, 0);
  // Chunk count.
  AppendInt32(&string_builder_, chunk_indices_.size());
  // Earliest & latest message times.
  AppendInt64(&string_builder_, earliest_message_->time_since_epoch().count());
  AppendInt64(&string_builder_, latest_message_.time_since_epoch().count());
  // Per-channel message counts.
  AppendChannelMap(&string_builder_, message_counts_);
  WriteRecord(OpCode::kStatistics, string_builder_.Result());
  return {OpCode::kStatistics, stats_offset,
          static_cast<uint64_t>(output_.tellp()) - stats_offset};
}

McapLogger::SummaryOffset McapLogger::WriteChunkIndices() {
  const uint64_t index_offset = output_.tellp();
  for (const ChunkIndex &index : chunk_indices_) {
    string_builder_.Reset();
    AppendInt64(&string_builder_, index.start_time.time_since_epoch().count());
    AppendInt64(&string_builder_, index.end_time.time_since_epoch().count());
    AppendInt64(&string_builder_, index.offset);
    AppendInt64(&string_builder_, index.chunk_size);
    AppendChannelMap(&string_builder_, index.message_index_offsets);
    AppendInt64(&string_builder_, index.message_index_size);
    // Compression used.
    AppendString(&string_builder_, "");
    // Compressed and uncompressed records size.
    AppendInt64(&string_builder_, index.records_size);
    AppendInt64(&string_builder_, index.records_size);
    WriteRecord(OpCode::kChunkIndex, string_builder_.Result());
  }
  return {OpCode::kChunkIndex, index_offset,
          static_cast<uint64_t>(output_.tellp()) - index_offset};
}

void McapLogger::WriteSummaryOffset(const SummaryOffset &offset) {
  string_builder_.Reset();
  string_builder_.AppendChar(static_cast<char>(offset.op_code));
  AppendInt64(&string_builder_, offset.offset);
  AppendInt64(&string_builder_, offset.size);
  WriteRecord(OpCode::kSummaryOffset, string_builder_.Result());
}

void McapLogger::AppendString(FastStringBuilder *builder,
                              std::string_view string) {
  AppendInt32(builder, string.size());
  builder->Append(string);
}

void McapLogger::AppendBytes(FastStringBuilder *builder,
                             std::string_view bytes) {
  AppendInt64(builder, bytes.size());
  builder->Append(bytes);
}

namespace {
template <typename T>
static void AppendInt(FastStringBuilder *builder, T val) {
  builder->Append(
      std::string_view(reinterpret_cast<const char *>(&val), sizeof(T)));
}
template <typename T>
void AppendMap(FastStringBuilder *builder, const T &map) {
  AppendInt<uint32_t>(
      builder, map.size() * (sizeof(typename T::value_type::first_type) +
                             sizeof(typename T::value_type::second_type)));
  for (const auto &pair : map) {
    AppendInt(builder, pair.first);
    AppendInt(builder, pair.second);
  }
}
}  // namespace

void McapLogger::AppendChannelMap(FastStringBuilder *builder,
                                  const std::map<uint16_t, uint64_t> &map) {
  AppendMap(builder, map);
}

void McapLogger::AppendMessageIndices(
    FastStringBuilder *builder,
    const std::vector<std::pair<uint64_t, uint64_t>> &messages) {
  AppendMap(builder, messages);
}

void McapLogger::AppendInt16(FastStringBuilder *builder, uint16_t val) {
  AppendInt(builder, val);
}

void McapLogger::AppendInt32(FastStringBuilder *builder, uint32_t val) {
  AppendInt(builder, val);
}

void McapLogger::AppendInt64(FastStringBuilder *builder, uint64_t val) {
  AppendInt(builder, val);
}
}  // namespace aos

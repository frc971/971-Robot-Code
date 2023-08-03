#include "aos/util/mcap_logger.h"

#include "absl/strings/str_replace.h"
#include "lz4/lz4.h"
#include "lz4/lz4frame.h"
#include "nlohmann/json.hpp"

#include "aos/configuration_schema.h"
#include "aos/flatbuffer_merge.h"

DEFINE_uint64(mcap_chunk_size, 10'000'000,
              "Size, in bytes, of individual MCAP chunks");
DEFINE_bool(fetch, false,
            "Whether to fetch most recent messages at start of logfile. Turn "
            "this on if there are, e.g., one-time messages sent before the "
            "start of the logfile that you need access to. Turn it off if you "
            "don't want to deal with having messages that have timestamps that "
            "may be arbitrarily far before any other interesting messages.");

namespace aos {

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

std::string ShortenedChannelName(const aos::Configuration *config,
                                 const aos::Channel *channel,
                                 std::string_view application_name,
                                 const aos::Node *node) {
  std::set<std::string> names =
      configuration::GetChannelAliases(config, channel, application_name, node);
  std::string_view shortest_name;
  for (const std::string &name : names) {
    if (shortest_name.empty() || name.size() < shortest_name.size()) {
      shortest_name = name;
    }
  }
  return std::string(shortest_name);
}

namespace {
std::string_view CompressionName(McapLogger::Compression compression) {
  switch (compression) {
    case McapLogger::Compression::kNone:
      return "";
    case McapLogger::Compression::kLz4:
      return "lz4";
  }
  LOG(FATAL) << "Unreachable.";
}
}  // namespace

McapLogger::McapLogger(EventLoop *event_loop, const std::string &output_path,
                       Serialization serialization,
                       CanonicalChannelNames canonical_channels,
                       Compression compression)
    : event_loop_(event_loop),
      output_(output_path),
      serialization_(serialization),
      canonical_channels_(canonical_channels),
      compression_(compression),
      configuration_channel_([]() {
        // Set up a fake Channel for providing the configuration in the MCAP
        // file. This is included for convenience so that consumers of the MCAP
        // file can actually dereference things like the channel indices in AOS
        // timing reports.
        flatbuffers::FlatBufferBuilder fbb;
        flatbuffers::Offset<flatbuffers::String> name_offset =
            fbb.CreateString("");
        flatbuffers::Offset<flatbuffers::String> type_offset =
            fbb.CreateString("aos.Configuration");
        flatbuffers::Offset<reflection::Schema> schema_offset =
            aos::CopyFlatBuffer(
                aos::FlatbufferSpan<reflection::Schema>(ConfigurationSchema()),
                &fbb);
        Channel::Builder channel(fbb);
        channel.add_name(name_offset);
        channel.add_type(type_offset);
        channel.add_schema(schema_offset);
        fbb.Finish(channel.Finish());
        return fbb.Release();
      }()),
      configuration_(CopyFlatBuffer(event_loop_->configuration())) {
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
  for (auto &pair : current_chunks_) {
    if (pair.second.data.tellp() > 0) {
      WriteChunk(&pair.second);
    }
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

  // TODO(james): Add compression. With flatbuffers messages that contain large
  // numbers of zeros (e.g., large grids or thresholded images) this can result
  // in massive savings.
  if (VLOG_IS_ON(2)) {
    // For debugging, print out how much space each channel is taking in the
    // overall log.
    LOG(INFO) << total_message_bytes_;
    std::vector<std::pair<size_t, const Channel *>> channel_bytes;
    for (const auto &pair : total_channel_bytes_) {
      channel_bytes.push_back(std::make_pair(pair.second, pair.first));
    }
    std::sort(channel_bytes.begin(), channel_bytes.end());
    for (const auto &pair : channel_bytes) {
      LOG(INFO) << configuration::StrippedChannelToString(pair.second) << ": "
                << static_cast<float>(pair.first) * 1e-6 << "MB "
                << static_cast<float>(pair.first) / total_message_bytes_
                << "\n";
    }
  }
}

std::vector<McapLogger::SummaryOffset> McapLogger::WriteSchemasAndChannels(
    RegisterHandlers register_handlers) {
  uint16_t id = 0;
  std::map<uint16_t, const Channel *> channels;
  for (const Channel *channel : *event_loop_->configuration()->channels()) {
    ++id;
    if (!configuration::ChannelIsReadableOnNode(channel, event_loop_->node())) {
      continue;
    }
    channels[id] = channel;

    if (register_handlers == RegisterHandlers::kYes) {
      message_counts_[id] = 0;
      event_loop_->MakeRawWatcher(
          channel, [this, id, channel](const Context &context, const void *) {
            ChunkStatus *chunk = &current_chunks_[id];
            WriteMessage(id, channel, context, chunk);
            if (static_cast<uint64_t>(chunk->data.tellp()) >
                FLAGS_mcap_chunk_size) {
              WriteChunk(chunk);
            }
          });
      fetchers_[id] = event_loop_->MakeRawFetcher(channel);
      event_loop_->OnRun([this, id, channel]() {
        if (FLAGS_fetch && fetchers_[id]->Fetch()) {
          WriteMessage(id, channel, fetchers_[id]->context(),
                       &current_chunks_[id]);
        }
      });
    }
  }

  // Manually add in a special /configuration channel.
  if (register_handlers == RegisterHandlers::kYes) {
    configuration_id_ = ++id;
  }

  std::vector<SummaryOffset> offsets;

  const uint64_t schema_offset = output_.tellp();

  for (const auto &pair : channels) {
    WriteSchema(pair.first, pair.second);
  }

  WriteSchema(configuration_id_, &configuration_channel_.message());

  const uint64_t channel_offset = output_.tellp();

  offsets.push_back(
      {OpCode::kSchema, schema_offset, channel_offset - schema_offset});

  for (const auto &pair : channels) {
    // Write out the channel entry that uses the schema (we just re-use
    // the schema ID for the channel ID, since we aren't deduplicating
    // schemas for channels that are of the same type).
    WriteChannel(pair.first, pair.first, pair.second);
  }

  // Provide the configuration message on a special channel that is just named
  // "configuration", which is guaranteed not to conflict with existing under
  // our current naming scheme (since our current scheme will, at a minimum, put
  // a space between the name/type of a channel).
  WriteChannel(configuration_id_, configuration_id_,
               &configuration_channel_.message(), "configuration");

  offsets.push_back({OpCode::kChannel, channel_offset,
                     static_cast<uint64_t>(output_.tellp()) - channel_offset});
  return offsets;
}

void McapLogger::WriteConfigurationMessage() {
  Context config_context;
  config_context.monotonic_event_time = event_loop_->monotonic_now();
  config_context.queue_index = 0;
  config_context.size = configuration_.span().size();
  config_context.data = configuration_.span().data();
  // Avoid infinite recursion...
  wrote_configuration_ = true;
  WriteMessage(configuration_id_, &configuration_channel_.message(),
               config_context, &current_chunks_[configuration_id_]);
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

  const FlatbufferDetachedBuffer<reflection::Schema> schema =
      RecursiveCopyFlatBuffer(channel->schema());

  // Write out the schema (we don't bother deduplicating schema types):
  string_builder_.Reset();
  // Schema ID
  AppendInt16(&string_builder_, id);
  // Type name
  AppendString(&string_builder_, channel->type()->string_view());
  switch (serialization_) {
    case Serialization::kJson:
      // Encoding
      AppendString(&string_builder_, "jsonschema");
      // Actual schema itself
      AppendString(&string_builder_,
                   JsonSchemaForFlatbuffer({channel->schema()}).dump());
      break;
    case Serialization::kFlatbuffer:
      // Encoding
      AppendString(&string_builder_, "flatbuffer");
      // Actual schema itself
      AppendString(&string_builder_,
                   {reinterpret_cast<const char *>(schema.span().data()),
                    schema.span().size()});
      break;
  }
  WriteRecord(OpCode::kSchema, string_builder_.Result());
}

void McapLogger::WriteChannel(const uint16_t id, const uint16_t schema_id,
                              const aos::Channel *channel,
                              std::string_view override_name) {
  string_builder_.Reset();
  // Channel ID
  AppendInt16(&string_builder_, id);
  // Schema ID
  AppendInt16(&string_builder_, schema_id);
  // Topic name
  std::string topic_name(override_name);
  if (topic_name.empty()) {
    switch (canonical_channels_) {
      case CanonicalChannelNames::kCanonical:
        topic_name = absl::StrCat(channel->name()->string_view(), " ",
                                  channel->type()->string_view());
        break;
      case CanonicalChannelNames::kShortened: {
        const std::string shortest_name =
            ShortenedChannelName(event_loop_->configuration(), channel,
                                 event_loop_->name(), event_loop_->node());
        if (shortest_name != channel->name()->string_view()) {
          VLOG(1) << "Shortening " << channel->name()->string_view() << " "
                  << channel->type()->string_view() << " to " << shortest_name;
        }
        topic_name =
            absl::StrCat(shortest_name, " ", channel->type()->string_view());
        break;
      }
    }
  }
  AppendString(&string_builder_, topic_name);
  // Encoding
  switch (serialization_) {
    case Serialization::kJson:
      AppendString(&string_builder_, "json");
      break;
    case Serialization::kFlatbuffer:
      AppendString(&string_builder_, "flatbuffer");
      break;
  }

  // Metadata (technically supposed to be a Map<string, string>)
  AppendString(&string_builder_, "");
  WriteRecord(OpCode::kChannel, string_builder_.Result());
}

void McapLogger::WriteMessage(uint16_t channel_id, const Channel *channel,
                              const Context &context, ChunkStatus *chunk) {
  if (!wrote_configuration_) {
    WriteConfigurationMessage();
  }
  CHECK_NOTNULL(context.data);

  message_counts_[channel_id]++;

  if (!earliest_message_.has_value()) {
    earliest_message_ = context.monotonic_event_time;
  } else {
    earliest_message_ =
        std::min(context.monotonic_event_time, earliest_message_.value());
  }
  if (!chunk->earliest_message.has_value()) {
    chunk->earliest_message = context.monotonic_event_time;
  } else {
    chunk->earliest_message =
        std::min(context.monotonic_event_time, chunk->earliest_message.value());
  }
  chunk->latest_message = context.monotonic_event_time;
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
  // Note: Foxglove Studio doesn't appear to actually support using publish time
  // right now.
  AppendInt64(&string_builder_,
              context.monotonic_event_time.time_since_epoch().count());

  CHECK(flatbuffers::Verify(*channel->schema(),
                            *channel->schema()->root_table(),
                            static_cast<const uint8_t *>(context.data),
                            static_cast<size_t>(context.size)))
      << ": Corrupted flatbuffer on " << channel->name()->c_str() << " "
      << channel->type()->c_str();

  switch (serialization_) {
    case Serialization::kJson:
      aos::FlatbufferToJson(&string_builder_, channel->schema(),
                            static_cast<const uint8_t *>(context.data));
      break;
    case Serialization::kFlatbuffer:
      string_builder_.Append(
          {static_cast<const char *>(context.data), context.size});
      break;
  }
  total_message_bytes_ += context.size;
  total_channel_bytes_[channel] += context.size;

  chunk->message_indices[channel_id].push_back(
      std::make_pair<uint64_t, uint64_t>(
          context.monotonic_event_time.time_since_epoch().count(),
          chunk->data.tellp()));

  WriteRecord(OpCode::kMessage, string_builder_.Result(), &chunk->data);
}

void McapLogger::WriteRecord(OpCode op, std::string_view record,
                             std::ostream *ostream) {
  ostream->put(static_cast<char>(op));
  uint64_t record_length = record.size();
  ostream->write(reinterpret_cast<const char *>(&record_length),
                 sizeof(record_length));
  *ostream << record;
}

void McapLogger::WriteChunk(ChunkStatus *chunk) {
  string_builder_.Reset();

  CHECK(chunk->earliest_message.has_value());
  const uint64_t chunk_offset = output_.tellp();
  AppendInt64(&string_builder_,
              chunk->earliest_message->time_since_epoch().count());
  CHECK(chunk->latest_message.has_value());
  AppendInt64(&string_builder_,
              chunk->latest_message.value().time_since_epoch().count());

  std::string chunk_records = chunk->data.str();
  // Reset the chunk buffer.
  chunk->data.str("");

  const uint64_t records_size = chunk_records.size();
  // Uncompressed chunk size.
  AppendInt64(&string_builder_, records_size);
  // Uncompressed CRC (unpopulated).
  AppendInt32(&string_builder_, 0);
  // Compression
  AppendString(&string_builder_, CompressionName(compression_));
  uint64_t records_size_compressed = records_size;
  switch (compression_) {
    case Compression::kNone:
      AppendBytes(&string_builder_, chunk_records);
      break;
    case Compression::kLz4: {
      // Default preferences.
      LZ4F_preferences_t *lz4_preferences = nullptr;
      const uint64_t max_size =
          LZ4F_compressFrameBound(records_size, lz4_preferences);
      CHECK_NE(0u, max_size);
      if (max_size > compression_buffer_.size()) {
        compression_buffer_.resize(max_size);
      }
      records_size_compressed = LZ4F_compressFrame(
          compression_buffer_.data(), compression_buffer_.size(),
          reinterpret_cast<const char *>(chunk_records.data()),
          chunk_records.size(), lz4_preferences);
      CHECK(!LZ4F_isError(records_size_compressed));
      AppendBytes(&string_builder_,
                  {reinterpret_cast<const char *>(compression_buffer_.data()),
                   static_cast<size_t>(records_size_compressed)});
      break;
    }
  }
  WriteRecord(OpCode::kChunk, string_builder_.Result());

  std::map<uint16_t, uint64_t> index_offsets;
  const uint64_t message_index_start = output_.tellp();
  for (const auto &indices : chunk->message_indices) {
    index_offsets[indices.first] = output_.tellp();
    string_builder_.Reset();
    AppendInt16(&string_builder_, indices.first);
    AppendMessageIndices(&string_builder_, indices.second);
    WriteRecord(OpCode::kMessageIndex, string_builder_.Result());
  }
  chunk->message_indices.clear();
  chunk_indices_.push_back(ChunkIndex{
      .start_time = chunk->earliest_message.value(),
      .end_time = chunk->latest_message.value(),
      .offset = chunk_offset,
      .chunk_size = message_index_start - chunk_offset,
      .records_size = records_size,
      .records_size_compressed = records_size_compressed,
      .message_index_offsets = index_offsets,
      .message_index_size =
          static_cast<uint64_t>(output_.tellp()) - message_index_start,
      .compression = compression_});
  chunk->earliest_message.reset();
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
    AppendString(&string_builder_, CompressionName(index.compression));
    // Compressed and uncompressed records size.
    AppendInt64(&string_builder_, index.records_size_compressed);
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

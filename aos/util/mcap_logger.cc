#include "aos/util/mcap_logger.h"

#include "absl/strings/str_replace.h"
#include "single_include/nlohmann/json.hpp"

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

McapLogger::McapLogger(EventLoop *event_loop, const std::string &output_path)
    : output_(output_path) {
  event_loop->SkipTimingReport();
  event_loop->SkipAosLog();
  CHECK(output_);
  WriteMagic();
  WriteHeader();
  uint16_t id = 1;
  for (const Channel *channel : *event_loop->configuration()->channels()) {
    if (!configuration::ChannelIsReadableOnNode(channel, event_loop->node())) {
      continue;
    }

    WriteSchema(id, channel);

    // Write out the channel entry that uses the schema (we just re-use the
    // chema ID for the channel ID, since we aren't deduplicating schemas for
    // channels that are of the same type).
    WriteChannel(id, id, channel);

    event_loop->MakeRawWatcher(
        channel, [this, id, channel](const Context &context, const void *) {
          WriteMessage(id, channel, context);
        });
    ++id;
  }
}

McapLogger::~McapLogger() {
  WriteDataEnd();
  WriteFooter();
  WriteMagic();
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

void McapLogger::WriteFooter() {
  string_builder_.Reset();
  // Offsets and CRC32 for summary section, which we don't populate.
  AppendInt64(&string_builder_, 0);
  AppendInt64(&string_builder_, 0);
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
                              const Context &context) {
  CHECK_NOTNULL(context.data);

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

  WriteRecord(OpCode::kMessage, string_builder_.Result());
}

void McapLogger::WriteRecord(OpCode op, std::string_view record) {
  output_.put(static_cast<char>(op));
  uint64_t record_length = record.size();
  output_.write(reinterpret_cast<const char *>(&record_length),
                sizeof(record_length));
  output_ << record;
}

void McapLogger::AppendString(FastStringBuilder *builder,
                              std::string_view string) {
  AppendInt32(builder, string.size());
  builder->Append(string);
}

namespace {
template <typename T>
static void AppendInt(FastStringBuilder *builder, T val) {
  builder->Append(
      std::string_view(reinterpret_cast<const char *>(&val), sizeof(T)));
}
}  // namespace

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

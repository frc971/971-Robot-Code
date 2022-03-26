#ifndef AOS_UTIL_MCAP_LOGGER_H_
#define AOS_UTIL_MCAP_LOGGER_H_

#include "aos/configuration_generated.h"
#include "aos/events/event_loop.h"
#include "aos/fast_string_builder.h"
#include "aos/flatbuffer_utils.h"
#include "single_include/nlohmann/json.hpp"

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

// Generates an MCAP file, per the specification at
// https://github.com/foxglove/mcap/tree/main/docs/specification
class McapLogger {
 public:
  McapLogger(EventLoop *event_loop, const std::string &output_path);
  ~McapLogger();

 private:
  enum class OpCode {
    kHeader = 0x01,
    kFooter = 0x02,
    kSchema = 0x03,
    kChannel = 0x04,
    kMessage = 0x05,
    kDataEnd = 0x0F,
  };
  // Helpers to write each type of relevant record.
  void WriteMagic();
  void WriteHeader();
  void WriteFooter();
  void WriteDataEnd();
  void WriteSchema(const uint16_t id, const aos::Channel *channel);
  void WriteChannel(const uint16_t id, const uint16_t schema_id,
                    const aos::Channel *channel);
  void WriteMessage(uint16_t channel_id, const Channel *channel,
                    const Context &context);
  void WriteConfig();

  // Writes an MCAP record to the output file.
  void WriteRecord(OpCode op, std::string_view record);
  // Adds an MCAP-spec string/fixed-size integer to a buffer.
  static void AppendString(FastStringBuilder *builder, std::string_view string);
  static void AppendInt16(FastStringBuilder *builder, uint16_t val);
  static void AppendInt32(FastStringBuilder *builder, uint32_t val);
  static void AppendInt64(FastStringBuilder *builder, uint64_t val);

  std::ofstream output_;
  FastStringBuilder string_builder_;
};
}  // namespace aos
#endif  // AOS_UTIL_MCAP_LOGGER_H_

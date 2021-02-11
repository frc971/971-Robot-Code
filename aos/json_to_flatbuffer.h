#ifndef AOS_JSON_TO_FLATBUFFER_H_
#define AOS_JSON_TO_FLATBUFFER_H_

#include <cstddef>
#include <fstream>
#include <string>
#include <string_view>

#include "aos/fast_string_builder.h"
#include "aos/flatbuffer_utils.h"
#include "aos/flatbuffers.h"
#include "flatbuffers/flatbuffers.h"
#include "flatbuffers/reflection.h"

namespace aos {

// Parses the flatbuffer into the buffer, or returns an empty buffer.
flatbuffers::DetachedBuffer JsonToFlatbuffer(std::string_view data,
                                             FlatbufferType type);

// Parses the flatbuffer into the builder, and returns the offset.
flatbuffers::Offset<flatbuffers::Table> JsonToFlatbuffer(
    std::string_view data, FlatbufferType type,
    flatbuffers::FlatBufferBuilder *fbb);

// Typed versions of the above methods.
template <typename T>
inline flatbuffers::DetachedBuffer JsonToFlatbuffer(
    const std::string_view data) {
  return JsonToFlatbuffer(data, FlatbufferType(T::MiniReflectTypeTable()));
}
template <typename T>
inline flatbuffers::Offset<T> JsonToFlatbuffer(
    const std::string_view data, flatbuffers::FlatBufferBuilder *fbb) {
  return flatbuffers::Offset<T>(
      JsonToFlatbuffer(data, FlatbufferType(T::MiniReflectTypeTable()), fbb).o);
}

struct JsonOptions {
  // controls if the Json is written ouut on multiple lines or one.
  bool multi_line = false;
  // the contents of vectors longer than max_vector_size will be skipped.
  size_t max_vector_size = SIZE_MAX;
};

// Converts a flatbuffer into a Json string.
// The methods below are generally more useful than TableFlatbufferToJson.
::std::string TableFlatbufferToJson(const flatbuffers::Table *t,
                                    const ::flatbuffers::TypeTable *typetable,
                                    JsonOptions json_options = {});

// Converts a Flatbuffer<T> holding a flatbuffer to JSON.
template <typename T>
inline ::std::string FlatbufferToJson(const Flatbuffer<T> &flatbuffer,
                                      JsonOptions json_options = {}) {
  return TableFlatbufferToJson(
      reinterpret_cast<const flatbuffers::Table *>(&flatbuffer.message()),
      Flatbuffer<T>::MiniReflectTypeTable(), json_options);
}

// Converts a flatbuffer::Table to JSON.
template <typename T>
typename std::enable_if<
    std::is_base_of<flatbuffers::Table, T>::value,
    std::string>::type inline FlatbufferToJson(const T *flatbuffer,
                                               JsonOptions json_options = {}) {
  return TableFlatbufferToJson(
      reinterpret_cast<const flatbuffers::Table *>(flatbuffer),
      Flatbuffer<T>::MiniReflectTypeTable(), json_options);
}

std::string FlatbufferToJson(const reflection::Schema *schema,
                             const uint8_t *data,
                             JsonOptions json_options = {});

void FlatbufferToJson(FastStringBuilder *builder,
                      const reflection::Schema *schema, const uint8_t *data,
                      JsonOptions json_options = {});

// Writes a Flatbuffer to a file, or dies.
template <typename T>
inline void WriteFlatbufferToJson(std::string_view filename,
                                  const Flatbuffer<T> &msg) {
  std::ofstream json_file(std::string(filename), std::ios::out);
  CHECK(json_file) << ": Couldn't open " << filename;
  json_file << FlatbufferToJson(msg);
  json_file.close();
}

// Writes a NonSizePrefixedFlatbuffer to a binary file, or dies.
template <typename T>
inline void WriteFlatbufferToFile(std::string_view filename,
                                  const NonSizePrefixedFlatbuffer<T> &msg) {
  std::ofstream file(std::string(filename),
                     std::ios::out | std::ofstream::binary);
  CHECK(file) << ": Couldn't open " << filename;
  std::copy(msg.span().begin(), msg.span().end(),
            std::ostreambuf_iterator<char>(file));
}

// Parses a file as JSON and returns the corresponding Flatbuffer, or dies.
template <typename T>
inline FlatbufferDetachedBuffer<T> JsonFileToFlatbuffer(
    const std::string_view path) {
  std::ifstream t{std::string(path)};
  std::istream_iterator<char> start(t), end;
  std::string result(start, end);
  return FlatbufferDetachedBuffer<T>(JsonToFlatbuffer<T>(result));
}

// Parses a file as a binary flatbuffer or dies.
template <typename T>
inline FlatbufferVector<T> FileToFlatbuffer(const std::string_view path) {
  std::ifstream instream(std::string(path), std::ios::in | std::ios::binary);
  ResizeableBuffer data;
  std::istreambuf_iterator<char> it(instream);
  while (it != std::istreambuf_iterator<char>()) {
    data.push_back(*it);
    ++it;
  }
  return FlatbufferVector<T>(std::move(data));
}

}  // namespace aos

#endif  // AOS_JSON_TO_FLATBUFFER_H_

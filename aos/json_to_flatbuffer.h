#ifndef AOS_JSON_TO_FLATBUFFER_H_
#define AOS_JSON_TO_FLATBUFFER_H_

#include <cstddef>
#include <fstream>
#include <string>
#include <string_view>

#include "aos/flatbuffers.h"
#include "flatbuffers/flatbuffers.h"
#include "flatbuffers/reflection.h"

namespace aos {

// Parses the flatbuffer into the buffer, or returns an empty buffer.
flatbuffers::DetachedBuffer JsonToFlatbuffer(
    const std::string_view data, const flatbuffers::TypeTable *typetable);

// Parses the flatbuffer into the builder, and returns the offset.
flatbuffers::Offset<flatbuffers::Table> JsonToFlatbuffer(
    const std::string_view data, const flatbuffers::TypeTable *typetable,
    flatbuffers::FlatBufferBuilder *fbb);

// Typed versions of the above methods.
template <typename T>
inline flatbuffers::DetachedBuffer JsonToFlatbuffer(
    const std::string_view data) {
  return JsonToFlatbuffer(data, T::MiniReflectTypeTable());
}
template <typename T>
inline flatbuffers::Offset<T> JsonToFlatbuffer(
    const std::string_view data, flatbuffers::FlatBufferBuilder *fbb) {
  return flatbuffers::Offset<T>(
      JsonToFlatbuffer(data, T::MiniReflectTypeTable(), fbb).o);
}

// Converts a flatbuffer into a Json string.
// multi_line controls if the Json is written out on multiple lines or one.
// The methods below are generally more useful than BufferFlatbufferToJson and
// TableFlatbufferToJson.
::std::string BufferFlatbufferToJson(const uint8_t *buffer,
                                     const flatbuffers::TypeTable *typetable,
                                     bool multi_line = false,
                                     size_t max_vector_size = SIZE_MAX);

::std::string TableFlatbufferToJson(const flatbuffers::Table *t,
                                    const ::flatbuffers::TypeTable *typetable,
                                    bool multi_line,
                                    size_t max_vector_size = SIZE_MAX);

// Converts a DetachedBuffer holding a flatbuffer to JSON.
inline ::std::string FlatbufferToJson(const flatbuffers::DetachedBuffer &buffer,
                                      const flatbuffers::TypeTable *typetable,
                                      bool multi_line = false,
                                      size_t max_vector_size = SIZE_MAX) {
  return BufferFlatbufferToJson(buffer.data(), typetable, multi_line,
                                max_vector_size);
}

// Converts a Flatbuffer<T> holding a flatbuffer to JSON.
template <typename T>
inline ::std::string FlatbufferToJson(const Flatbuffer<T> &flatbuffer,
                                      bool multi_line = false,
                                      size_t max_vector_size = SIZE_MAX) {
  return BufferFlatbufferToJson(flatbuffer.data(),
                                Flatbuffer<T>::MiniReflectTypeTable(),
                                multi_line, max_vector_size);
}

// Converts a flatbuffer::Table to JSON.
template <typename T>
typename std::enable_if<std::is_base_of<flatbuffers::Table, T>::value,
                        std::string>::
    type inline FlatbufferToJson(const T *flatbuffer, bool multi_line = false,
                                 size_t max_vector_size = SIZE_MAX) {
  return TableFlatbufferToJson(
      reinterpret_cast<const flatbuffers::Table *>(flatbuffer),
      Flatbuffer<T>::MiniReflectTypeTable(), multi_line, max_vector_size);
}

std::string FlatbufferToJson(const reflection::Schema *const schema,
                             const uint8_t *const data, bool multi_line = false,
                             size_t max_vector_size = SIZE_MAX);

// Writes a Flatbuffer to a file, or dies.
template <typename T>
inline void WriteFlatbufferToJson(const std::string_view filename,
                                  const Flatbuffer<T> &msg) {
  std::ofstream json_file(std::string(filename), std::ios::out);
  CHECK(json_file) << ": Couldn't open " << filename;
  json_file << FlatbufferToJson(msg);
  json_file.close();
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

}  // namespace aos

#endif  // AOS_JSON_TO_FLATBUFFER_H_

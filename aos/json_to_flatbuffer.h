#ifndef AOS_JSON_TO_FLATBUFFER_H_
#define AOS_JSON_TO_FLATBUFFER_H_

#include <cstddef>
#include <string>
#include <string_view>

#include "aos/flatbuffers.h"
#include "flatbuffers/flatbuffers.h"
#include "flatbuffers/reflection.h"

namespace aos {

// Parses the flatbuffer into the vector, or returns an empty vector.
flatbuffers::DetachedBuffer JsonToFlatbuffer(
    const std::string_view data, const flatbuffers::TypeTable *typetable);

// Converts a flatbuffer into a Json string.
// multi_line controls if the Json is written out on multiple lines or one.
// The methods below are generally more useful than BufferFlatbufferToJson and
// TableFlatbufferToJson.
::std::string BufferFlatbufferToJson(const uint8_t *buffer,
                                     const flatbuffers::TypeTable *typetable,
                                     bool multi_line = false);

::std::string TableFlatbufferToJson(const flatbuffers::Table *t,
                                    const ::flatbuffers::TypeTable *typetable,
                                    bool multi_line);

// Converts a DetachedBuffer holding a flatbuffer to JSON.
inline ::std::string FlatbufferToJson(const flatbuffers::DetachedBuffer &buffer,
                                      const flatbuffers::TypeTable *typetable,
                                      bool multi_line = false) {
  return BufferFlatbufferToJson(buffer.data(), typetable, multi_line);
}

// Converts a Flatbuffer<T> holding a flatbuffer to JSON.
template <typename T>
inline ::std::string FlatbufferToJson(const Flatbuffer<T> &flatbuffer,
                                      bool multi_line = false) {
  return BufferFlatbufferToJson(
      flatbuffer.data(), Flatbuffer<T>::MiniReflectTypeTable(), multi_line);
}

// Converts a flatbuffer::Table to JSON.
template <typename T>
typename std::enable_if<
    std::is_base_of<flatbuffers::Table, T>::value,
    std::string>::type inline FlatbufferToJson(const T *flatbuffer,
                                               bool multi_line = false) {
  return TableFlatbufferToJson(
      reinterpret_cast<const flatbuffers::Table *>(flatbuffer),
      Flatbuffer<T>::MiniReflectTypeTable(), multi_line);
}

std::string FlatbufferToJson(const reflection::Schema *const schema,
                             const uint8_t *const data);

}  // namespace aos

#endif  // AOS_JSON_TO_FLATBUFFER_H_

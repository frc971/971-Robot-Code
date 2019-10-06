#ifndef AOS_JSON_TO_FLATBUFFER_H_
#define AOS_JSON_TO_FLATBUFFER_H_

#include <cstddef>
#include <string>

#include "absl/strings/string_view.h"
#include "flatbuffers/flatbuffers.h"

namespace aos {

// Parses the flatbuffer into the vector, or returns an empty vector.
::std::vector<uint8_t> JsonToFlatbuffer(
    const absl::string_view data, const flatbuffers::TypeTable *typetable);

// Converts a flatbuffer into a Json string.
//
// multi_line controls if the Json is written out on multiple lines or one.
::std::string FlatbufferToJson(const uint8_t *buffer,
                               const flatbuffers::TypeTable *typetable,
                               bool multi_line = false);

}  // namespace aos

#endif  // AOS_JSON_TO_FLATBUFFER_H_

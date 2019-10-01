#ifndef AOS_FLATBUFFER_UTILS_
#define AOS_FLATBUFFER_UTILS_

#include "flatbuffers/flatbuffers.h"

namespace aos {

// Returns a human readable description of the type.
inline const char *ElementaryTypeName(
    const flatbuffers::ElementaryType elementary_type) {
  return flatbuffers::ElementaryTypeNames()[elementary_type] + 3;
}

}  // namespace aos

#endif  // AOS_FLATBUFFER_UTILS_

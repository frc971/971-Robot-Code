#ifndef AOS_FLATBUFFER_MERGE_H_
#define AOS_FLATBUFFER_MERGE_H_

#include <cstddef>
#include <string>

#include "flatbuffers/flatbuffers.h"

namespace aos {

::std::vector<uint8_t> MergeFlatBuffers(const flatbuffers::TypeTable *typetable,
                                        const uint8_t *data1,
                                        const uint8_t *data2);

template <class T>
::std::vector<uint8_t> MergeFlatBuffers(const uint8_t *data1,
                                        const uint8_t *data2) {
  return MergeFlatBuffers(T::MiniReflectTypeTable(), data1, data2);
}

}  // namespace aos

#endif  // AOS_FLATBUFFER_MERGE_H_

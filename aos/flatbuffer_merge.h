#ifndef AOS_FLATBUFFER_MERGE_H_
#define AOS_FLATBUFFER_MERGE_H_

#include <cstddef>
#include <string>

#include "aos/flatbuffers.h"
#include "flatbuffers/flatbuffers.h"

namespace aos {

flatbuffers::DetachedBuffer MergeFlatBuffers(
    const flatbuffers::TypeTable *typetable, const uint8_t *data1,
    const uint8_t *data2);

// Merges 2 flat buffers with the provided type table into the builder.  Returns
// the offset to the flatbuffers.
// One or both of t1 and t2 must be non-null.  If one is null, this method
// coppies instead of merging.
flatbuffers::Offset<flatbuffers::Table> MergeFlatBuffers(
    const flatbuffers::TypeTable *typetable, const flatbuffers::Table *t1,
    const flatbuffers::Table *t2, flatbuffers::FlatBufferBuilder *fbb);

template <class T>
inline flatbuffers::Offset<T> MergeFlatBuffers(
    const flatbuffers::Table *t1,
    const flatbuffers::Table *t2, flatbuffers::FlatBufferBuilder *fbb) {
  return MergeFlatBuffers(T::MiniReflectTypeTable(), t1, t2, fbb).o;
}

template <class T>
inline flatbuffers::DetachedBuffer MergeFlatBuffers(const uint8_t *data1,
                                                    const uint8_t *data2) {
  return MergeFlatBuffers(T::MiniReflectTypeTable(), data1, data2);
}

template <class T>
inline flatbuffers::DetachedBuffer MergeFlatBuffers(
    const flatbuffers::DetachedBuffer &data1,
    const flatbuffers::DetachedBuffer &data2) {
  return MergeFlatBuffers(T::MiniReflectTypeTable(), data1.data(),
                          data2.data());
}

template <class T>
inline aos::FlatbufferDetachedBuffer<T> MergeFlatBuffers(
    const aos::Flatbuffer<T> &fb1, const aos::Flatbuffer<T> &fb2) {
const uint8_t *data1 = fb1.data();
const uint8_t *data2 = fb2.data();
  return aos::FlatbufferDetachedBuffer<T>(
      MergeFlatBuffers(T::MiniReflectTypeTable(), data1, data2));
}

template <class T>
inline aos::FlatbufferDetachedBuffer<T> MergeFlatBuffers(const T *fb1,
                                                         const T *fb2) {
  return aos::FlatbufferDetachedBuffer<T>(MergeFlatBuffers(
      T::MiniReflectTypeTable(), reinterpret_cast<const uint8_t *>(fb1),
      reinterpret_cast<const uint8_t *>(fb2)));
}

template <class T>
inline flatbuffers::Offset<T> CopyFlatBuffer(
    const T *t1, flatbuffers::FlatBufferBuilder *fbb) {
  return MergeFlatBuffers<T>(reinterpret_cast<const flatbuffers::Table *>(t1),
                             nullptr, fbb);
}

template <class T>
inline FlatbufferDetachedBuffer<T> CopyFlatBuffer(const T *t) {
  flatbuffers::FlatBufferBuilder fbb;
  fbb.ForceDefaults(1);
  fbb.Finish(CopyFlatBuffer<T>(t, &fbb));
  return FlatbufferDetachedBuffer<T>(fbb.Release());
}

}  // namespace aos

#endif  // AOS_FLATBUFFER_MERGE_H_

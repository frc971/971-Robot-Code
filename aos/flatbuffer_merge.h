#ifndef AOS_FLATBUFFER_MERGE_H_
#define AOS_FLATBUFFER_MERGE_H_

#include <cstddef>
#include <string>

#include "flatbuffers/flatbuffers.h"

#include "aos/flatbuffers.h"

namespace aos {

// Merges 2 flat buffers with the provided type table into the builder.  Returns
// the offset to the flatbuffers.
// One or both of t1 and t2 must be non-null.  If one is null, this method
// copies instead of merging.
flatbuffers::Offset<flatbuffers::Table> MergeFlatBuffers(
    const flatbuffers::TypeTable *typetable, const flatbuffers::Table *t1,
    const flatbuffers::Table *t2, flatbuffers::FlatBufferBuilder *fbb);

template <class T>
inline flatbuffers::Offset<T> MergeFlatBuffers(
    const flatbuffers::Table *t1, const flatbuffers::Table *t2,
    flatbuffers::FlatBufferBuilder *fbb) {
  return MergeFlatBuffers(T::MiniReflectTypeTable(), t1, t2, fbb).o;
}

template <class T>
inline aos::FlatbufferDetachedBuffer<T> MergeFlatBuffers(const T *fb1,
                                                         const T *fb2) {
  flatbuffers::FlatBufferBuilder fbb;
  fbb.ForceDefaults(true);
  fbb.DedupVtables(false);
  fbb.Finish(MergeFlatBuffers<T>(
      reinterpret_cast<const flatbuffers::Table *>(fb1),
      reinterpret_cast<const flatbuffers::Table *>(fb2), &fbb));
  return aos::FlatbufferDetachedBuffer<T>(fbb.Release());
}

template <class T>
inline flatbuffers::Offset<T> MergeFlatBuffers(
    const T *fb1, const T *fb2, flatbuffers::FlatBufferBuilder *fbb) {
  return MergeFlatBuffers<T>(reinterpret_cast<const flatbuffers::Table *>(fb1),
                             reinterpret_cast<const flatbuffers::Table *>(fb2),
                             fbb);
}

template <class T>
inline aos::FlatbufferDetachedBuffer<T> MergeFlatBuffers(
    const aos::Flatbuffer<T> &fb1, const aos::Flatbuffer<T> &fb2) {
  return aos::FlatbufferDetachedBuffer<T>(
      MergeFlatBuffers<T>(&fb1.message(), &fb2.message()));
}

template <class T>
inline flatbuffers::Offset<T> MergeFlatBuffers(
    const aos::Flatbuffer<T> &fb1, const aos::Flatbuffer<T> &fb2,
    flatbuffers::FlatBufferBuilder *fbb) {
  return MergeFlatBuffers<T>(
      reinterpret_cast<const flatbuffers::Table *>(&fb1.message()),
      reinterpret_cast<const flatbuffers::Table *>(&fb2.message()), fbb);
}

// Copies a flatbuffer by walking the tree and copying all the pieces.  This
// converts DAGs to trees.
template <class T>
inline flatbuffers::Offset<T> RecursiveCopyFlatBuffer(
    const T *t1, flatbuffers::FlatBufferBuilder *fbb) {
  return MergeFlatBuffers<T>(reinterpret_cast<const flatbuffers::Table *>(t1),
                             nullptr, fbb);
}

// Copies a flatbuffer by finding the extents of the memory using the typetable
// and copying the containing memory.  This doesn't allocate memory, and
// preserves DAGs.
flatbuffers::Offset<flatbuffers::Table> CopyFlatBuffer(
    const flatbuffers::Table *t1, const flatbuffers::TypeTable *typetable,
    flatbuffers::FlatBufferBuilder *fbb);

template <class T>
inline flatbuffers::Offset<T> CopyFlatBuffer(
    const T *t1, flatbuffers::FlatBufferBuilder *fbb) {
  return flatbuffers::Offset<T>(
      CopyFlatBuffer(reinterpret_cast<const flatbuffers::Table *>(t1),
                     T::MiniReflectTypeTable(), fbb)
          .o);
}

template <class T>
inline flatbuffers::Offset<T> CopyFlatBuffer(
    const Flatbuffer<T> &t1, flatbuffers::FlatBufferBuilder *fbb) {
  return flatbuffers::Offset<T>(
      CopyFlatBuffer(
          reinterpret_cast<const flatbuffers::Table *>(&t1.message()),
          T::MiniReflectTypeTable(), fbb)
          .o);
}

namespace flatbuffer_merge_internal {

inline flatbuffers::uoffset_t DoBlindCopyFlatBuffer(
    const void *message, absl::Span<const uint8_t> span,
    flatbuffers::FlatBufferBuilder *fbb) {
  // Enforce 8 byte alignment so anything inside the flatbuffer can be read.
  fbb->Align(sizeof(flatbuffers::largest_scalar_t));

  // We don't know how much of the start of the flatbuffer is padding.  The
  // safest thing to do from an alignment point of view (without looking inside)
  // is to copy the initial offset and leave it as dead space.
  fbb->PushBytes(span.data(), span.size());
  // Then, compute the offset from the back by computing the distance from the
  // front to the start of the message.
  return fbb->GetSize() -
         static_cast<flatbuffers::uoffset_t>(
             reinterpret_cast<const uint8_t *>(message) - span.data());
}

}  // namespace flatbuffer_merge_internal

// Copies a flatbuffer by copying all the data without looking inside and
// pointing inside it.
template <class T>
inline flatbuffers::Offset<T> BlindCopyFlatBuffer(
    const NonSizePrefixedFlatbuffer<T> &t,
    flatbuffers::FlatBufferBuilder *fbb) {
  return flatbuffer_merge_internal::DoBlindCopyFlatBuffer(&t.message(),
                                                          t.span(), fbb);
}

// Copies a flatbuffer by copying all the data without looking inside and
// pointing inside it.
template <class T>
inline flatbuffers::Offset<T> BlindCopyFlatBuffer(
    const SizePrefixedFlatbuffer<T> &t, flatbuffers::FlatBufferBuilder *fbb) {
  return flatbuffer_merge_internal::DoBlindCopyFlatBuffer(&t.message(),
                                                          t.span(), fbb);
}

template <class T>
inline flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<T>>>
RecursiveCopyVectorTable(const flatbuffers::Vector<flatbuffers::Offset<T>> *t1,
                         flatbuffers::FlatBufferBuilder *fbb) {
  if (t1 == nullptr) {
    return 0;
  }
  std::vector<flatbuffers::Offset<T>> v;
  for (const T *t : *t1) {
    v.emplace_back(RecursiveCopyFlatBuffer(t, fbb));
  }
  return fbb->CreateVector(v);
}

inline flatbuffers::Offset<
    flatbuffers::Vector<flatbuffers::Offset<flatbuffers::String>>>
CopyVectorSharedString(
    const flatbuffers::Vector<flatbuffers::Offset<flatbuffers::String>> *t1,
    flatbuffers::FlatBufferBuilder *fbb) {
  if (t1 == nullptr) {
    return 0;
  }
  std::vector<flatbuffers::Offset<flatbuffers::String>> v;
  for (const flatbuffers::String *t : *t1) {
    v.emplace_back(fbb->CreateSharedString(t));
  }
  return fbb->CreateVector(v);
}

template <class T>
inline FlatbufferDetachedBuffer<T> CopyFlatBuffer(const T *t) {
  flatbuffers::FlatBufferBuilder fbb;
  fbb.ForceDefaults(true);
  fbb.DedupVtables(false);
  fbb.Finish(CopyFlatBuffer<T>(t, &fbb));
  return FlatbufferDetachedBuffer<T>(fbb.Release());
}

template <class T>
inline FlatbufferDetachedBuffer<T> RecursiveCopyFlatBuffer(const T *t) {
  flatbuffers::FlatBufferBuilder fbb;
  fbb.ForceDefaults(true);
  fbb.DedupVtables(false);
  fbb.Finish(RecursiveCopyFlatBuffer<T>(t, &fbb));
  return FlatbufferDetachedBuffer<T>(fbb.Release());
}

// Compares 2 flatbuffers.  Returns true if they match, false otherwise.
bool CompareFlatBuffer(const flatbuffers::TypeTable *typetable,
                       const flatbuffers::Table *t1,
                       const flatbuffers::Table *t2);

template <class T>
inline bool CompareFlatBuffer(const T *t1, const T *t2) {
  return CompareFlatBuffer(T::MiniReflectTypeTable(),
                           reinterpret_cast<const flatbuffers::Table *>(t1),
                           reinterpret_cast<const flatbuffers::Table *>(t2));
}

template <class T>
inline bool CompareFlatBuffer(const aos::NonSizePrefixedFlatbuffer<T> &t1,
                              const aos::NonSizePrefixedFlatbuffer<T> &t2) {
  return t1.span() == t2.span();
}

template <class T>
inline bool CompareFlatBuffer(const aos::SizePrefixedFlatbuffer<T> &t1,
                              const aos::SizePrefixedFlatbuffer<T> &t2) {
  return t1.span() == t2.span();
}

}  // namespace aos

#endif  // AOS_FLATBUFFER_MERGE_H_

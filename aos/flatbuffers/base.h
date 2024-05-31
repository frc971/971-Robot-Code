#ifndef AOS_FLATBUFFERS_BASE_H_
#define AOS_FLATBUFFERS_BASE_H_

#include <stdint.h>
#include <sys/types.h>

#include <cstring>
#include <memory>
#include <optional>
#include <ostream>
#include <span>
#include <utility>
#include <vector>

#include "absl/types/span.h"
#include "flatbuffers/base.h"
#include "glog/logging.h"

#include "aos/containers/resizeable_buffer.h"
#include "aos/ipc_lib/data_alignment.h"
#include "aos/shared_span.h"

namespace aos::fbs {

using ::flatbuffers::soffset_t;
using ::flatbuffers::uoffset_t;
using ::flatbuffers::voffset_t;

// Returns the offset into the buffer needed to provide 'alignment' alignment
// 'aligned_offset' bytes after the returned offset.  This assumes that the
// first 'starting_offset' bytes are spoken for.
constexpr size_t AlignOffset(size_t starting_offset, size_t alignment,
                             size_t aligned_offset = 0) {
  // We can be clever with bitwise operations by assuming that aligment is a
  // power of two. Or we can just be clearer about what we mean and eat a few
  // integer divides.
  return (((starting_offset + aligned_offset - 1) / alignment) + 1) *
             alignment -
         aligned_offset;
}

// Used as a parameter to methods where we are messing with memory and may or
// may not want to clear it to zeroes.
enum class SetZero { kYes, kNo };

class Allocator;

// Parent type of any object that may need to dynamically change size at
// runtime. Used by the static table and vector types to request additional
// blocks of memory when needed.
//
// The way that this works is that every ResizeableObject has some number of
// children that are themselves ResizeableObject's and whose memory is entirely
// contained within their parent's memory. A ResizeableObject without a parent
// instead has an Allocator that it can use to allocate additional blocks
// of memory. Whenever a child needs to grow in size, it will make a call to
// InsertBytes() on its parent, which will percolate up until InsertBytes() gets
// called on the root allocator. If the insert succeeds, then every single child
// through the entire tree will get notified (this is because the allocator may
// have shifted the entire memory buffer, so any pointers may need to be
// updated). Child types will provide implementations of the GetObjects() method
// to both allow tree traversal as well as to allow the various internal offsets
// to be updated appropriately.
class ResizeableObject {
 public:
  // Returns the underlying memory buffer into which the flatbuffer will be
  // serialized.
  std::span<uint8_t> buffer() { return buffer_; }
  std::span<const uint8_t> buffer() const { return buffer_; }

  // Updates the underlying memory buffer to new_buffer, with an indication of
  // where bytes were inserted/removed from the buffer. It is assumed that
  // new_buffer already has the state of the serialized flatbuffer
  // copied into it.
  // * When bytes have been inserted, modification_point will point to the first
  //   of the inserted bytes in new_buffer and bytes_inserted will be the number
  //   of new bytes.
  // * Buffer shrinkage is not currently supported.
  // * When bytes_inserted is zero, modification_point is ignored.
  void UpdateBuffer(std::span<uint8_t> new_buffer, void *modification_point,
                    ssize_t bytes_inserted);

 protected:
  // Data associated with a sub-object of this object.
  struct SubObject {
    // A direct pointer to the inline entry in the flatbuffer table data. The
    // pointer must be valid, but the entry itself may be zero if the object is
    // not actually populated.
    // If *inline_entry is non-zero, this will get updated if any new memory got
    // added/removed in-between inline_entry and the actual data pointed to be
    // inline_entry.
    uoffset_t *inline_entry;
    // The actual child object. Should be nullptr if *inline_entry is zero; must
    // be valid if *inline_entry is non-zero.
    ResizeableObject *object;
    // The nominal offset from buffer_.data() to object->buffer_.data().
    // Must be provided, and must always be valid, even if *inline_entry is
    // zero.
    // I.e., the following holds when object is not nullptr:
    // SubObject object = parent.GetSubObject(index);
    // CHECK_EQ(parent.buffer()->data() + *object.absolute_offset,
    // object.object->buffer().data());
    size_t *absolute_offset;
  };

  ResizeableObject(std::span<uint8_t> buffer, ResizeableObject *parent)
      : buffer_(buffer), parent_(parent) {}
  ResizeableObject(std::span<uint8_t> buffer, Allocator *allocator)
      : buffer_(buffer), allocator_(allocator) {}
  ResizeableObject(std::span<uint8_t> buffer,
                   std::unique_ptr<Allocator> allocator)
      : buffer_(buffer),
        owned_allocator_(std::move(allocator)),
        allocator_(owned_allocator_.get()) {}
  ResizeableObject(const ResizeableObject &) = delete;
  ResizeableObject &operator=(const ResizeableObject &) = delete;
  // Users do not end up using the move constructor; however, it is needed to
  // handle the fact that a ResizeableObject may be a member of an std::vector
  // in the various generated types.
  ResizeableObject(ResizeableObject &&other);
  // Required alignment of this object.
  virtual size_t Alignment() const = 0;
  // Causes bytes bytes to be inserted between insertion_point - 1 and
  // insertion_point.
  // If requested, the new bytes will be cleared to zero; otherwise they will be
  // left uninitialized.
  // The insertion_point may not be equal to this->buffer_.data(); it may be a
  // pointer just past the end of the buffer. This is to ease the
  // implementation, and is merely a requirement that any buffer growth occur
  // only on the inside or past the end of the vector, and not prior to the
  // start of the vector.
  // Returns a span of the inserted bytes on success, nullopt on failure (e.g.,
  // if the allocator has no memory available).
  std::optional<std::span<uint8_t>> InsertBytes(void *insertion_point,
                                                size_t bytes, SetZero set_zero);
  // Called *after* the internal buffer_ has been swapped out and *after* the
  // object tree has been traversed and fixed.
  virtual void ObserveBufferModification() {}

  // Returns the index'th sub object of this object.
  // index must be less than NumberOfSubObjects().
  // This will include objects which are not currently populated but which may
  // be populated in the future (so that we can track what the necessary offsets
  // are when we do populate it).
  virtual SubObject GetSubObject(size_t index) = 0;
  // Number of sub-objects of this object. May be zero.
  virtual size_t NumberOfSubObjects() const = 0;

  // Treating the supplied absolute_offset as an offset into the internal memory
  // buffer, return the pointer to the underlying memory.
  const void *PointerForAbsoluteOffset(const size_t absolute_offset) {
    return buffer_.data() + absolute_offset;
  }
  // Returns a span at the requested offset into the buffer for the requested
  // size.
  std::span<uint8_t> BufferForObject(size_t absolute_offset, size_t size);
  // When memory has been inserted/removed, this iterates over the sub-objects
  // and notifies/adjusts them appropriately.
  // This will be called after buffer_ has been updated, and:
  // * For insertion, modification_point will point into the new buffer_ to the
  //   first of the newly inserted bytes.
  // * Removal is not entirely implemented yet, but for removal,
  //   modification_point should point to the first byte after the removed
  //   chunk.
  void FixObjects(void *modification_point, ssize_t bytes_inserted);

  Allocator *allocator() { return allocator_; }

  std::span<uint8_t> buffer_;

 private:
  ResizeableObject *parent_ = nullptr;
  std::unique_ptr<Allocator> owned_allocator_;
  Allocator *allocator_ = nullptr;
};

// Interface to represent a memory allocator for use with ResizeableObject.
class Allocator {
 public:
  virtual ~Allocator() {}
  // Allocates memory of the requested size and alignment. alignment is
  // guaranteed.
  //
  // On failure to allocate the requested size, returns nullopt;
  // Never returns a partial span.
  // The span will be initialized to zero upon request.
  // Once Allocate() has been called once, it may not be called again until
  // Deallocate() has been called. In order to adjust the size of the buffer,
  // call InsertBytes() and RemoveBytes().
  [[nodiscard]] virtual std::optional<std::span<uint8_t>> Allocate(
      size_t size, size_t alignment, SetZero set_zero) = 0;
  // Identical to Allocate(), but dies on failure.
  [[nodiscard]] std::span<uint8_t> AllocateOrDie(size_t size, size_t alignment,
                                                 SetZero set_zero) {
    std::optional<std::span<uint8_t>> span =
        Allocate(size, alignment, set_zero);
    CHECK(span.has_value()) << ": Failed to allocate " << size << " bytes.";
    CHECK_EQ(size, span.value().size())
        << ": Failed to allocate " << size << " bytes.";
    CHECK_EQ(reinterpret_cast<size_t>(span.value().data()) % alignment, 0u)
        << "Failed to allocate data of length " << size << " with alignment "
        << alignment;

    return span.value();
  }
  // Increases the size of the buffer by inserting bytes bytes immediately
  // before insertion_point.
  // alignment_hint specifies the alignment of the entire buffer, not of the
  // inserted bytes.
  // The returned span may or may not overlap with the original buffer in
  // memory.
  // The inserted bytes will be set to zero or uninitialized depending on the
  // value of SetZero.
  // insertion_point must be in (or 1 past the end of) the buffer.
  // Returns nullopt on a failure to allocate the requested bytes.
  [[nodiscard]] virtual std::optional<std::span<uint8_t>> InsertBytes(
      void *insertion_point, size_t bytes, size_t alignment_hint,
      SetZero set_zero) = 0;
  // Removes the requested span of bytes from the buffer, returning the new
  // buffer.
  [[nodiscard]] virtual std::span<uint8_t> RemoveBytes(
      std::span<uint8_t> remove_bytes) = 0;
  // Deallocates the currently allocated buffer. The provided buffer must match
  // the latest version of the buffer.
  // If Allocate() has been called, Deallocate() must be called prior to
  // destroying the Allocator.
  virtual void Deallocate(std::span<uint8_t> buffer) = 0;
};

// Allocator that allocates all of its memory within a provided span. To match
// the behavior of the FlatBufferBuilder, it will start its allocations at the
// end of the provided span.
//
// Attempts to allocate more memory than is present in the provided buffer will
// fail.
class SpanAllocator : public Allocator {
 public:
  SpanAllocator(std::span<uint8_t> buffer) : buffer_(buffer) {}
  ~SpanAllocator() {
    CHECK(!allocated_)
        << ": Must deallocate before destroying the SpanAllocator.";
  }

  std::optional<std::span<uint8_t>> Allocate(size_t size, size_t /*alignment*/,
                                             SetZero set_zero) override;

  std::optional<std::span<uint8_t>> InsertBytes(void *insertion_point,
                                                size_t bytes,
                                                size_t /*alignment*/,
                                                SetZero set_zero) override;

  std::span<uint8_t> RemoveBytes(std::span<uint8_t> remove_bytes) override;

  void Deallocate(std::span<uint8_t>) override;

 private:
  std::span<uint8_t> buffer_;
  bool allocated_ = false;
  size_t allocated_size_ = 0;
};

// Allocator that uses an AllocatorResizeableBuffer to allow arbitrary-sized
// allocations.  Aligns the end of the buffer to an alignment of
// kChannelDataAlignment.
class AlignedVectorAllocator : public fbs::Allocator {
 public:
  static constexpr size_t kAlignment = aos::kChannelDataAlignment;
  AlignedVectorAllocator() {}
  ~AlignedVectorAllocator();

  std::optional<std::span<uint8_t>> Allocate(size_t size, size_t alignment,
                                             fbs::SetZero set_zero) override;

  std::optional<std::span<uint8_t>> InsertBytes(void *insertion_point,
                                                size_t bytes, size_t alignment,
                                                fbs::SetZero set_zero) override;

  std::span<uint8_t> RemoveBytes(std::span<uint8_t> remove_bytes) override;

  void Deallocate(std::span<uint8_t>) override;

  aos::SharedSpan Release();

 private:
  struct SharedSpanHolder {
    aos::AllocatorResizeableBuffer<aos::AlignedReallocator<kAlignment>> buffer;
    absl::Span<const uint8_t> span;
  };
  uint8_t *data() { return buffer_.data() + buffer_.size() - allocated_size_; }

  aos::AllocatorResizeableBuffer<aos::AlignedReallocator<kAlignment>> buffer_;

  size_t allocated_size_ = 0u;
  bool released_ = false;
};

// Allocates and owns a fixed-size memory buffer on the stack.
//
// This provides a convenient Allocator for use with the aos::fbs::Builder
// in realtime code instead of trying to use the VectorAllocator.
template <std::size_t N, std::size_t alignment = 64>
class FixedStackAllocator : public SpanAllocator {
 public:
  FixedStackAllocator() : SpanAllocator({buffer_, sizeof(buffer_)}) {}

 private:
  alignas(alignment) uint8_t buffer_[N];
};

namespace internal {
std::ostream &DebugBytes(std::span<const uint8_t> span, std::ostream &os);
inline void ClearSpan(std::span<uint8_t> span) {
  memset(span.data(), 0, span.size());
}
// std::span::subspan does not do bounds checking.
template <typename T>
inline std::span<T> GetSubSpan(std::span<T> span, size_t offset,
                               size_t count = std::dynamic_extent) {
  if (count != std::dynamic_extent) {
    CHECK_LE(offset + count, span.size());
  }
  return span.subspan(offset, count);
}
// Normal users should never move any of the special flatbuffer types that we
// provide. However, they do need to be moveable in order to support the use of
// resizeable vectors. As such, we make all the move constructors private and
// friend the TableMover struct. The TableMover struct is then used in places
// that need to have moveable objects. It should never be used by a user.
template <typename T>
struct TableMover {
  TableMover(std::span<uint8_t> buffer, ResizeableObject *parent)
      : t(buffer, parent) {}
  TableMover(std::span<uint8_t> buffer, Allocator *allocator)
      : t(buffer, allocator) {}
  TableMover(std::span<uint8_t> buffer, ::std::unique_ptr<Allocator> allocator)
      : t(buffer, ::std::move(allocator)) {}
  TableMover(TableMover &&) = default;
  T t;
};
}  // namespace internal
}  // namespace aos::fbs

#endif  // AOS_FLATBUFFERS_BASE_H_

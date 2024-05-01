#ifndef AOS_FLATBUFFERS_ALIGNED_ALLOCATOR_H_
#define AOS_FLATBUFFERS_ALIGNED_ALLOCATOR_H_

#include <memory>
#include <optional>
#include <span>

#include "glog/logging.h"

#include "aos/containers/resizeable_buffer.h"
#include "aos/events/event_loop.h"
#include "aos/flatbuffers/base.h"
#include "aos/ipc_lib/data_alignment.h"

namespace aos::fbs {

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
    aos::AllocatorResizeableBuffer<
        aos::AlignedReallocator<kChannelDataAlignment>>
        buffer;
    absl::Span<const uint8_t> span;
  };
  uint8_t *data() { return buffer_.data() + buffer_.size() - allocated_size_; }

  aos::AllocatorResizeableBuffer<aos::AlignedReallocator<kChannelDataAlignment>>
      buffer_;

  size_t allocated_size_ = 0u;
  bool released_ = false;
};

}  // namespace aos::fbs

#endif  // AOS_FLATBUFFERS_ALIGNED_ALLOCATOR_H_

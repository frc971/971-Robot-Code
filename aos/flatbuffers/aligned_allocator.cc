#include "aos/flatbuffers/aligned_allocator.h"

namespace aos::fbs {

AlignedVectorAllocator::~AlignedVectorAllocator() {
  CHECK(buffer_.empty())
      << ": Must deallocate before destroying the AlignedVectorAllocator.";
}

std::optional<std::span<uint8_t>> AlignedVectorAllocator::Allocate(
    size_t size, size_t /*alignment*/, fbs::SetZero set_zero) {
  CHECK(buffer_.empty()) << ": Must deallocate before calling Allocate().";
  buffer_.resize(((size + kAlignment - 1) / kAlignment) * kAlignment);
  allocated_size_ = size;
  if (set_zero == fbs::SetZero::kYes) {
    memset(buffer_.data(), 0, buffer_.size());
  }

  return std::span<uint8_t>{data(), allocated_size_};
}

std::optional<std::span<uint8_t>> AlignedVectorAllocator::InsertBytes(
    void *insertion_point, size_t bytes, size_t /*alignment*/,
    fbs::SetZero set_zero) {
  DCHECK_GE(reinterpret_cast<const uint8_t *>(insertion_point), data());
  DCHECK_LE(reinterpret_cast<const uint8_t *>(insertion_point),
            data() + allocated_size_);
  const size_t buffer_offset =
      reinterpret_cast<const uint8_t *>(insertion_point) - data();
  // TODO(austin): This has an extra memcpy in it that isn't strictly needed
  // when we resize.  Remove it if performance is a concern.
  const size_t absolute_buffer_offset =
      reinterpret_cast<const uint8_t *>(insertion_point) - buffer_.data();
  const size_t previous_size = buffer_.size();

  buffer_.resize(((allocated_size_ + bytes + kAlignment - 1) / kAlignment) *
                 kAlignment);

  // Now, we've got space both before and after the block of data.  Move the
  // data after to the end, and the data before to the start.

  const size_t new_space_after = buffer_.size() - previous_size;

  // Move the rest of the data to be end aligned.  If the buffer wasn't resized,
  // this will be a nop.
  memmove(buffer_.data() + absolute_buffer_offset + new_space_after,
          buffer_.data() + absolute_buffer_offset,
          previous_size - absolute_buffer_offset);

  // Now, move the data at the front to be aligned too.
  memmove(buffer_.data() + buffer_.size() - (allocated_size_ + bytes),
          buffer_.data() + previous_size - allocated_size_,
          allocated_size_ - (previous_size - absolute_buffer_offset));

  if (set_zero == fbs::SetZero::kYes) {
    memset(data() - bytes + buffer_offset, 0, bytes);
  }
  allocated_size_ += bytes;

  return std::span<uint8_t>{data(), allocated_size_};
}

std::span<uint8_t> AlignedVectorAllocator::RemoveBytes(
    std::span<uint8_t> remove_bytes) {
  const ssize_t removal_index = remove_bytes.data() - buffer_.data();
  const size_t old_start_index = buffer_.size() - allocated_size_;
  CHECK_LE(static_cast<ssize_t>(old_start_index), removal_index);
  CHECK_LE(removal_index, static_cast<ssize_t>(buffer_.size()));
  CHECK_LE(removal_index + remove_bytes.size(), buffer_.size());
  uint8_t *old_buffer_start = buffer_.data() + old_start_index;
  memmove(old_buffer_start + remove_bytes.size(), old_buffer_start,
          removal_index - old_start_index);
  allocated_size_ -= remove_bytes.size();

  return std::span<uint8_t>{data(), allocated_size_};
}

void AlignedVectorAllocator::Deallocate(std::span<uint8_t>) {
  if (!released_) {
    CHECK(!buffer_.empty())
        << ": Called Deallocate() without a prior allocation.";
  }
  released_ = false;
  buffer_.resize(0);
}

aos::SharedSpan AlignedVectorAllocator::Release() {
  absl::Span<uint8_t> span{data(), allocated_size_};
  std::shared_ptr<SharedSpanHolder> result = std::make_shared<SharedSpanHolder>(
      std::move(buffer_), absl::Span<const uint8_t>());
  result->span = span;
  released_ = true;
  return aos::SharedSpan(result, &(result->span));
}

}  // namespace aos::fbs

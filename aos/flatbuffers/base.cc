#include "aos/flatbuffers/base.h"
namespace aos::fbs {

namespace {
void *DereferenceOffset(uoffset_t *offset) {
  return reinterpret_cast<uint8_t *>(offset) + *offset;
}
}  // namespace

bool ResizeableObject::InsertBytes(void *insertion_point, size_t bytes,
                                   SetZero set_zero) {
  // See comments on InsertBytes() declaration and in FixObjects()
  // implementation below.
  CHECK_LT(buffer_.data(), reinterpret_cast<const uint8_t *>(insertion_point))
      << ": Insertion may not be prior to the start of the buffer.";
  // Check that we started off with a properly aligned size.
  // Doing this CHECK earlier is tricky because if done in the constructor then
  // it executes prior to the Alignment() implementation being available.
  CHECK_EQ(0u, buffer_.size() % Alignment());
  // Note that we will round up the size to the current alignment, so that we
  // ultimately end up only adjusting the buffer size by a multiple of its
  // alignment, to avoid having to do any more complicated bookkeeping.
  const size_t aligned_bytes = PaddedSize(bytes, Alignment());
  if (parent_ != nullptr) {
    return parent_->InsertBytes(insertion_point, aligned_bytes, set_zero);
  } else {
    std::optional<std::span<uint8_t>> new_buffer =
        CHECK_NOTNULL(allocator_)
            ->InsertBytes(insertion_point, aligned_bytes, Alignment(),
                          set_zero);
    if (!new_buffer.has_value()) {
      return false;
    }
    UpdateBuffer(new_buffer.value(),
                 new_buffer.value().data() +
                     (reinterpret_cast<const uint8_t *>(insertion_point) -
                      buffer_.data()),
                 aligned_bytes);
    return true;
  }
}

void ResizeableObject::UpdateBuffer(std::span<uint8_t> new_buffer,
                                    void *modification_point,
                                    ssize_t bytes_inserted) {
  buffer_ = new_buffer;
  FixObjects(modification_point, bytes_inserted);
  ObserveBufferModification();
}

std::span<uint8_t> ResizeableObject::BufferForObject(
    size_t absolute_offset, size_t size, size_t terminal_alignment) {
  const size_t padded_size = PaddedSize(size, terminal_alignment);
  std::span<uint8_t> padded_buffer =
      internal::GetSubSpan(buffer_, absolute_offset, padded_size);
  std::span<uint8_t> object_buffer =
      internal::GetSubSpan(padded_buffer, 0, size);
  std::span<uint8_t> padding = internal::GetSubSpan(padded_buffer, size);
  internal::ClearSpan(padding);
  return object_buffer;
}

void ResizeableObject::FixObjects(void *modification_point,
                                  ssize_t bytes_inserted) {
  CHECK_EQ(bytes_inserted % Alignment(), 0u)
      << ": We only support inserting N * Alignment() bytes at a time. This "
         "may change in the future.";
  for (size_t index = 0; index < NumberOfSubObjects(); ++index) {
    SubObject object = GetSubObject(index);
    const void *const absolute_offset =
        PointerForAbsoluteOffset(*object.absolute_offset);
    if (absolute_offset >= modification_point &&
        object.inline_entry < modification_point) {
      if (*object.inline_entry != 0) {
        CHECK_EQ(static_cast<const void *>(
                     static_cast<const uint8_t *>(absolute_offset) +
                     CHECK_NOTNULL(object.object)->AbsoluteOffsetOffset()),
                 DereferenceOffset(object.inline_entry));
        *object.inline_entry += bytes_inserted;
        CHECK_GE(DereferenceOffset(object.inline_entry), modification_point)
            << ": Encountered offset which points to a now-deleted section "
               "of memory. The offset should have been null'd out prior to "
               "deleting the memory.";
      } else {
        CHECK_EQ(nullptr, object.object);
      }
      *object.absolute_offset += bytes_inserted;
    }
    // We only need to update the object's buffer if it currently exists.
    if (object.object != nullptr) {
      std::span<uint8_t> subbuffer = BufferForObject(
          *object.absolute_offset, object.object->buffer_.size(),
          object.object->Alignment());
      // By convention (enforced in InsertBytes()), the modification_point shall
      // not be at the start of the subobjects data buffer; it may be the byte
      // just past the end of the buffer. This makes it so that is unambiguous
      // which subobject(s) should get the extra space when a buffer size
      // increase is requested on the edge of a buffer.
      if (subbuffer.data() < modification_point &&
          (subbuffer.data() + subbuffer.size()) >= modification_point) {
        subbuffer = {subbuffer.data(), subbuffer.size() + bytes_inserted};
      }
      object.object->UpdateBuffer(subbuffer, modification_point,
                                  bytes_inserted);
    }
  }
}

std::optional<std::span<uint8_t>> VectorAllocator::Allocate(
    size_t size, size_t /*alignment*/, SetZero set_zero) {
  CHECK(buffer_.empty()) << ": Must deallocate before calling Allocate().";
  buffer_.resize(size);
  if (set_zero == SetZero::kYes) {
    memset(buffer_.data(), 0, buffer_.size());
  }
  return std::span<uint8_t>{buffer_.data(), buffer_.size()};
}

std::optional<std::span<uint8_t>> VectorAllocator::InsertBytes(
    void *insertion_point, size_t bytes, size_t /*alignment*/, SetZero) {
  const ssize_t insertion_index =
      reinterpret_cast<uint8_t *>(insertion_point) - buffer_.data();
  CHECK_LE(0, insertion_index);
  CHECK_LE(insertion_index, static_cast<ssize_t>(buffer_.size()));
  buffer_.insert(buffer_.begin() + insertion_index, bytes, 0);
  return std::span<uint8_t>{buffer_.data(), buffer_.size()};
}

std::span<uint8_t> VectorAllocator::RemoveBytes(
    std::span<uint8_t> remove_bytes) {
  const ssize_t removal_index = remove_bytes.data() - buffer_.data();
  CHECK_LE(0, removal_index);
  CHECK_LE(removal_index, static_cast<ssize_t>(buffer_.size()));
  CHECK_LE(removal_index + remove_bytes.size(), buffer_.size());
  buffer_.erase(buffer_.begin() + removal_index,
                buffer_.begin() + removal_index + remove_bytes.size());
  return {buffer_.data(), buffer_.size()};
}

std::optional<std::span<uint8_t>> SpanAllocator::Allocate(size_t size,
                                                          size_t /*alignment*/,
                                                          SetZero set_zero) {
  CHECK(!allocated_);
  if (size > buffer_.size()) {
    return std::nullopt;
  }
  if (set_zero == SetZero::kYes) {
    memset(buffer_.data(), 0, buffer_.size());
  }
  allocated_size_ = size;
  allocated_ = true;
  return internal::GetSubSpan(buffer_, buffer_.size() - size);
}

std::optional<std::span<uint8_t>> SpanAllocator::InsertBytes(
    void *insertion_point, size_t bytes, size_t /*alignment*/,
    SetZero set_zero) {
  uint8_t *insertion_point_typed = reinterpret_cast<uint8_t *>(insertion_point);
  const ssize_t insertion_index = insertion_point_typed - buffer_.data();
  CHECK_LE(0, insertion_index);
  CHECK_LE(insertion_index, static_cast<ssize_t>(buffer_.size()));
  const size_t new_size = allocated_size_ + bytes;
  if (new_size > buffer_.size()) {
    VLOG(1) << ": Insufficient space to grow by " << bytes << " bytes.";
    return std::nullopt;
  }
  const size_t old_start_index = buffer_.size() - allocated_size_;
  const size_t new_start_index = buffer_.size() - new_size;
  memmove(buffer_.data() + new_start_index, buffer_.data() + old_start_index,
          insertion_index - old_start_index);
  if (set_zero == SetZero::kYes) {
    memset(insertion_point_typed - bytes, 0, bytes);
  }
  allocated_size_ = new_size;
  return internal::GetSubSpan(buffer_, buffer_.size() - allocated_size_);
}

std::span<uint8_t> SpanAllocator::RemoveBytes(std::span<uint8_t> remove_bytes) {
  const ssize_t removal_index = remove_bytes.data() - buffer_.data();
  const size_t old_start_index = buffer_.size() - allocated_size_;
  CHECK_LE(static_cast<ssize_t>(old_start_index), removal_index);
  CHECK_LE(removal_index, static_cast<ssize_t>(buffer_.size()));
  CHECK_LE(removal_index + remove_bytes.size(), buffer_.size());
  uint8_t *old_buffer_start = buffer_.data() + old_start_index;
  memmove(old_buffer_start + remove_bytes.size(), old_buffer_start,
          removal_index - old_start_index);
  allocated_size_ -= remove_bytes.size();
  return internal::GetSubSpan(buffer_, buffer_.size() - allocated_size_);
}

void SpanAllocator::Deallocate(std::span<uint8_t>) {
  CHECK(allocated_) << ": Called Deallocate() without a prior allocation.";
  allocated_ = false;
}

namespace internal {
std::ostream &DebugBytes(std::span<const uint8_t> span, std::ostream &os) {
  constexpr size_t kRowSize = 8u;
  for (size_t index = 0; index < span.size(); index += kRowSize) {
    os << std::hex << std::setw(4) << std::setfill('0') << std::uppercase
       << index << ": ";
    for (size_t subindex = 0;
         subindex < kRowSize && (index + subindex) < span.size(); ++subindex) {
      os << std::setw(2) << static_cast<int>(span[index + subindex]) << " ";
    }
    os << "\n";
  }
  os << std::resetiosflags(std::ios_base::basefield | std::ios_base::uppercase);
  return os;
}
}  // namespace internal
}  // namespace aos::fbs

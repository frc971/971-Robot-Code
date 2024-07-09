#ifndef AOS_FLATBUFFERS_BUILDER_H_
#define AOS_FLATBUFFERS_BUILDER_H_
#include "aos/flatbuffers.h"
#include "aos/flatbuffers/static_table.h"
namespace aos::fbs {

// Builder class to handle the memory for a static flatbuffer object. This
// fulfills a similar role to the FlatBufferBuilder type in the traditional API.
// Typical usage:
//  aos::fbs::VectorAllocator allocator;
//  Builder<TestTableStatic> builder(&allocator);
//  TestTableStatic *object = builder.get();
//  object->set_scalar(123);
//
// At all points you will have a valid and complete flatbuffer, so you never
// need to call Finish() or anything. You can just directly use the flatbuffer
// as if it is a real flatbuffer.
template <typename T>
class Builder final : public ResizeableObject {
 public:
  static constexpr size_t kBufferSize = T::kRootSize;
  static constexpr size_t kAlign = T::kAlign;
  // Note on memory initialization: We zero-initialize all the memory that we
  // create at the start. While this can be overkill, it is simpler to manage
  // the alternatives, and we don't currently have a clear performance need for
  // doing this more piecemeal. Note that the memory that this zero-initializes
  // falls into three categories:
  // 1. Padding that we need to zero-initialize (arguably we could get away
  //    without initializing our padding, but leaving uninitialized memory
  //    floating around generally isn't great, especially since we generally
  //    want to be able to compress flatbuffer messages efficiently).
  // 2. Memory that will end up getting used.
  // 3. Memory corresponding to sub-tables/vectors that may or may not end up
  //    getting used; if it is not used, we want it to get zero-initialized
  //    since it will effectively be padding. If it is used, then the
  //    zero-initialization is redundant.
  // For messages with large byte buffers (e.g., for camera messages), we
  // typically expect that the user will end up dynamically resizing the buffer
  // rather than having the length statically set. In those cases, the user
  // still has the ability to select whether or not the new memory gets
  // zero-initialized.
  Builder(Allocator *allocator)
      : ResizeableObject(
            allocator->AllocateOrDie(kBufferSize, T::kAlign, SetZero::kYes),
            allocator),
        flatbuffer_start_(BufferStart(buffer_)),
        flatbuffer_(internal::GetSubSpan(buffer_, flatbuffer_start_, T::kSize),
                    this) {
    SetPrefix();
  }
  Builder(std::unique_ptr<Allocator> allocator =
              std::make_unique<AlignedVectorAllocator>())
      : ResizeableObject(
            allocator->AllocateOrDie(kBufferSize, T::kAlign, SetZero::kYes),
            std::move(allocator)),
        flatbuffer_start_(BufferStart(buffer_)),
        flatbuffer_(internal::GetSubSpan(buffer_, flatbuffer_start_, T::kSize),
                    this) {
    SetPrefix();
  }
  Builder(Builder &&other)
      : ResizeableObject(std::move(other)),
        flatbuffer_(std::move(other.flatbuffer_)) {
    flatbuffer_start_ = other.flatbuffer_start_;
    other.flatbuffer_start_ = 0;
  }

  ~Builder() {
    if (allocator() != nullptr) {
      allocator()->Deallocate(buffer_);
    }
  }

  // Returns an object containing the current raw flatbuffer type. Note that if
  // the allocator type allows changes to the structure/amount of allocated
  // memory, the underlying buffer will not be stable and so the returned
  // FlatbufferSpan may be invalidated by mutations to the flatbuffer.
  FlatbufferSpan<typename T::Flatbuffer> AsFlatbufferSpan() {
    return {buffer()};
  }
  FlatbufferSpan<const typename T::Flatbuffer> AsFlatbufferSpan() const {
    return {buffer()};
  }

  // Returns true if the flatbuffer is validly constructed. Should always return
  // true (barring some sort of memory corruption). Exposed for convenience.
  bool Verify() { return AsFlatbufferSpan().Verify(); }

  // Returns the actual object for you to operate on and construct the
  // flatbuffer. Unlike AsFlatbufferSpan(), this will be stable.
  T *get() { return &flatbuffer_.t; }
  T &operator*() { return *get(); }
  T *operator->() { return get(); }

 private:
  size_t Alignment() const override { return flatbuffer_.t.Alignment(); }
  size_t NumberOfSubObjects() const override { return 1; }
  void SetPrefix() {
    // We can't do much if the provided buffer isn't at least 4-byte aligned,
    // because we are required to put the root table offset at the start of the
    // buffer.
    CHECK_EQ(reinterpret_cast<size_t>(buffer_.data()) % alignof(uoffset_t), 0u);
    *reinterpret_cast<uoffset_t *>(buffer_.data()) = flatbuffer_start_;
  }
  // Manually aligns the start of the actual flatbuffer to handle the alignment
  // offset.
  static size_t BufferStart(std::span<uint8_t> buffer) {
    CHECK_EQ(reinterpret_cast<size_t>(buffer.data()) % T::kAlign, 0u)
        << "Failed to allocate data of length " << buffer.size()
        << " with alignment " << T::kAlign;

    return aos::fbs::AlignOffset(
               reinterpret_cast<size_t>(buffer.data()) + sizeof(uoffset_t),
               T::kAlign, T::kAlignOffset) -
           reinterpret_cast<size_t>(buffer.data());
  }

  // Some allocators don't do a great job of supporting arbitrary alignments; if
  // the alignment of the buffer changes, we need to reshuffle everything to
  // continue guaranteeing alignment.
  void ObserveBufferModification() override {
    const size_t new_start = BufferStart(buffer_);
    if (new_start != flatbuffer_start_) {
      const size_t used_size = flatbuffer_.t.buffer().size();
      CHECK_LT(flatbuffer_start_ + used_size, buffer_.size());
      CHECK_LT(new_start + used_size, buffer_.size());
      memmove(buffer_.data() + new_start, buffer_.data() + flatbuffer_start_,
              used_size);
      flatbuffer_.t.UpdateBuffer(
          internal::GetSubSpan(buffer_, new_start, used_size),
          buffer_.data() + new_start, 0);
      flatbuffer_start_ = new_start;
      SetPrefix();
    }
  }
  using ResizeableObject::SubObject;
  SubObject GetSubObject(size_t index) override {
    CHECK_EQ(0u, index);
    return {reinterpret_cast<uoffset_t *>(buffer_.data()), &flatbuffer_.t,
            &flatbuffer_start_};
  }
  // Offset from the start of the buffer to the actual start of the flatbuffer
  // (identical to the root offset of the flatbuffer).
  size_t flatbuffer_start_;
  internal::TableMover<T> flatbuffer_;
};
}  // namespace aos::fbs
#endif  // AOS_FLATBUFFERS_BUILDER_H_

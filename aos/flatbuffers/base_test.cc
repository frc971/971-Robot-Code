#include "aos/flatbuffers/base.h"

#include <stddef.h>

#include <algorithm>

#include "gtest/gtest.h"

namespace aos::fbs::testing {
// Tests that AlignOffset() behaves as expected.
TEST(BaseTest, AlignOffset) {
  EXPECT_EQ(0, AlignOffset(0, 4));
  EXPECT_EQ(4, AlignOffset(4, 4));
  EXPECT_EQ(8, AlignOffset(5, 4));
  EXPECT_EQ(8, AlignOffset(6, 4));
  EXPECT_EQ(8, AlignOffset(7, 4));
}

// Tests that AlignOffset handles the alignment point being nonzero.  This shows
// up when you want 8 byte alignment 4 bytes into the start of the buffer, and
// don't want to pad out the front of the buffer.
TEST(BaseTest, AlignOffsetWithOffset) {
  EXPECT_EQ(4, AlignOffset(4, 4, 4));

  EXPECT_EQ(4, AlignOffset(0, 8, 4));
  EXPECT_EQ(4, AlignOffset(1, 8, 4));
  EXPECT_EQ(4, AlignOffset(2, 8, 4));
  EXPECT_EQ(4, AlignOffset(3, 8, 4));
  EXPECT_EQ(4, AlignOffset(4, 8, 4));
  EXPECT_EQ(12, AlignOffset(5, 8, 4));
}

inline constexpr size_t kDefaultSize = AlignedVectorAllocator::kAlignment * 2;
template <typename T>
class AllocatorTest : public ::testing::Test {
 protected:
  AllocatorTest() : allocator_(std::make_unique<T>()) {}
  alignas(64) std::array<uint8_t, kDefaultSize> buffer_;
  // unique_ptr so that we can destroy the allocator at will.
  std::unique_ptr<T> allocator_;
};

template <>
AllocatorTest<SpanAllocator>::AllocatorTest()
    : allocator_(std::make_unique<SpanAllocator>(
          std::span<uint8_t>{buffer_.data(), buffer_.size()})) {}

using AllocatorTypes = ::testing::Types<SpanAllocator, AlignedVectorAllocator,
                                        FixedStackAllocator<kDefaultSize>>;
TYPED_TEST_SUITE(AllocatorTest, AllocatorTypes);

// Tests that we can create and not use a VectorAllocator.
TYPED_TEST(AllocatorTest, UnusedAllocator) {}

// Tests that a simple allocate works.
TYPED_TEST(AllocatorTest, BasicAllocate) {
  std::span<uint8_t> span =
      this->allocator_->Allocate(kDefaultSize, 4, SetZero::kYes).value();
  ASSERT_EQ(kDefaultSize, span.size());
  // We set SetZero::kYes; it should be zero-initialized.
  EXPECT_EQ(kDefaultSize, std::count(span.begin(), span.end(), 0));
  this->allocator_->Deallocate(span);
}

// Tests that we can insert bytes into an arbitrary spot in the buffer.
TYPED_TEST(AllocatorTest, InsertBytes) {
  const size_t half_size = kDefaultSize / 2;
  std::span<uint8_t> span =
      this->allocator_->Allocate(half_size, 4, SetZero::kYes).value();
  ASSERT_EQ(half_size, span.size());
  // Set the span with some sentinel values so that we can detect that the
  // insertion occurred correctly.
  for (size_t ii = 0; ii < span.size(); ++ii) {
    span[ii] = ii + 1;
  }

  // Insert new bytes such that one old byte will still be at the start.
  span = this->allocator_
             ->InsertBytes(span.data() + 1u, half_size, 0, SetZero::kYes)
             .value();
  ASSERT_EQ(kDefaultSize, span.size());
  size_t index = 0;
  EXPECT_EQ(1u, span[index]);
  index++;
  for (; index < half_size + 1u; ++index) {
    EXPECT_EQ(0u, span[index]);
  }
  for (; index < span.size(); ++index) {
    EXPECT_EQ(index - half_size + 1, span[index]);
  }
  this->allocator_->Deallocate(span);
}

// Tests that all allocators return data aligned to the requested alignment.
TYPED_TEST(AllocatorTest, Alignment) {
  for (size_t alignment : {4, 8, 16, 32, 64}) {
    std::span<uint8_t> span =
        this->allocator_->Allocate(kDefaultSize, alignment, SetZero::kYes)
            .value();
    EXPECT_EQ(reinterpret_cast<size_t>(span.data()) % alignment, 0);
    this->allocator_->Deallocate(span);
  }
}

// Tests that we can remove bytes from an arbitrary spot in the buffer.
TYPED_TEST(AllocatorTest, RemoveBytes) {
  // Deletion doesn't require resizing, so we don't need to worry about it being
  // larger than the alignment to test everything.  The test requires the size
  // to be < 255 to store the sentinal values.
  const size_t kDefaultSize = 128;

  const size_t half_size = kDefaultSize / 2;
  std::span<uint8_t> span =
      this->allocator_->Allocate(kDefaultSize, 4, SetZero::kYes).value();
  ASSERT_EQ(kDefaultSize, span.size());
  // Set the span with some sentinel values so that we can detect that the
  // removal occurred correctly.
  for (size_t ii = 0; ii < span.size(); ++ii) {
    span[ii] = ii + 1;
  }

  // Remove bytes such that one old byte will remain at the start, and a chunk
  // of 8 bytes will be cut out after that..
  span = this->allocator_->RemoveBytes(span.subspan(1, half_size));
  ASSERT_EQ(half_size, span.size());
  size_t index = 0;
  EXPECT_EQ(1u, span[index]);
  index++;
  for (; index < span.size(); ++index) {
    EXPECT_EQ(index + half_size + 1, span[index]);
  }
  this->allocator_->Deallocate(span);
}

// Tests that if we fail to deallocate that we fail during destruction.
TYPED_TEST(AllocatorTest, NoDeallocate) {
  EXPECT_DEATH(
      {
        EXPECT_EQ(
            4, this->allocator_->Allocate(4, 4, SetZero::kYes).value().size());
        this->allocator_.reset();
      },
      "Must deallocate");
}

// Tests that if we never allocate that we cannot deallocate.
TYPED_TEST(AllocatorTest, NoAllocateThenDeallocate) {
  EXPECT_DEATH(this->allocator_->Deallocate(std::span<uint8_t>()),
               "prior allocation");
}

// Tests that if we attempt to allocate more than the backing span allows that
// we correctly return an empty span.
TEST(SpanAllocatorTest, OverAllocate) {
  std::vector<uint8_t> buffer(kDefaultSize);
  SpanAllocator allocator({buffer.data(), buffer.size()});
  EXPECT_FALSE(
      allocator.Allocate(kDefaultSize + 1u, 0, SetZero::kYes).has_value());
}

// Tests that if we attempt to insert more than the backing span allows that
// we correctly return an empty span.
TEST(SpanAllocatorTest, OverInsert) {
  std::vector<uint8_t> buffer(kDefaultSize);
  SpanAllocator allocator({buffer.data(), buffer.size()});
  std::span<uint8_t> span =
      allocator.Allocate(kDefaultSize, 1, SetZero::kYes).value();
  EXPECT_EQ(kDefaultSize, span.size());
  EXPECT_FALSE(
      allocator.InsertBytes(span.data(), 1u, 0, SetZero::kYes).has_value());
  allocator.Deallocate(span);
}

// Because we really aren't meant to instantiate ResizeableObject's directly (if
// nothing else it has virtual member functions), define a testing
// implementation.

class TestResizeableObject : public ResizeableObject {
 public:
  TestResizeableObject(std::span<uint8_t> buffer, ResizeableObject *parent)
      : ResizeableObject(buffer, parent) {}
  TestResizeableObject(std::span<uint8_t> buffer, Allocator *allocator)
      : ResizeableObject(buffer, allocator) {}
  virtual ~TestResizeableObject() {}
  using ResizeableObject::SubObject;
  bool InsertBytes(void *insertion_point, size_t bytes) {
    return ResizeableObject::InsertBytes(insertion_point, bytes, SetZero::kYes)
        .has_value();
  }
  TestResizeableObject(TestResizeableObject &&) = default;

  struct TestObject {
    uoffset_t inline_entry_offset;
    std::unique_ptr<TestResizeableObject> object;
    size_t absolute_offset;
  };

  // Adds a new object of the requested size.
  void AddEntry(uoffset_t inline_entry_offset, size_t absolute_offset,
                size_t buffer_size, bool set_object) {
    *reinterpret_cast<uoffset_t *>(buffer_.data() + inline_entry_offset) =
        set_object ? (absolute_offset - inline_entry_offset) : 0;
    objects_.emplace_back(
        TestObject{inline_entry_offset, nullptr, absolute_offset});
    if (set_object) {
      objects_.back().object = std::make_unique<TestResizeableObject>(
          buffer().subspan(absolute_offset, buffer_size), this);
    }
  }

  size_t NumberOfSubObjects() const override { return objects_.size(); }
  SubObject GetSubObject(size_t index) override {
    TestObject &subobject = objects_.at(index);
    return {reinterpret_cast<uoffset_t *>(buffer_.data() +
                                          subobject.inline_entry_offset),
            subobject.object.get(), &subobject.absolute_offset};
  }

  TestObject &GetObject(size_t index) { return objects_.at(index); }

  size_t Alignment() const override { return 64; }

 private:
  std::vector<TestObject> objects_;
};

class ResizeableObjectTest : public ::testing::Test {
 protected:
  static constexpr size_t kInitialSize = 128;
  ResizeableObjectTest()
      : object_(allocator_.Allocate(kInitialSize, 4, SetZero::kYes).value(),
                &allocator_) {}
  ~ResizeableObjectTest() { allocator_.Deallocate(object_.buffer()); }
  AlignedVectorAllocator allocator_;
  TestResizeableObject object_;
};

// Tests that if we created an object and then do nothing with it that nothing
// untoward happens.
TEST_F(ResizeableObjectTest, DoNothing) {}

// Test that when we move the ResizeableObject we clear the reference to the old
// buffer.
TEST_F(ResizeableObjectTest, Move) {
  TestResizeableObject target_object = std::move(object_);
  ASSERT_EQ(0u, object_.buffer().size());
  ASSERT_EQ(kInitialSize, target_object.buffer().size());
}

// Tests the pathways for resizing a nested ResizeableObject works.
TEST_F(ResizeableObjectTest, ResizeNested) {
  constexpr size_t kAbsoluteOffset = 64;
  object_.AddEntry(4, kAbsoluteOffset, 64, true);
  TestResizeableObject *subobject = object_.GetObject(0).object.get();
  object_.AddEntry(0, kAbsoluteOffset, 64, false);
  EXPECT_EQ(60, *object_.GetSubObject(0).inline_entry);
  EXPECT_EQ(0, *object_.GetSubObject(1).inline_entry);
  EXPECT_EQ(64, object_.GetObject(0).object->buffer().data() -
                    object_.buffer().data());

  constexpr size_t kInsertBytes = 5;
  // The insert should succeed.
  ASSERT_TRUE(
      subobject->InsertBytes(subobject->buffer().data() + 1u, kInsertBytes));
  // We should now observe the size of the buffers increasing, but the start
  // _not_ moving.
  // We should've rounded the insert up to the alignment we areusing (64 bytes).
  EXPECT_EQ(kInitialSize + 64, object_.buffer().size());
  EXPECT_EQ(128, subobject->buffer().size());
  EXPECT_EQ(60, *object_.GetSubObject(0).inline_entry);
  EXPECT_EQ(0, *object_.GetSubObject(1).inline_entry);
  EXPECT_EQ(kAbsoluteOffset, object_.GetObject(0).absolute_offset);
  EXPECT_EQ(kAbsoluteOffset, object_.GetObject(1).absolute_offset);

  // And next we insert before the subobjects, so that we can see their offsets
  // shift. The insert should succeed.
  ASSERT_TRUE(object_.InsertBytes(subobject->buffer().data(), kInsertBytes));
  EXPECT_EQ(kInitialSize + 2 * 64, object_.buffer().size());
  EXPECT_EQ(128, subobject->buffer().size());
  EXPECT_EQ(60 + 64, *object_.GetSubObject(0).inline_entry);
  // The unpopulated object's inline entry should not have changed since
  // it was zero.
  EXPECT_EQ(0, *object_.GetSubObject(1).inline_entry);
  EXPECT_EQ(kAbsoluteOffset + 64, object_.GetObject(0).absolute_offset);
  EXPECT_EQ(kAbsoluteOffset + 64, object_.GetObject(1).absolute_offset);
}

}  // namespace aos::fbs::testing

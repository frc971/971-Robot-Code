#ifndef AOS_FLATBUFFERS_H_
#define AOS_FLATBUFFERS_H_

#include <array>
#include <string_view>

#include "flatbuffers/flatbuffers.h"
#include "glog/logging.h"

namespace aos {

// This class is a base class for all sizes of array backed allocators.
class FixedAllocatorBase : public flatbuffers::Allocator {
 public:
  // TODO(austin): Read the contract for these.
  uint8_t *allocate(size_t) override;

  void deallocate(uint8_t *, size_t) override { is_allocated_ = false; }

  uint8_t *reallocate_downward(uint8_t *, size_t, size_t, size_t,
                               size_t) override;

  virtual const uint8_t *data() const = 0;
  virtual uint8_t *data() = 0;
  virtual size_t size() const = 0;

  void Reset() { is_allocated_ = false; }

 private:
  bool is_allocated_ = false;
};

// This class is a fixed memory allocator which holds the data for a flatbuffer
// in an array.
class FixedAllocator : public FixedAllocatorBase {
 public:
  FixedAllocator(size_t size) : buffer_(size, 0) {}

  uint8_t *data() override { return &buffer_[0]; }
  const uint8_t *data() const override { return &buffer_[0]; }
  size_t size() const override { return buffer_.size(); }

  // Releases the data in the buffer.
  std::vector<uint8_t> release() { return std::move(buffer_); }

 private:
  std::vector<uint8_t> buffer_;
};

// This class adapts a preallocated memory region to an Allocator.
class PreallocatedAllocator : public FixedAllocatorBase {
 public:
  PreallocatedAllocator(void *data, size_t size) : data_(data), size_(size) {}
  uint8_t *data() override { return reinterpret_cast<uint8_t *>(data_); }
  const uint8_t *data() const override {
    return reinterpret_cast<const uint8_t *>(data_);
  }
  size_t size() const override { return size_; }

 private:
  void* data_;
  size_t size_;
};

// Base class representing an object which holds the memory representing a root
// flatbuffer.
template <typename T>
class Flatbuffer {
 public:
  virtual ~Flatbuffer() {}
  // Returns the MiniReflectTypeTable for T.
  static const flatbuffers::TypeTable *MiniReflectTypeTable() {
    return T::MiniReflectTypeTable();
  }

  // Returns a message from the buffer.
  const T &message() const {
    return *flatbuffers::GetRoot<T>(reinterpret_cast<const void *>(data()));
  }
  // Returns a mutable message.  It can be mutated via the flatbuffer rules.
  T *mutable_message() {
    return flatbuffers::GetMutableRoot<T>(reinterpret_cast<void *>(data()));
  }

  virtual const uint8_t *data() const = 0;
  virtual uint8_t *data() = 0;
  virtual size_t size() const = 0;
};

// String backed flatbuffer.
template <typename T>
class FlatbufferString : public Flatbuffer<T> {
 public:
  // Builds a flatbuffer using the contents of the string.
  FlatbufferString(const std::string_view data) : data_(data) {}
  // Builds a Flatbuffer by copying the data from the other flatbuffer.
  FlatbufferString(const Flatbuffer<T> &other) {
    data_ = std::string(other.data(), other.size());
  }

  // Coppies the data from the other flatbuffer.
  FlatbufferString &operator=(const Flatbuffer<T> &other) {
    data_ = std::string(other.data(), other.size());
    return *this;
  }

  virtual ~FlatbufferString() override {}

  const uint8_t *data() const override {
    return reinterpret_cast<const uint8_t *>(data_.data());
  }
  uint8_t *data() override { return reinterpret_cast<uint8_t *>(data_.data()); }
  size_t size() const override { return data_.size(); }

 private:
  std::string data_;
};

// Vector backed flatbuffer.
template <typename T>
class FlatbufferVector : public Flatbuffer<T> {
 public:
  // Builds a Flatbuffer around a vector.
  FlatbufferVector(std::vector<uint8_t> &&data) : data_(std::move(data)) {}

  // Builds a Flatbuffer by copying the data from the other flatbuffer.
  FlatbufferVector(const Flatbuffer<T> &other)
      : data_(other.data(), other.data() + other.size()) {}

  // Move constructor.
  FlatbufferVector(Flatbuffer<T> &&other) : data_(std::move(other.data())) {}

  // Copies the data from the other flatbuffer.
  FlatbufferVector &operator=(const Flatbuffer<T> &other) {
    data_ = std::vector<uint8_t>(other.data(), other.data() + other.size());
    return *this;
  }

  // Constructs an empty flatbuffer of type T.
  static FlatbufferVector<T> Empty() {
    return FlatbufferVector<T>(std::vector<uint8_t>{});
  }

  virtual ~FlatbufferVector() override {}

  const uint8_t *data() const override { return data_.data(); }
  uint8_t *data() override { return data_.data(); }
  size_t size() const override { return data_.size(); }

 private:
  std::vector<uint8_t> data_;
};

// This object associates the message type with the memory storing the
// flatbuffer.  This only stores root tables.
//
// From a usage point of view, pointers to the data are very different than
// pointers to the tables.
template <typename T>
class FlatbufferDetachedBuffer final : public Flatbuffer<T> {
 public:
  // Builds a Flatbuffer by taking ownership of the buffer.
  FlatbufferDetachedBuffer(flatbuffers::DetachedBuffer &&buffer)
      : buffer_(::std::move(buffer)) {}

  // Builds a flatbuffer by taking ownership of the buffer from the other
  // flatbuffer.
  FlatbufferDetachedBuffer(FlatbufferDetachedBuffer &&fb)
      : buffer_(::std::move(fb.buffer_)) {}
  FlatbufferDetachedBuffer &operator=(FlatbufferDetachedBuffer &&fb) {
    ::std::swap(buffer_, fb.buffer_);
    return *this;
  }

  virtual ~FlatbufferDetachedBuffer() override {}

  // Constructs an empty flatbuffer of type T.
  static FlatbufferDetachedBuffer<T> Empty() {
    flatbuffers::FlatBufferBuilder fbb;
    fbb.ForceDefaults(1);
    const auto end = fbb.EndTable(fbb.StartTable());
    fbb.Finish(flatbuffers::Offset<flatbuffers::Table>(end));
    return FlatbufferDetachedBuffer<T>(fbb.Release());
  }

  // Returns references to the buffer, and the data.
  const flatbuffers::DetachedBuffer &buffer() const { return buffer_; }
  const uint8_t *data() const override { return buffer_.data(); }
  uint8_t *data() override { return buffer_.data(); }
  size_t size() const override { return buffer_.size(); }

 private:
  flatbuffers::DetachedBuffer buffer_;
};

// This object associates the message type with the memory storing the
// flatbuffer.  This only stores root tables.
//
// From a usage point of view, pointers to the data are very different than
// pointers to the tables.
template <typename T>
class SizePrefixedFlatbufferDetachedBuffer final : public Flatbuffer<T> {
 public:
  // Builds a Flatbuffer by taking ownership of the buffer.
  SizePrefixedFlatbufferDetachedBuffer(flatbuffers::DetachedBuffer &&buffer)
      : buffer_(::std::move(buffer)) {
    CHECK_GE(buffer_.size(), sizeof(flatbuffers::uoffset_t));
  }

  // Builds a flatbuffer by taking ownership of the buffer from the other
  // flatbuffer.
  SizePrefixedFlatbufferDetachedBuffer(
      SizePrefixedFlatbufferDetachedBuffer &&fb)
      : buffer_(::std::move(fb.buffer_)) {}
  SizePrefixedFlatbufferDetachedBuffer &operator=(
      SizePrefixedFlatbufferDetachedBuffer &&fb) {
    ::std::swap(buffer_, fb.buffer_);
    return *this;
  }

  virtual ~SizePrefixedFlatbufferDetachedBuffer() override {}

  // Returns references to the buffer, and the data.
  const flatbuffers::DetachedBuffer &buffer() const { return buffer_; }
  const uint8_t *data() const override {
    return buffer_.data() + sizeof(flatbuffers::uoffset_t);
  }
  uint8_t *data() override {
    return buffer_.data() + sizeof(flatbuffers::uoffset_t);
  }
  size_t size() const override {
    return buffer_.size() - sizeof(flatbuffers::uoffset_t);
  }

 private:
  flatbuffers::DetachedBuffer buffer_;
};
// TODO(austin): Need a way to get our hands on the max size.  Can start with
// "large" for now.

}  // namespace aos

#endif  // AOS_FLATBUFFERS_H_

#ifndef AOS_FLATBUFFERS_H_
#define AOS_FLATBUFFERS_H_

#include "flatbuffers/flatbuffers.h"

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

 private:
  bool is_allocated_ = false;
};

// This class is a fixed memory allocator which holds the data for a flatbuffer
// in an array.
template <size_t S>
class FixedAllocator : public FixedAllocatorBase {
 public:
  uint8_t *data() override { return &buffer_[0]; }
  const uint8_t *data() const override { return &buffer_[0]; }
  size_t size() const override { return buffer_.size(); }

 private:
  std::array<uint8_t, S> buffer_;
};

// Base class representing an object which holds the memory representing a root
// flatbuffer.
template <typename T>
class Flatbuffer {
 public:
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

// Array backed flatbuffer.
template <typename T>
class FlatbufferArray : public Flatbuffer<T> {
 public:
  // Builds a Flatbuffer by copying the data from the other flatbuffer.
  FlatbufferArray(const Flatbuffer<T> &other) {
    CHECK_LE(other.size(), data_.size());

    memcpy(data_.data(), other.data(), other.size());
    size_ = other.size();
  }

  // Coppies the data from the other flatbuffer.
  FlatbufferArray &operator=(const Flatbuffer<T> &other) {
    CHECK_LE(other.size(), data_.size());

    memcpy(data_.data(), other.data(), other.size());
    size_ = other.size();
    return *this;
  }

  // Creates a builder wrapping the underlying data.
  flatbuffers::FlatBufferBuilder FlatBufferBuilder() {
    data_.deallocate(data_.data(), data_.size());
    flatbuffers::FlatBufferBuilder fbb(data_.size(), &data_);
    fbb.ForceDefaults(1);
    return fbb;
  }

  const uint8_t *data() const override { return data_.data(); }
  uint8_t *data() override { return data_.data(); }
  size_t size() const override { return size_; }

 private:
  FixedAllocator<8 * 1024> data_;
  size_t size_ = data_.size();
};

// This object associates the message type with the memory storing the
// flatbuffer.  This only stores root tables.
//
// From a usage point of view, pointers to the data are very different than
// pointers to the tables.
template <typename T>
class FlatbufferDetachedBuffer : public Flatbuffer<T> {
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

// TODO(austin): Need a way to get our hands on the max size.  Can start with
// "large" for now.

}  // namespace aos

#endif  // AOS_FLATBUFFERS_H_

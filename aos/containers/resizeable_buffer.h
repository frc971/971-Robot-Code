#ifndef AOS_CONTAINERS_RESIZEABLE_BUFFER_H_
#define AOS_CONTAINERS_RESIZEABLE_BUFFER_H_

#include <stdlib.h>

#include <memory>

#include "glog/logging.h"

namespace aos {

// Kind of like a subset of vector<uint8_t>, but with less destructor calls.
// When building unoptimized, especially with sanitizers, the vector<uint8_t>
// version ends up being really slow in tests.
class ResizeableBuffer {
 public:
  ResizeableBuffer() = default;

  ResizeableBuffer(const ResizeableBuffer &other) { *this = other; }
  ResizeableBuffer(ResizeableBuffer &&other) { *this = std::move(other); }
  ResizeableBuffer &operator=(const ResizeableBuffer &other) {
    resize(other.size());
    memcpy(storage_.get(), other.storage_.get(), size());
    return *this;
  }
  ResizeableBuffer &operator=(ResizeableBuffer &&other) {
    std::swap(storage_, other.storage_);
    std::swap(size_, other.size_);
    std::swap(capacity_, other.capacity_);
    return *this;
  }

  uint8_t *data() { return static_cast<uint8_t *>(storage_.get()); }
  const uint8_t *data() const {
    return static_cast<const uint8_t *>(storage_.get());
  }

  uint8_t *begin() { return data(); }
  const uint8_t *begin() const { return data(); }
  uint8_t *end() { return data() + size(); }
  const uint8_t *end() const { return data() + size(); }

  size_t size() const { return size_; }
  size_t capacity() const { return capacity_; }

  void resize(size_t new_size) {
    if (new_size > capacity_) {
      Allocate(new_size);
    }
    size_ = new_size;
  }

  void erase_front(size_t count) {
    if (count == 0) {
      return;
    }
    CHECK_LE(count, size_);
    memmove(static_cast<void *>(data()), static_cast<void *>(data() + count),
            size_ - count);
    size_ -= count;
  }

  void push_back(uint8_t x) {
    if (size_ == capacity_) {
      Allocate(std::max<size_t>(16, size_ * 2));
    }
    *end() = x;
    ++size_;
    CHECK_LE(size_, capacity_);
  }

 private:
  // We need this silly function because C++ is bad, and extensions to it which
  // we work with make it a true nightmare.
  //
  // (a) You can't easily write out the signature of free because it depends on
  // whether exceptions are enabled or not. You could use decltype, but see (b).
  //
  // (b) You can't easily write &free because CUDA overloads it with a
  // __device__ version. You could cast to the appropriate version, but see (a).
  //
  // There's probably some kind of SFINAE thing which could find the matching
  // signature from a set of choices, and then we could just
  // static_cast<TheSignature>(&free). However, that sounds like a nightmare,
  // especially because it has to conditionally enable the part mentioning CUDA
  // identifiers in the preprocessor. This little function is way simpler.
  static void DoFree(void *p) { free(p); }

  void Allocate(size_t new_capacity) {
    void *const old = storage_.release();
    storage_.reset(CHECK_NOTNULL(realloc(old, new_capacity)));
    capacity_ = new_capacity;
  }

  std::unique_ptr<void, decltype(&DoFree)> storage_{nullptr, &DoFree};
  size_t size_ = 0, capacity_ = 0;
};

}  // namespace aos

#endif  // AOS_CONTAINERS_RESIZEABLE_BUFFER_H_

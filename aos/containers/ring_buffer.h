#ifndef AOS_RING_BUFFER_H_
#define AOS_RING_BUFFER_H_

#include <array>

namespace aos {

// This is a helper to keep track of some amount of recent data. As you push
// data into the ring buffer, it gets stored. If the buffer becomes full, it
// will start overwriting the oldest data.
template <typename Data, size_t buffer_size>
class RingBuffer {
 public:
  static constexpr size_t kBufferSize = buffer_size;

  constexpr RingBuffer() {}

  // Add an item to the RingBuffer, overwriting the oldest element if necessary
  void Push(const Data &data) {
    if (full()) {
      data_[oldest_] = data;
      oldest_ = (oldest_ + 1) % buffer_size;
    } else {
      data_[(oldest_ + size_) % buffer_size] = data;
      ++size_;
    }
  }

  void Shift() {
    oldest_ = (oldest_ + 1) % buffer_size;
    --size_;
  }

  // Return the value of the index requested, adjusted so that the RingBuffer
  // contains the oldest element first and the newest last.
  Data &operator[](size_t index) {
    return data_[(oldest_ + index) % buffer_size];
  }

  const Data &operator[](size_t index) const {
    return data_[(oldest_ + index) % buffer_size];
  }

  // Returns the capacity of the RingBuffer
  size_t capacity() const { return buffer_size; }

  // Returns the number of elements stored in the RingBuffer
  size_t size() const { return size_; }

  // Is the RingBuffer empty or full?
  bool empty() const { return size_ == 0; }

  bool full() const { return size_ == buffer_size; }

  // Clears all the data out of the buffer.
  void Reset() { size_ = 0; }

  class iterator {
   public:
    using iterator_category = ::std::forward_iterator_tag;
    using value_type = Data;
    using difference_type = ::std::ptrdiff_t;
    using pointer = Data *;
    using reference = Data &;

    explicit iterator(RingBuffer *buffer, size_t index)
        : buffer_(buffer), index_(index) {}

    iterator &operator++() {
      ++index_;
      return *this;
    }
    iterator operator++(int) {
      iterator retval = *this;
      ++(*this);
      return retval;
    }
    bool operator==(iterator other) const {
      return buffer_ == other.buffer_ && index_ == other.index_;
    }
    bool operator!=(iterator other) const { return !(*this == other); }
    reference operator*() const { return (*buffer_)[index_]; }

   private:
    RingBuffer *buffer_;
    size_t index_;
  };

  class const_iterator {
   public:
    using iterator_category = ::std::forward_iterator_tag;
    using value_type = Data;
    using difference_type = ::std::ptrdiff_t;
    using pointer = Data *;
    using reference = Data &;

    explicit const_iterator(const RingBuffer *buffer, size_t index)
        : buffer_(buffer), index_(index) {}

    const_iterator &operator++() {
      ++index_;
      return *this;
    }
    const_iterator operator++(int) {
      const_iterator retval = *this;
      ++(*this);
      return retval;
    }
    bool operator==(const_iterator other) const {
      return buffer_ == other.buffer_ && index_ == other.index_;
    }
    bool operator!=(const_iterator other) const { return !(*this == other); }
    const Data &operator*() const { return (*buffer_)[index_]; }

   private:
    const RingBuffer *buffer_;
    size_t index_;
  };

  iterator begin() { return iterator(this, 0); }
  iterator end() { return iterator(this, size()); }

  const_iterator begin() const { return const_iterator(this, 0); }
  const_iterator end() const { return const_iterator(this, size()); }

 private:
  ::std::array<Data, buffer_size> data_;

  // Oldest contains the oldest item added to the RingBuffer which will be the
  // next one to be overwritten
  size_t oldest_ = 0;
  size_t size_ = 0;
};

}  // namespace aos

#endif  // AOS_RING_BUFFER_H_

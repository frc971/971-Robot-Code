#ifndef AOS_COMMON_RING_BUFFER_H_
#define AOS_COMMON_RING_BUFFER_H_

#include <array>

namespace aos {

// This is a helper to keep track of some amount of recent data. As you push
// data into the ring buffer, it gets stored. If the buffer becomes full, it
// will start overwriting the oldest data.
template <typename Data, size_t buffer_size>
class RingBuffer {
 public:
  static constexpr size_t kBufferSize = buffer_size;

  RingBuffer() {}

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

 private:
  ::std::array<Data, buffer_size> data_;

  // Oldest contains the oldest item added to the RingBuffer which will be the
  // next one to be overwritten
  size_t oldest_ = 0;
  size_t size_ = 0;
};

}  // namespace aos

#endif  // AOS_COMMON_RING_BUFFER_H_

#ifndef MOTORS_USB_QUEUE_H_
#define MOTORS_USB_QUEUE_H_

#include <memory>
#include <atomic>

namespace frc971 {
namespace teensy {

// A FIFO queue which reads/writes variable-sized chunks.
//
// Reading data happens-after it is written. However, external synchronization
// between multiple readers/writers is required.
class Queue {
 public:
  Queue(size_t size) : size_(size), data_(new char[size]) {}

  // Writes as much of in_data as will fit to the queue. Returns the number of
  // bytes written.
  size_t Write(const char *in_data, size_t in_size);

  // Reads up to out_size from the queue into out_data. Returns the number of
  // bytes read.
  size_t Read(char *out_data, size_t out_size);

  size_t data_queued() const {
    return space_used(read_cursor_.load(::std::memory_order_relaxed),
                      write_cursor_.load(::std::memory_order_relaxed));
  }

  size_t space_available() const {
    return size_ - data_queued() - 1;
  }

  bool empty() const {
    return read_cursor_.load(::std::memory_order_relaxed) ==
           write_cursor_.load(::std::memory_order_relaxed);
  }

 private:
  size_t space_used(size_t read_cursor, size_t write_cursor) const {
    size_t r = write_cursor - read_cursor;
    if (r > size_) {
      r = size_ + r;
    }
    return r;
  }

  size_t wrap(size_t cursor) const {
    if (cursor >= size_) {
      return cursor - size_;
    } else {
      return cursor;
    }
  }

  const size_t size_;
  const ::std::unique_ptr<char[]> data_;

  // The next index we're going to read from.
  ::std::atomic<size_t> read_cursor_{0};
  // The next index we're going to write to.
  ::std::atomic<size_t> write_cursor_{0};
};

}  // namespace teensy
}  // namespace frc971

#endif  // MOTORS_USB_QUEUE_H_

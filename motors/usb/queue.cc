#include "motors/usb/queue.h"

#include <string.h>

namespace frc971 {
namespace teensy {

size_t Queue::Read(char *out_data, size_t out_size) {
  const size_t read_cursor = read_cursor_.load(::std::memory_order_relaxed);
  const size_t write_cursor = write_cursor_.load(::std::memory_order_acquire);
  const size_t r = ::std::min(out_size, space_used(read_cursor, write_cursor));
  const size_t first_chunk = ::std::min(r, size_ - read_cursor);
  memcpy(out_data, &data_[read_cursor], first_chunk);
  memcpy(out_data + first_chunk, &data_[0], r - first_chunk);
  read_cursor_.store(wrap(read_cursor + r), ::std::memory_order_release);
  return r;
}

size_t Queue::Write(const char *in_data, size_t in_size) {
  const size_t write_cursor = write_cursor_.load(::std::memory_order_relaxed);
  const size_t read_cursor = read_cursor_.load(::std::memory_order_acquire);
  const size_t r =
      ::std::min(in_size, size_ - space_used(read_cursor, write_cursor) - 1);
  const size_t first_chunk = ::std::min(r, size_ - write_cursor);
  memcpy(&data_[write_cursor], in_data, first_chunk);
  memcpy(&data_[0], in_data + first_chunk, r - first_chunk);
  write_cursor_.store(wrap(write_cursor + r), ::std::memory_order_release);
  return r;
}

}  // namespace teensy
}  // namespace frc971

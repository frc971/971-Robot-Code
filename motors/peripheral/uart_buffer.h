#ifndef MOTORS_PERIPHERAL_UART_BUFFER_H_
#define MOTORS_PERIPHERAL_UART_BUFFER_H_

#include <array>

#include "third_party/GSL/include/gsl/gsl"

namespace frc971 {
namespace teensy {

// Manages a circular buffer of data to send out.
template<int kSize>
class UartBuffer {
 public:
  // Returns the number of characters added.
  __attribute__((warn_unused_result)) int PushSpan(gsl::span<const char> data);

  // max is the maximum size the returned span should be.
  // The data in the result is only valid until another method is called.
  // Note that this may not return all available data when doing so would
  // require wrapping around, but it will always return a non-empty span if any
  // data is available.
  gsl::span<const char> PopSpan(int max);

  bool empty() const { return size_ == 0; }
  bool full() const { return size_ == kSize; }

  void clear() { size_ = 0; }

  // This may only be called when !empty().
  char PopSingle();
  // This may only be called when !full().
  void PushSingle(char c);

  static constexpr int size() { return kSize; }

 private:
  // The index at which we will pop the next character.
  int start_ = 0;
  // How many characters we currently have.
  int size_ = 0;

  ::std::array<char, kSize> data_;
};

template <int kSize>
int UartBuffer<kSize>::PushSpan(gsl::span<const char> data) {
  const int end_location = (start_ + size_) % kSize;
  const int remaining_end = ::std::min(kSize - size_, kSize - end_location);
  const int on_end = ::std::min<int>(data.size(), remaining_end);
  if (on_end > 0) {
    memcpy(&data_[end_location], data.data(), on_end);
  }
  size_ += on_end;
  const int not_on_end = data.size() - on_end;
  if (not_on_end == 0) {
    return data.size();
  }

  const int remaining_start = ::std::min(kSize - size_, start_);
  const int on_start = ::std::min(not_on_end, remaining_start);
  memcpy(data_.data(), &data[on_end], on_start);
  size_ += on_start;
  return on_end + on_start;
}

template <int kSize>
gsl::span<const char> UartBuffer<kSize>::PopSpan(int max) {
  const size_t result_size = std::min(max, std::min(kSize - start_, size_));
  const auto result = gsl::span<const char>(data_).subspan(start_, result_size);
  start_ = (start_ + result_size) % kSize;
  size_ -= result_size;
  return result;
}

template <int kSize>
char UartBuffer<kSize>::PopSingle() {
  const char r = data_[start_];
  --size_;
  start_ = (start_ + 1) % kSize;
  return r;
}

template <int kSize>
void UartBuffer<kSize>::PushSingle(char c) {
  const int end_location = (start_ + size_) % kSize;
  data_[end_location] = c;
  ++size_;
}

}  // namespace teensy
}  // namespace frc971

#endif  // MOTORS_PERIPHERAL_UART_BUFFER_H_

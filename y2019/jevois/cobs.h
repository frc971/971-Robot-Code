#ifndef Y2019_JEVOIS_COBS_H_
#define Y2019_JEVOIS_COBS_H_

#include <stdint.h>

#include <algorithm>
#include <array>

#include "third_party/GSL/include/gsl/gsl"

// This file contains code for encoding and decoding Consistent Overhead Byte
// Stuffing data. <http://www.stuartcheshire.org/papers/cobsforton.pdf> has
// details on what this entails and why it's a good idea.

namespace frc971 {
namespace jevois {

constexpr size_t CobsMaxEncodedSize(size_t decoded_size) {
  return decoded_size + ((decoded_size + 253) / 254);
}

// Encodes some data using COBS.
// input is the data to encode. Its size may be at most max_decoded_size.
// output_buffer is where to store the result.
// Returns a span in output_buffer which has no 0 bytes.
template <size_t max_decoded_size>
gsl::span<char> CobsEncode(
    gsl::span<const char> input,
    std::array<char, CobsMaxEncodedSize(max_decoded_size)> *output_buffer);

// Decodes some COBS-encoded data.
// input is the data to decide. Its size may be at most
// CobsMaxEncodedSize(max_decoded_size), and it may not have any 0 bytes.
// output_buffer is where to store the result.
// Returns a span in output_buffer.
// If the input data is invalid, this will simply stop when either the input or
// output buffer is exhausted and return the result.
template <size_t max_decoded_size>
gsl::span<char> CobsDecode(gsl::span<const char> input,
                           std::array<char, max_decoded_size> *output_buffer);

// Manages scanning a stream of bytes for 0s and exposing the resulting buffers.
//
// This will silently truncate packets longer than max_decoded_size, and ignore
// empty packets.
template <size_t max_decoded_size>
class CobsPacketizer {
 public:
  CobsPacketizer() = default;
  CobsPacketizer(const CobsPacketizer &) = delete;
  CobsPacketizer &operator=(const CobsPacketizer &) = delete;

  // Parses some new data. received_packet() will be filled out to the end of
  // a packet if the end delimeters for any packets are present in new_data. If
  // multiple end delimiters are present, received_packet() will be filled out
  // to an arbitrary one of them.
  void ParseData(gsl::span<const char> new_data);

  // Returns the most-recently-parsed packet.
  // If this is empty, it indicates no packet was received.
  gsl::span<const char> received_packet() const { return complete_packet_; }
  void clear_received_packet() { complete_packet_ = gsl::span<char>(); }

 private:
  using Buffer = std::array<char, CobsMaxEncodedSize(max_decoded_size)>;

  void CopyData(gsl::span<const char> input) {
    const size_t size = std::min(input.size(), remaining_active_.size());
    for (size_t i = 0; i < size; ++i) {
      remaining_active_[i] = input[i];
    }
    remaining_active_ = remaining_active_.subspan(size);
  }

  void FinishPacket() {
    const Buffer &active_buffer = buffers_[active_index_];
    complete_packet_ =
        gsl::span<const char>(active_buffer)
            .first(active_buffer.size() - remaining_active_.size());

    active_index_ = 1 - active_index_;
    remaining_active_ = buffers_[active_index_];
  }

  Buffer buffers_[2];
  // The remaining space in the active buffer.
  gsl::span<char> remaining_active_ = buffers_[0];
  // The last complete packet we parsed.
  gsl::span<const char> complete_packet_;
  int active_index_ = 0;
};

template <size_t max_decoded_size>
gsl::span<char> CobsEncode(
    gsl::span<const char> input,
    std::array<char, CobsMaxEncodedSize(max_decoded_size)> *output_buffer) {
  static_assert(max_decoded_size > 0, "Empty buffers not supported");
  if (static_cast<size_t>(input.size()) > max_decoded_size) {
    __builtin_trap();
  }
  auto input_pointer = input.begin();
  auto output_pointer = output_buffer->begin();
  auto code_pointer = output_pointer;
  ++output_pointer;
  uint8_t code = 1;
  while (input_pointer < input.end()) {
    if (output_pointer >= output_buffer->end()) {
      __builtin_trap();
    }
    if (*input_pointer == 0u) {
      *code_pointer = code;
      code_pointer = output_pointer;
      ++output_pointer;
      code = 1;
    } else {
      *output_pointer = *input_pointer;
      ++output_pointer;
      ++code;
      if (code == 0xFFu) {
        *code_pointer = 0xFF;
        code_pointer = output_pointer;
        ++output_pointer;
        code = 1;
      }
    }
    ++input_pointer;
  }
  *code_pointer = code;
  if (output_pointer > output_buffer->end()) {
    __builtin_trap();
  }
  return gsl::span<char>(*output_buffer)
      .subspan(0, output_pointer - output_buffer->begin());
}

template <size_t max_decoded_size>
gsl::span<char> CobsDecode(gsl::span<const char> input,
                           std::array<char, max_decoded_size> *output_buffer) {
  static_assert(max_decoded_size > 0, "Empty buffers not supported");
  if (static_cast<size_t>(input.size()) >
      CobsMaxEncodedSize(max_decoded_size)) {
    __builtin_trap();
  }
  auto input_pointer = input.begin();
  auto output_pointer = output_buffer->begin();
  while (input_pointer < input.end()) {
    const uint8_t code = *input_pointer;
    ++input_pointer;
    for (uint8_t i = 1; i < code; ++i) {
      if (input_pointer == input.end()) {
        break;
      }
      if (output_pointer == output_buffer->end()) {
        return gsl::span<char>(*output_buffer);
      }
      *output_pointer = *input_pointer;
      ++output_pointer;
      ++input_pointer;
    }
    if (output_pointer == output_buffer->end()) {
      return gsl::span<char>(*output_buffer);
    }
    if (code < 0xFFu) {
      *output_pointer = 0;
      ++output_pointer;
    }
  }
  return gsl::span<char>(*output_buffer)
      .subspan(0, output_pointer - output_buffer->begin() - 1);
}

template <size_t max_decoded_size>
void CobsPacketizer<max_decoded_size>::ParseData(
    gsl::span<const char> new_data) {
  // Find where the active packet ends.
  const auto first_end = std::find(new_data.begin(), new_data.end(), 0);
  if (first_end == new_data.end()) {
    // This is the common case, where there's no packet end in new_data.
    CopyData(new_data);
    return;
  }

  // Copy any remaining data for the active packet, and then finish it.
  const auto first_end_index = first_end - new_data.begin();
  CopyData(new_data.subspan(0, first_end_index));
  FinishPacket();

  // Look for where the last packet end is.
  const auto first_end_reverse = new_data.rend() - first_end_index - 1;
  const auto last_end = std::find(new_data.rbegin(), first_end_reverse, 0);
  if (last_end == first_end_reverse) {
    // If we didn't find another zero afterwards, then copy the rest of the data
    // into the new packet and we're done.
    CopyData(new_data.subspan(first_end_index + 1));
    return;
  }

  // Otherwise, find the second-to-the-end packet end, which is where the last
  // packet starts.
  auto new_start = last_end;
  auto new_end = new_data.rbegin();
  // If a second packet ends at the end of new_data, then we want to grab it
  // instead of ignoring it.
  if (new_start == new_end) {
    ++new_end;
    new_start = std::find(new_end, first_end_reverse, 0);
  }

  // Being here means we found the end of multiple packets in new_data. Only
  // copy the data which is part of the last one.
  const auto new_start_index = new_data.rend() - new_start;
  CopyData(new_data.subspan(new_start_index, new_start - new_end));
  if (last_end == new_data.rbegin()) {
    // If we also found the end of a packet, then return it.
    FinishPacket();
  }
}

}  // namespace jevois
}  // namespace frc971

#endif  // Y2019_JEVOIS_COBS_H_

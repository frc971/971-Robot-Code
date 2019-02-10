#ifndef Y2019_JEVOIS_COBS_H_
#define Y2019_JEVOIS_COBS_H_

#include <stdint.h>

#include <array>

#include "aos/logging/logging.h"
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

template <size_t max_decoded_size>
gsl::span<char> CobsEncode(
    gsl::span<const char> input,
    std::array<char, CobsMaxEncodedSize(max_decoded_size)> *output_buffer) {
  static_assert(max_decoded_size > 0, "Empty buffers not supported");
  CHECK_LE(static_cast<size_t>(input.size()), max_decoded_size);
  auto input_pointer = input.begin();
  auto output_pointer = output_buffer->begin();
  auto code_pointer = output_pointer;
  ++output_pointer;
  uint8_t code = 1;
  while (input_pointer < input.end()) {
    CHECK(output_pointer < output_buffer->end());
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
  CHECK(output_pointer <= output_buffer->end());
  return gsl::span<char>(*output_buffer)
      .subspan(0, output_pointer - output_buffer->begin());
}

template <size_t max_decoded_size>
gsl::span<char> CobsDecode(gsl::span<const char> input,
                           std::array<char, max_decoded_size> *output_buffer) {
  static_assert(max_decoded_size > 0, "Empty buffers not supported");
  CHECK_LE(static_cast<size_t>(input.size()),
           CobsMaxEncodedSize(max_decoded_size));
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

}  // namespace jevois
}  // namespace frc971

#endif  // Y2019_JEVOIS_COBS_H_

#ifndef AOS_UTIL_BITPACKING_H_
#define AOS_UTIL_BITPACKING_H_

#include <assert.h>

#include <type_traits>

#include "third_party/GSL/include/gsl/gsl"

namespace aos {

template <typename Integer>
typename std::enable_if<std::is_unsigned<Integer>::value, Integer>::type
MaskOnes(size_t bits) {
  // Get these edge cases out of the way first, so we can subtract 1 from bits
  // safely later without getting a negative number.
  if (bits == 0) {
    return 0;
  }
  if (bits == 1) {
    return 1;
  }
  static constexpr Integer kOne = 1;
  // Note that we shift at most by bits - 1. bits == sizeof(Integer) * 8 is
  // valid, and shifting by the width of a type is undefined behavior, so we
  // need to get a bit fancy to make it all work. Just ORing high_bit in
  // explicitly at the end makes it work.
  const Integer high_bit = kOne << (bits - 1);
  return (high_bit - kOne) | high_bit;
}

template <typename Integer, size_t bits, size_t offset>
typename std::enable_if<std::is_unsigned<Integer>::value &&
                        sizeof(Integer) * 8 >= bits>::type
PackBits(const Integer value, const gsl::span<char> destination) {
  assert(static_cast<size_t>(destination.size()) * 8u >= bits + offset);
  size_t bits_completed = 0;
  while (bits_completed < bits) {
    // Which logical bit (through all the bytes) we're writing at.
    const size_t output_bit = offset + bits_completed;
    // The lowest-numbered bit in the current byte we're writing to.
    const size_t output_min_bit = output_bit % 8;
    // The number of bits we're writing in this byte.
    const size_t new_bits = std::min(8 - output_min_bit, bits - bits_completed);
    // The highest-numbered bit in the current byte we're writing to.
    const size_t output_max_bit = output_min_bit + new_bits;
    // The new bits to set in the this byte.
    const uint8_t new_byte_part =
        (value >> bits_completed) & MaskOnes<Integer>(new_bits);
    // A mask of bits to keep from the current value of this byte. Start with
    // just the low ones.
    uint8_t existing_mask = MaskOnes<uint8_t>(output_min_bit);
    // And then add in the high bits to keep.
    existing_mask |= MaskOnes<uint8_t>(std::max<int>(8 - output_max_bit, 0))
                     << output_max_bit;
    // The index of the byte we're writing to.
    const size_t byte_index = output_bit / 8;
    // The full new value of the current byte. Start with just the existing bits
    // we're not touching.
    uint8_t new_byte = destination[byte_index] & existing_mask;
    // Add in the new part.
    new_byte |= new_byte_part << output_min_bit;
    destination[byte_index] = new_byte;
    bits_completed += new_bits;
  }
  assert(bits_completed == bits);
}

template <typename Integer, size_t bits, size_t offset>
typename std::enable_if<std::is_unsigned<Integer>::value &&
                        sizeof(Integer) * 8 >= bits, Integer>::type
UnpackBits(const gsl::span<const char> source) {
  Integer result = 0;
  assert(static_cast<size_t>(source.size()) * 8u >= bits + offset);
  size_t bits_completed = 0;
  while (bits_completed < bits) {
    // Which logical bit (through all the bytes) we're reading at.
    const size_t input_bit = offset + bits_completed;
    // The lowest-numbered bit in the current byte we're reading from.
    const size_t input_min_bit = input_bit % 8;
    // The number of bits we're reading in this byte.
    const size_t new_bits = std::min(8 - input_min_bit, bits - bits_completed);
    // The index of the byte we're reading from.
    const size_t byte_index = input_bit / 8;
    // The part of the current byte we're actually reading.
    const uint8_t new_byte_part =
        (source[byte_index] >> input_min_bit) & MaskOnes<Integer>(new_bits);
    result |= static_cast<Integer>(new_byte_part) << bits_completed;
    bits_completed += new_bits;
  }
  assert(bits_completed == bits);
  return result;
}

template <int bits>
uint32_t FloatToIntLinear(float min, float max, float value) {
  static_assert(bits <= 31, "Only support 32-bit outputs for now");
  static_assert(bits >= 1, "Bits must be positive");
  // Start such that value in [0, 1) maps to [0, 2**bits) in the final
  // result.
  float result = (value - min) / (max - min);
  // Multiply so that value is in [0, 2**bits).
  // Make sure we do the shifting in a 32-bit integer, despite C++'s weird
  // integer promotions, which is safe because bits is at most 31.
  result *= static_cast<uint32_t>(UINT32_C(1) << bits);
  if (result <= 0.0f) {
    return 0;
  }
  const float max_result = MaskOnes<uint32_t>(bits);
  if (result >= max_result) {
    return max_result;
  }
  return static_cast<uint32_t>(result);
}

template <int bits>
float IntToFloatLinear(float min, float max, uint32_t value) {
  static_assert(bits <= 31, "Only support 32-bit outputs for now");
  static_assert(bits >= 1, "Bits must be positive");
  const float max_value = MaskOnes<uint32_t>(bits);
  if (value > max_value) {
    value = max_value;
  }
  // Start such that result in [0, 2**bits) maps to [min, max) in the final
  // result.
  float result = value;
  // Offset by half a bit so we return a value in the middle of each one.
  // This causes us to return the middle floating point value which could be
  // represented by a given integer value.
  result += 0.5f;
  // Multiply so that result is in [0, 1).
  // Make sure we do the shifting in a 32-bit integer, despite C++'s weird
  // integer promotions, which is safe because bits is at most 31.
  result *= 1.0f / static_cast<uint32_t>(UINT32_C(1) << bits);
  return min + result * (max - min);
}

}  // namespace aos

#endif  // AOS_UTIL_BITPACKING_H_

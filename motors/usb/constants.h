#ifndef MOTORS_USB_CONSTANTS_H_
#define MOTORS_USB_CONSTANTS_H_

#include <stdint.h>

namespace frc971 {
namespace teensy {

enum class Direction : uint32_t {
  kTx = 1 << 1,
  kRx = 0,
};

enum class EvenOdd : uint32_t {
  kOdd = 1 << 0,
  kEven = 0,
};

constexpr static inline EvenOdd EvenOddInverse(EvenOdd odd) {
  return static_cast<EvenOdd>(static_cast<uint32_t>(odd) ^
                              static_cast<uint32_t>(EvenOdd::kOdd));
}

// Returns 0 for kEven and 1 for kOdd. This is useful for indexing into arrays
// and similar things.
constexpr static inline int EvenOddIndex(EvenOdd odd) {
  static_assert(static_cast<int>(EvenOdd::kOdd) == 1, "Value changed");
  return static_cast<int>(odd);
}

enum class EndpointBufferState : int {
  // The values are chosen carefully so bit arithmetic can efficiently
  // manipulate these values. This math is all encapsulated in methods
  // immediately following.
  // Bit 0 is even full.
  // Bit 1 is odd full.
  // Bit 2 is which one to fill next (1 for odd).
  // Bit 3 is which one to empty next (1 for odd).

  // Both are empty and we should fill the even one first.
  kBothEmptyEvenFirst = 0x0,
  kBothEmptyOddFirst = 0xC,
  kEvenFull = 0x5,
  kOddFull = 0xA,
  // Both are full and we should empty the even one first.
  kBothFullEvenFirst = 0x3,
  kBothFullOddFirst = 0xF,
};

// kBothEmptyEvenFirst fill even kEvenFull fill odd kBothFullEvenFirst
//   empty even kOddFull empty odd kBothEmptyEvenFirst

// Returns true if state has at least one empty buffer.
constexpr static inline bool BufferStateHasEmpty(EndpointBufferState state) {
  return (static_cast<int>(state) & 0x3) != 0x3;
}

// Returns true if state has at least one full buffer.
constexpr static inline bool BufferStateHasFull(EndpointBufferState state) {
  return (static_cast<int>(state) & 0x3) != 0;
}

// Returns the next buffer to fill from state.
//
// This won't make sense if !BufferStateHasEmpty(state).
constexpr static inline EvenOdd BufferStateToFill(EndpointBufferState state) {
  return (static_cast<int>(state) & 0x4) ? EvenOdd::kOdd : EvenOdd::kEven;
}

// Returns the next buffer to empty from state.
//
// This won't make sense if !BufferStateHasFull(state).
constexpr static inline EvenOdd BufferStateToEmpty(EndpointBufferState state) {
  return (static_cast<int>(state) & 0x8) ? EvenOdd::kOdd : EvenOdd::kEven;
}

// Returns the new state after filling BufferStateToFill(state).
//
// This won't make sense if !BufferStateHasEmpty(state).
constexpr static inline EndpointBufferState BufferStateAfterFill(
    EndpointBufferState state) {
  return static_cast<EndpointBufferState>(
      // XOR with bit 2 to toggle which is next.
      (static_cast<int>(state) ^ 0x4) |
      // Set the bit corresponding to the buffer which was filled.
      (1 << EvenOddIndex(BufferStateToFill(state))));
}

// Returns the new state after emptying BufferStateToEmpty(state).
//
// This won't make sense if !BufferStateHasFull(state).
constexpr static inline EndpointBufferState BufferStateAfterEmpty(
    EndpointBufferState state) {
  return static_cast<EndpointBufferState>(
      // XOR with bit 3 to toggle which is next.
      (static_cast<int>(state) ^ 0x8) &
      // Clear the bit corresponding to the buffer which was emptied.
      ~(1 << EvenOddIndex(BufferStateToEmpty(state))));
}

enum class Data01 : uint32_t {
  kData1 = 1 << 6,
  kData0 = 0,
};

constexpr static inline Data01 Data01Inverse(Data01 toggle) {
  return static_cast<Data01>(static_cast<uint32_t>(toggle) ^
                             static_cast<uint32_t>(Data01::kData1));
}

}  // namespace teensy
}  // namespace frc971

#endif  // MOTORS_USB_CONSTANTS_H_

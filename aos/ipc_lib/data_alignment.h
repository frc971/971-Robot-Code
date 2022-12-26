#ifndef AOS_IPC_LIB_DATA_ALIGNMENT_H_
#define AOS_IPC_LIB_DATA_ALIGNMENT_H_

#include "glog/logging.h"

namespace aos {

// All data buffers sent over or received from a channel will guarantee this
// alignment for their end. Flatbuffers aligns from the end, so this is what
// matters.
//
// 128 is a reasonable choice for now:
//   Cortex-A72 (Raspberry Pi 4) and Cortex-A53 (Xavier AGX) both have 64 byte
//   cache lines.
//   V4L2 requires 64 byte alignment for USERPTR buffers.
//
//   rockpi v4l2 requires 128 byte alignment for USERPTR buffers.
static constexpr size_t kChannelDataAlignment = 128;

template <typename T>
inline void CheckChannelDataAlignment(T *data, size_t size) {
  CHECK_EQ((reinterpret_cast<uintptr_t>(data) + size) % kChannelDataAlignment,
           0u)
      << ": data pointer is not end aligned as it should be: " << data << " + "
      << size;
}

// Aligns the beginning of a channel data buffer. There must be
// kChannelDataAlignment-1 extra bytes beyond the end to potentially use after
// aligning it.
inline char *RoundChannelData(char *data, size_t size) {
  const uintptr_t data_value = reinterpret_cast<uintptr_t>(data);
  const uintptr_t data_end = data_value + size;
  const uintptr_t data_end_max = data_end + (kChannelDataAlignment - 1);
  const uintptr_t rounded_data_end =
      data_end_max - (data_end_max % kChannelDataAlignment);
  const uintptr_t rounded_data = rounded_data_end - size;
  return reinterpret_cast<char *>(rounded_data);
}

// The size of the redzone we maintain outside each message's data to help
// detect out-of-bounds writes.
static constexpr size_t kChannelDataRedzone = 32;

}  // namespace aos

#endif  // AOS_IPC_LIB_DATA_ALIGNMENT_H_

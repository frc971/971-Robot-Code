// Copyright (c) 2020, the YACCLAB contributors, as
// shown by the AUTHORS file. All rights reserved.
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.
//
// See
// https://iris.unimore.it/bitstream/11380/1179616/1/2018_TPDS_Optimized_Block_Based_Algorithms_to_Label_Connected_Components_on_GPUs.pdf
// for more details.
//
// This file was started from the YACCLAB implementation, but is now quite
// different to match what the april tag detection algorithm wants.  The notable
// changes are:
//  * 4 way connectivity on the background at the same time as the foreground.
//  * 0 and 255 are the only colors considered, 127 means "I'm my own blob".
//  * Blob size is also computed in parallel using atomics to aid blob size
//    filtering.
//

#include "frc971/orin/labeling_allegretti_2019_BKE.h"

#include "absl/log/check.h"
#include "absl/log/log.h"

#include "cuda_runtime.h"
#include "device_launch_parameters.h"

#define BLOCK_ROWS 16
#define BLOCK_COLS 16

namespace {

//         This is a block-based algorithm.
// Blocks are 2x2 sized, with internal pixels named as:
//                       +---+
//                       |a b|
//                       |c d|
//                       +---+
//
//       Neighbour blocks of block X are named as:
//                      +-+-+-+
//                      |P|Q|R|
//                      +-+-+-+
//                      |S|X|
//                      +-+-+

enum class Info : uint8_t {
  a = 0,
  b = 1,
  c = 2,
  d = 3,
  P = 4,
  Q = 5,
  R = 6,
  S = 7
};

// Only use it with unsigned numeric types
template <typename T>
__device__ __forceinline__ uint8_t HasBit(const T bitmap, Info pos) {
  return (bitmap >> static_cast<uint8_t>(pos)) & 1;
}

template <typename T>
__device__ __forceinline__ uint8_t HasBit(const T bitmap, uint8_t pos) {
  return (bitmap >> pos) & 1;
}

// Only use it with unsigned numeric types
__device__ __forceinline__ void SetBit(uint8_t &bitmap, Info pos) {
  bitmap |= (1 << static_cast<uint8_t>(pos));
}

// Returns the root index of the UFTree
__device__ uint32_t Find(const uint32_t *s_buf, uint32_t n) {
  while (s_buf[n] != n) {
    n = s_buf[n];
  }
  return n;
}

// Returns the root index of the UFTree, re-assigning ourselves as we go.
__device__ uint32_t FindAndCompress(uint32_t *s_buf, uint32_t n) {
  uint32_t id = n;
  while (s_buf[n] != n) {
    n = s_buf[n];
    s_buf[id] = n;
  }
  return n;
}

// Merges the UFTrees of a and b, linking one root to the other
__device__ void Union(uint32_t *s_buf, uint32_t a, uint32_t b) {
  bool done;

  do {
    a = Find(s_buf, a);
    b = Find(s_buf, b);

    if (a < b) {
      uint32_t old = atomicMin(s_buf + b, a);
      done = (old == b);
      b = old;
    } else if (b < a) {
      uint32_t old = atomicMin(s_buf + a, b);
      done = (old == a);
      a = old;
    } else {
      done = true;
    }

  } while (!done);
}

// Initializes the labels in an image to hold the masks, info, and UF tree
// pointers needed for the next steps.
__global__ void InitLabeling(const GpuImage<uint8_t> img,
                             GpuImage<uint32_t> labels) {
  const unsigned row = (blockIdx.y * blockDim.y + threadIdx.y) * 2;
  const unsigned col = (blockIdx.x * blockDim.x + threadIdx.x) * 2;
  const uint32_t img_index = row * img.step + col;
  const uint32_t foreground_labels_index = row * labels.step + col;
  const uint32_t background_labels_index = (row + 1) * labels.step + col;

  if (row >= labels.rows || col >= labels.cols) {
    return;
  }

  uint32_t P_foreground = 0;
  uint32_t P_background = 0;

  // Bitmask representing two kinds of information
  // Bits 0, 1, 2, 3 are set if pixel a, b, c, d are foreground, respectively
  // Bits 4, 5, 6, 7 are set if block P, Q, R, S need to be merged to X in
  // Merge phase
  uint8_t info_foreground = 0;
  uint8_t info_left_background = 0;
  uint8_t info_right_background = 0;

  uint8_t buffer alignas(int)[4];
  *(reinterpret_cast<int *>(buffer)) = 0;

  // Read pairs of consecutive values in memory at once
  // This does not depend on endianness
  *(reinterpret_cast<uint16_t *>(buffer)) =
      *(reinterpret_cast<uint16_t *>(img.data + img_index));

  *(reinterpret_cast<uint16_t *>(buffer + 2)) =
      *(reinterpret_cast<uint16_t *>(img.data + img_index + img.step));

  // P is a bitmask saying where to check.
  //
  //                     0  1  2  3
  //                       +----+
  //                     4 |a  b| 7
  //                     8 |c  d| 11
  //                       +----+
  //                    12  13 14 15
  if (buffer[0] == 255u) {
    P_foreground |= 0x777;
    SetBit(info_foreground, Info::a);
  } else if (buffer[0] == 0u) {
    // This is the background, we are only doing 4 way connectivity, only look
    // in the 4 directions.
    P_background |= 0x272;
    SetBit(info_left_background, Info::a);
  }

  if (buffer[1] == 255u) {
    P_foreground |= (0x777 << 1);
    SetBit(info_foreground, Info::b);
  } else if (buffer[1] == 0u) {
    P_background |= (0x272 << 1);
    SetBit(info_right_background, Info::b);
  }

  if (buffer[2] == 255u) {
    P_foreground |= (0x777 << 4);
    SetBit(info_foreground, Info::c);
  } else if (buffer[2] == 0u) {
    P_background |= (0x272 << 4);
    SetBit(info_left_background, Info::c);
  }

  if (buffer[3] == 255u) {
    SetBit(info_foreground, Info::d);
  } else if (buffer[3] == 0u) {
    SetBit(info_right_background, Info::d);
  }

  if (col == 0) {
    P_foreground &= 0xEEEE;
    P_background &= 0xEEEE;
  }
  if (col + 2 >= img.cols) {
    P_foreground &= 0x7777;
    P_background &= 0x7777;
  }

  if (row == 0) {
    P_foreground &= 0xFFF0;
    P_background &= 0xFFF0;
  }
  if (row + 2 >= img.rows) {
    P_foreground &= 0x0FFF;
    P_background &= 0x0FFF;
  }

  // P is now ready to be used to find neighbour blocks
  // P value avoids range errors
  int father_offset_foreground = 0;
  int father_offset_left_background = 0;
  int father_offset_right_background = 0;

  // P square
  if (HasBit(P_foreground, 0) && img.data[img_index - img.step - 1] == 255u) {
    father_offset_foreground = -(2 * (labels.step) + 2);
  }

  // Q square
  if ((HasBit(P_foreground, 1) && img.data[img_index - img.step] == 255u) ||
      (HasBit(P_foreground, 2) && img.data[img_index + 1 - img.step] == 255u)) {
    if (!father_offset_foreground) {
      father_offset_foreground = -(2 * (labels.step));
    } else {
      SetBit(info_foreground, Info::Q);
    }
  }
  if ((HasBit(P_background, 1) && img.data[img_index - img.step] == 0u)) {
    father_offset_left_background = -2 * labels.step;
  }
  if ((HasBit(P_background, 2) && img.data[img_index + 1 - img.step] == 0u)) {
    father_offset_right_background = -2 * labels.step;
  }

  // R square
  if (HasBit(P_foreground, 3) && img.data[img_index + 2 - img.step] == 255u) {
    if (!father_offset_foreground) {
      father_offset_foreground = -(2 * (labels.step) - 2);
    } else {
      SetBit(info_foreground, Info::R);
    }
  }

  // S square
  if ((HasBit(P_foreground, 4) && img.data[img_index - 1] == 255u) ||
      (HasBit(P_foreground, 8) && img.data[img_index + img.step - 1] == 255u)) {
    if (!father_offset_foreground) {
      father_offset_foreground = -2;
    } else {
      SetBit(info_foreground, Info::S);
    }
  }
  if ((HasBit(P_background, 4) && img.data[img_index - 1] == 0u) ||
      (HasBit(P_background, 8) && img.data[img_index + img.step - 1] == 0u)) {
    if (!father_offset_left_background) {
      father_offset_left_background = -1;
    } else {
      SetBit(info_left_background, Info::S);
    }
  }

  if ((HasBit(info_left_background, Info::a) &&
       HasBit(info_right_background, Info::b)) ||
      (HasBit(info_left_background, Info::c) &&
       HasBit(info_right_background, Info::d))) {
    if (!father_offset_right_background) {
      father_offset_right_background = -1;
    } else {
      SetBit(info_right_background, Info::S);
    }
  }

  // Now, write everything back out to memory.
  *reinterpret_cast<uint64_t *>(labels.data + foreground_labels_index) =
      static_cast<uint64_t>(foreground_labels_index +
                            father_offset_foreground) +
      (static_cast<uint64_t>(info_foreground) << 32) +
      (static_cast<uint64_t>(info_left_background) << 40) +
      (static_cast<uint64_t>(info_right_background) << 48);

  *reinterpret_cast<uint64_t *>(labels.data + background_labels_index) =
      static_cast<uint64_t>(background_labels_index +
                            father_offset_left_background) +
      (static_cast<uint64_t>(background_labels_index +
                             father_offset_right_background + 1)
       << 32);
}

__global__ void Compression(GpuImage<uint32_t> labels) {
  const unsigned row = (blockIdx.y * blockDim.y + threadIdx.y) * 2;
  const unsigned col = (blockIdx.x * blockDim.x + threadIdx.x) * 2;
  const uint32_t foreground_labels_index = row * labels.step + col;
  const uint32_t background_labels_index = (row + 1) * labels.step + col;

  if (row >= labels.rows || col >= labels.cols) {
    return;
  }

  FindAndCompress(labels.data, foreground_labels_index);
  FindAndCompress(labels.data, background_labels_index);
  FindAndCompress(labels.data, background_labels_index + 1);
}

__global__ void Merge(GpuImage<uint32_t> labels) {
  const unsigned row = (blockIdx.y * blockDim.y + threadIdx.y) * 2;
  const unsigned col = (blockIdx.x * blockDim.x + threadIdx.x) * 2;
  const uint32_t foreground_labels_index = row * labels.step + col;
  const uint32_t background_labels_index = (row + 1) * labels.step + col;

  if (row >= labels.rows || col >= labels.cols) {
    return;
  }

  const uint32_t info = *(labels.data + foreground_labels_index + 1);

  const uint8_t info_foreground = info & 0xff;
  const uint8_t info_left_background = (info >> 8) & 0xff;
  const uint8_t info_right_background = (info >> 16) & 0xff;

  if (HasBit(info_foreground, Info::Q)) {
    Union(labels.data, foreground_labels_index,
          foreground_labels_index - 2 * (labels.step));
  }

  if (HasBit(info_foreground, Info::R)) {
    Union(labels.data, foreground_labels_index,
          foreground_labels_index - 2 * (labels.step) + 2);
  }

  if (HasBit(info_foreground, Info::S)) {
    Union(labels.data, foreground_labels_index, foreground_labels_index - 2);
  }

  if (HasBit(info_left_background, Info::S)) {
    Union(labels.data, background_labels_index, background_labels_index - 1);
  }
  if (HasBit(info_right_background, Info::S)) {
    Union(labels.data, background_labels_index + 1, background_labels_index);
  }
}

__global__ void FinalLabeling(GpuImage<uint32_t> labels,
                              GpuImage<uint32_t> union_markers_size_device) {
  const unsigned row = (blockIdx.y * blockDim.y + threadIdx.y) * 2;
  const unsigned col = (blockIdx.x * blockDim.x + threadIdx.x) * 2;
  const uint32_t foreground_labels_index = row * labels.step + col;
  const uint32_t background_labels_index = (row + 1) * labels.step + col;

  if (row >= labels.rows || col >= labels.cols) {
    return;
  }

  const uint64_t foreground_buffer =
      *reinterpret_cast<uint64_t *>(labels.data + foreground_labels_index);
  const uint32_t foreground_label = (foreground_buffer & (0xFFFFFFFF));
  const uint8_t foreground_info = (foreground_buffer >> 32) & 0xFF;
  const uint64_t background_buffer =
      *reinterpret_cast<uint64_t *>(labels.data + background_labels_index);
  const uint32_t background_left_label = (background_buffer & (0xFFFFFFFF));
  const uint32_t background_right_label =
      ((background_buffer >> 32) & (0xFFFFFFFF));
  const uint8_t background_left_info = (foreground_buffer >> 40) & 0xFF;
  const uint8_t background_right_info = (foreground_buffer >> 48) & 0xFF;

  uint32_t a_label;
  uint32_t b_label;
  uint32_t c_label;
  uint32_t d_label;

  if ((foreground_info & 0xf) == 0u && (background_left_info & 0xf) == 0u &&
      (background_right_info & 0xf) == 0u) {
    a_label = foreground_labels_index;
    b_label = foreground_labels_index + 1;
    c_label = background_labels_index;
    d_label = background_labels_index + 1;
    *reinterpret_cast<uint64_t *>(labels.data + foreground_labels_index) =
        (static_cast<uint64_t>(b_label) << 32) | a_label;
    *reinterpret_cast<uint64_t *>(labels.data + background_labels_index) =
        (static_cast<uint64_t>(d_label) << 32) | (c_label);
    return;
  } else {
    a_label = (HasBit(foreground_info, Info::a) * foreground_label +
               HasBit(background_left_info, Info::a) * background_left_label);
    b_label = HasBit(foreground_info, Info::b) * foreground_label +
              HasBit(background_right_info, Info::b) * background_right_label;
    c_label = HasBit(foreground_info, Info::c) * foreground_label +
              HasBit(background_left_info, Info::c) * background_left_label;
    d_label = HasBit(foreground_info, Info::d) * foreground_label +
              HasBit(background_right_info, Info::d) * background_right_label;

    *reinterpret_cast<uint64_t *>(labels.data + foreground_labels_index) =
        (static_cast<uint64_t>(b_label) << 32) | a_label;
    *reinterpret_cast<uint64_t *>(labels.data + background_labels_index) =
        (static_cast<uint64_t>(d_label) << 32) | (c_label);
  }

  if ((foreground_info & 0xf) != 0u) {
    // We've got foreground!
    uint32_t count = 0;
    if (HasBit(foreground_info, Info::a)) {
      ++count;
    }
    if (HasBit(foreground_info, Info::b)) {
      ++count;
    }
    if (HasBit(foreground_info, Info::c)) {
      ++count;
    }
    if (HasBit(foreground_info, Info::d)) {
      ++count;
    }

    atomicAdd(union_markers_size_device.data + foreground_label, count);
  }

  if ((background_left_info & 0xf) == 0u &&
      (background_right_info & 0xf) == 0u) {
    return;
  }

  if ((background_left_info & 0xf) != 0u &&
      (background_right_info & 0xf) != 0u &&
      background_left_label == background_right_label) {
    // They are all populated and match, go for it.
    uint32_t count = 0;
    if (HasBit(background_left_info, Info::a)) {
      ++count;
    }
    if (HasBit(background_right_info, Info::b)) {
      ++count;
    }
    if (HasBit(background_left_info, Info::c)) {
      ++count;
    }
    if (HasBit(background_right_info, Info::d)) {
      ++count;
    }

    atomicAdd(union_markers_size_device.data + background_left_label, count);
    return;
  }

  if ((background_left_info & 0xf) != 0u) {
    uint32_t count = 0;
    if (HasBit(background_left_info, Info::a)) {
      ++count;
    }
    if (HasBit(background_left_info, Info::c)) {
      ++count;
    }
    atomicAdd(union_markers_size_device.data + background_left_label, count);
  }

  if ((background_right_info & 0xf) != 0u) {
    uint32_t count = 0;
    if (HasBit(background_right_info, Info::b)) {
      ++count;
    }
    if (HasBit(background_right_info, Info::d)) {
      ++count;
    }
    atomicAdd(union_markers_size_device.data + background_right_label, count);
  }
}

}  // namespace

void LabelImage(const GpuImage<uint8_t> input, GpuImage<uint32_t> output,
                GpuImage<uint32_t> union_markers_size_device,
                cudaStream_t stream) {
  CHECK_NE(input.rows, 1u);
  CHECK_NE(input.cols, 1u);

  // Need an even number of rows and colums, we don't need to solve the actual
  // hard problems...
  CHECK_EQ(input.rows % 2, 0u);
  CHECK_EQ(input.cols % 2, 0u);

  dim3 grid_size =
      dim3((((input.cols + 1) / 2) + BLOCK_COLS - 1) / BLOCK_COLS,
           (((input.rows + 1) / 2) + BLOCK_ROWS - 1) / BLOCK_ROWS, 1);
  dim3 block_size = dim3(BLOCK_COLS, BLOCK_ROWS, 1);

  InitLabeling<<<grid_size, block_size, 0, stream>>>(input, output);

  Compression<<<grid_size, block_size, 0, stream>>>(output);

  Merge<<<grid_size, block_size, 0, stream>>>(output);

  Compression<<<grid_size, block_size, 0, stream>>>(output);

  FinalLabeling<<<grid_size, block_size, 0, stream>>>(
      output, union_markers_size_device);
}

#include "frc971/orin/threshold.h"

#include <stdint.h>

#include "frc971/orin/cuda.h"

namespace frc971::apriltag {
namespace {

// 1280 -> 2 * 128 * 5
// 720 -> 2 * 8 * 5 * 9
//
// 1456 -> 2 * 8 * 7 * 13
// 1088 -> 2 * 32 * 17

// Writes out the grayscale image and decimated image.
__global__ void InternalCudaToGreyscaleAndDecimateHalide(
    const uint8_t *color_image, uint8_t *gray_image, uint8_t *decimated_image,
    size_t width, size_t height) {
  size_t i = blockIdx.x * blockDim.x + threadIdx.x;
  while (i < width * height) {
//    uint8_t pixel = gray_image[i] = color_image[i * 2]; // Y CbCr input
    // BGR input
    const uint8_t pixel = gray_image[i] = 0.114 * color_image[i * 3] + 0.587 * color_image[i * 3 + 1] + 0.299 * color_image[i * 3 + 2];

    const size_t row = i / width;
    const size_t col = i - width * row;

    // Copy over every other pixel.
    if (row % 2 == 0 && col % 2 == 0) {
      size_t decimated_row = row / 2;
      size_t decimated_col = col / 2;
      decimated_image[decimated_row * width / 2 + decimated_col] = pixel;
    }
    i += blockDim.x * gridDim.x;
  }

  // TODO(austin): Figure out how to load contiguous memory reasonably
  // efficiently and max/min over it.

  // TODO(austin): Can we do the threshold here too?  That would be less memory
  // bandwidth consumed...
}

// Returns the min and max for a row of 4 pixels.
__forceinline__ __device__ uchar2 minmax(uchar4 row) {
  uint8_t min_val = std::min(std::min(row.x, row.y), std::min(row.z, row.w));
  uint8_t max_val = std::max(std::max(row.x, row.y), std::max(row.z, row.w));
  return make_uchar2(min_val, max_val);
}

// Returns the min and max for a set of min and maxes.
__forceinline__ __device__ uchar2 minmax(uchar2 val0, uchar2 val1) {
  return make_uchar2(std::min(val0.x, val1.x), std::max(val0.y, val1.y));
}

// Returns the pixel index of a pixel at the provided x and y location.
__forceinline__ __device__ size_t XYToIndex(size_t width, size_t x, size_t y) {
  return width * y + x;
}

// Computes the min and max pixel value for each block of 4 pixels.
__global__ void InternalBlockMinMax(const uint8_t *decimated_image,
                                    uchar2 *unfiltered_minmax_image,
                                    size_t width, size_t height) {
  uchar2 vals[4];
  const size_t x = blockIdx.x * blockDim.x + threadIdx.x;
  const size_t y = blockIdx.y * blockDim.y + threadIdx.y;

  if (x >= width || y >= height) {
    return;
  }

  for (int i = 0; i < 4; ++i) {
    const uchar4 decimated_block = *reinterpret_cast<const uchar4 *>(
        decimated_image + XYToIndex(width * 4, x * 4, y * 4 + i));

    vals[i] = minmax(decimated_block);
  }

  unfiltered_minmax_image[XYToIndex(width, x, y)] =
      minmax(minmax(vals[0], vals[1]), minmax(vals[2], vals[3]));
}

// Filters the min/max for the surrounding block of 9 pixels centered on our
// location using min/max and writes the result back out.
__global__ void InternalBlockFilter(const uchar2 *unfiltered_minmax_image,
                                    uchar2 *minmax_image, size_t width,
                                    size_t height) {
  uchar2 result = make_uchar2(255, 0);

  const size_t x = blockIdx.x * blockDim.x + threadIdx.x;
  const size_t y = blockIdx.y * blockDim.y + threadIdx.y;

  if (x >= width || y >= height) {
    return;
  }

  // Iterate through the 3x3 set of points centered on the point this image is
  // responsible for, and compute the overall min/max.
#pragma unroll
  for (int i = -1; i <= 1; ++i) {
#pragma unroll
    for (int j = -1; j <= 1; ++j) {
      const ssize_t read_x = x + i;
      const ssize_t read_y = y + j;

      if (read_x < 0 || read_x >= static_cast<ssize_t>(width)) {
        continue;
      }
      if (read_y < 0 || read_y >= static_cast<ssize_t>(height)) {
        continue;
      }

      result = minmax(
          result, unfiltered_minmax_image[XYToIndex(width, read_x, read_y)]);
    }
  }

  minmax_image[XYToIndex(width, x, y)] = result;
}

// Thresholds the image based on the filtered thresholds.
__global__ void InternalThreshold(const uint8_t *decimated_image,
                                  const uchar2 *minmax_image,
                                  uint8_t *thresholded_image, size_t width,
                                  size_t height, size_t min_white_black_diff) {
  size_t i = blockIdx.x * blockDim.x + threadIdx.x;
  while (i < width * height) {
    const size_t x = i % width;
    const size_t y = i / width;

    const uchar2 minmax_val = minmax_image[x / 4 + (y / 4) * width / 4];

    uint8_t result;
    if (minmax_val.y - minmax_val.x < min_white_black_diff) {
      result = 127;
    } else {
      uint8_t thresh = minmax_val.x + (minmax_val.y - minmax_val.x) / 2;
      if (decimated_image[i] > thresh) {
        result = 255;
      } else {
        result = 0;
      }
    }

    thresholded_image[i] = result;
    i += blockDim.x * gridDim.x;
  }
}

}  // namespace

void CudaToGreyscaleAndDecimateHalide(
    const uint8_t *color_image, uint8_t *gray_image, uint8_t *decimated_image,
    uint8_t *unfiltered_minmax_image, uint8_t *minmax_image,
    uint8_t *thresholded_image, size_t width, size_t height,
    size_t min_white_black_diff, CudaStream *stream) {
  CHECK((width % 8) == 0);
  CHECK((height % 8) == 0);
  constexpr size_t kThreads = 256;
  {
    // Step one, convert to gray and decimate.
    size_t kBlocks = (width * height + kThreads - 1) / kThreads / 4;
    InternalCudaToGreyscaleAndDecimateHalide<<<kBlocks, kThreads, 0,
                                               stream->get()>>>(
        color_image, gray_image, decimated_image, width, height);
    MaybeCheckAndSynchronize();
  }

  size_t decimated_width = width / 2;
  size_t decimated_height = height / 2;

  {
    // Step 2, compute a min/max for each block of 4x4 (16) pixels.
    dim3 threads(16, 16, 1);
    dim3 blocks((decimated_width / 4 + 15) / 16,
                (decimated_height / 4 + 15) / 16, 1);

    InternalBlockMinMax<<<blocks, threads, 0, stream->get()>>>(
        decimated_image, reinterpret_cast<uchar2 *>(unfiltered_minmax_image),
        decimated_width / 4, decimated_height / 4);
    MaybeCheckAndSynchronize();

    // Step 3, Blur those min/max's a further +- 1 block in each direction using
    // min/max.
    InternalBlockFilter<<<blocks, threads, 0, stream->get()>>>(
        reinterpret_cast<uchar2 *>(unfiltered_minmax_image),
        reinterpret_cast<uchar2 *>(minmax_image), decimated_width / 4,
        decimated_height / 4);
    MaybeCheckAndSynchronize();
  }

  {
    // Now, write out 127 if the min/max are too close to each other, or 0/255
    // if the pixels are above or below the average of the min/max.
    size_t kBlocks = (width * height / 4 + kThreads - 1) / kThreads / 4;
    InternalThreshold<<<kBlocks, kThreads, 0, stream->get()>>>(
        decimated_image, reinterpret_cast<uchar2 *>(minmax_image),
        thresholded_image, decimated_width, decimated_height,
        min_white_black_diff);
    MaybeCheckAndSynchronize();
  }
}

}  // namespace frc971::apriltag

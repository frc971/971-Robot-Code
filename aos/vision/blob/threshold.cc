#include "aos/vision/blob/threshold.h"

#include "aos/logging/logging.h"

namespace aos {
namespace vision {
namespace {

constexpr int kChunkSize = 8;

}  // namespace

// At a high level, the algorithm is the same as the slow thresholding, except
// it operates in kChunkSize-pixel chunks.
RangeImage FastYuyvYThreshold(ImageFormat fmt, const char *data,
                              uint8_t value) {
  CHECK_EQ(0, fmt.w % kChunkSize);
  std::vector<std::vector<ImageRange>> result;
  result.reserve(fmt.h);

  // Iterate through each row.
  for (int y = 0; y < fmt.h; ++y) {
    // The start of the data for the current row.
    const char *const current_row = fmt.w * y * 2 + data;
    bool in_range = false;
    int current_range_start = -1;
    std::vector<ImageRange> current_row_ranges;
    // Iterate through each kChunkSize-pixel chunk
    for (int x = 0; x < fmt.w / kChunkSize; ++x) {
      // The per-channel (YUYV) values in the current chunk.
      uint8_t chunk_channels[2 * kChunkSize];
      memcpy(&chunk_channels[0], current_row + x * kChunkSize * 2, 2 * kChunkSize);

      for (int i = 0; i < kChunkSize; ++i) {
        if ((chunk_channels[i * 2] > value) != in_range) {
          const int here = x * kChunkSize + i;
          if (in_range) {
            current_row_ranges.emplace_back(ImageRange(current_range_start, here));
          } else {
            current_range_start = here;
          }
          in_range = !in_range;
        }
      }
    }
    if (in_range) {
      current_row_ranges.emplace_back(ImageRange(current_range_start, fmt.w));
    }
    result.push_back(current_row_ranges);
  }
  return RangeImage(0, std::move(result));
}

}  // namespace vision
}  // namespace aos

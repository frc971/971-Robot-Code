#include "aos/vision/blob/threshold.h"

#include "aos/logging/logging.h"

namespace aos {
namespace vision {

// Expands to a unique value for each combination of values for 5 bools.
#define MASH(v0, v1, v2, v3, v4)                                  \
  ((uint8_t(v0) << 4) | (uint8_t(v1) << 3) | (uint8_t(v2) << 2) | \
   (uint8_t(v3) << 1) | (uint8_t(v4)))

// At a high level, the algorithm is the same as the slow thresholding, except
// it operates in 4-pixel chunks. The handling for each of these chunks is
// manually flattened (via codegen) into a 32-case switch statement. There are
// 2^4 cases for each pixel being in or out, along with another set of cases
// depending on whether the start of the chunk is in a range or not.
RangeImage FastYuyvYThreshold(ImageFormat fmt, const char *data,
                              uint8_t value) {
  CHECK_EQ(0, fmt.w % 4);
  std::vector<std::vector<ImageRange>> result;
  result.reserve(fmt.h);

  // Iterate through each row.
  for (int y = 0; y < fmt.h; ++y) {
    // The start of the data for the current row.
    const char *const current_row = fmt.w * y * 2 + data;
    bool in_range = false;
    int current_range_start = -1;
    std::vector<ImageRange> current_row_ranges;
    // Iterate through each 4-pixel chunk
    for (int x = 0; x < fmt.w / 4; ++x) {
      // The per-channel (YUYV) values in the current chunk.
      uint8_t chunk_channels[8];
      memcpy(&chunk_channels[0], current_row + x * 4 * 2, 8);
      const uint8_t pattern =
          MASH(in_range, chunk_channels[0] > value, chunk_channels[2] > value,
               chunk_channels[4] > value, chunk_channels[6] > value);
      switch (pattern) {
        // clang-format off
/*
# Ruby code to generate the below code:
32.times do |v|
        puts "case MASH(#{[v[4], v[3], v[2], v[1], v[0]].join(", ")}):"
        in_range = v[4]
        current_range_start = "current_range_start"
        4.times do |i|
                if v[3 - i] != in_range
                        if (in_range == 1)
                                puts "  current_row_ranges.emplace_back(ImageRange(#{current_range_start}, x * 4 + #{i}));"
                        else
                                current_range_start = "x * 4 + #{i}"
                        end
                        in_range = v[3 - i]
                end
        end
        if (current_range_start != "current_range_start")
                puts "  current_range_start = #{current_range_start};"
        end
        if (in_range != v[4])
                puts "  in_range = #{["false", "true"][v[0]]};"
        end
        puts "  break;"
end
*/
        // clang-format on
        case MASH(0, 0, 0, 0, 0):
          break;
        case MASH(0, 0, 0, 0, 1):
          current_range_start = x * 4 + 3;
          in_range = true;
          break;
        case MASH(0, 0, 0, 1, 0):
          current_row_ranges.emplace_back(ImageRange(x * 4 + 2, x * 4 + 3));
          current_range_start = x * 4 + 2;
          break;
        case MASH(0, 0, 0, 1, 1):
          current_range_start = x * 4 + 2;
          in_range = true;
          break;
        case MASH(0, 0, 1, 0, 0):
          current_row_ranges.emplace_back(ImageRange(x * 4 + 1, x * 4 + 2));
          current_range_start = x * 4 + 1;
          break;
        case MASH(0, 0, 1, 0, 1):
          current_row_ranges.emplace_back(ImageRange(x * 4 + 1, x * 4 + 2));
          current_range_start = x * 4 + 3;
          in_range = true;
          break;
        case MASH(0, 0, 1, 1, 0):
          current_row_ranges.emplace_back(ImageRange(x * 4 + 1, x * 4 + 3));
          current_range_start = x * 4 + 1;
          break;
        case MASH(0, 0, 1, 1, 1):
          current_range_start = x * 4 + 1;
          in_range = true;
          break;
        case MASH(0, 1, 0, 0, 0):
          current_row_ranges.emplace_back(ImageRange(x * 4 + 0, x * 4 + 1));
          current_range_start = x * 4 + 0;
          break;
        case MASH(0, 1, 0, 0, 1):
          current_row_ranges.emplace_back(ImageRange(x * 4 + 0, x * 4 + 1));
          current_range_start = x * 4 + 3;
          in_range = true;
          break;
        case MASH(0, 1, 0, 1, 0):
          current_row_ranges.emplace_back(ImageRange(x * 4 + 0, x * 4 + 1));
          current_row_ranges.emplace_back(ImageRange(x * 4 + 2, x * 4 + 3));
          current_range_start = x * 4 + 2;
          break;
        case MASH(0, 1, 0, 1, 1):
          current_row_ranges.emplace_back(ImageRange(x * 4 + 0, x * 4 + 1));
          current_range_start = x * 4 + 2;
          in_range = true;
          break;
        case MASH(0, 1, 1, 0, 0):
          current_row_ranges.emplace_back(ImageRange(x * 4 + 0, x * 4 + 2));
          current_range_start = x * 4 + 0;
          break;
        case MASH(0, 1, 1, 0, 1):
          current_row_ranges.emplace_back(ImageRange(x * 4 + 0, x * 4 + 2));
          current_range_start = x * 4 + 3;
          in_range = true;
          break;
        case MASH(0, 1, 1, 1, 0):
          current_row_ranges.emplace_back(ImageRange(x * 4 + 0, x * 4 + 3));
          current_range_start = x * 4 + 0;
          break;
        case MASH(0, 1, 1, 1, 1):
          current_range_start = x * 4 + 0;
          in_range = true;
          break;
        case MASH(1, 0, 0, 0, 0):
          current_row_ranges.emplace_back(
              ImageRange(current_range_start, x * 4 + 0));
          in_range = false;
          break;
        case MASH(1, 0, 0, 0, 1):
          current_row_ranges.emplace_back(
              ImageRange(current_range_start, x * 4 + 0));
          current_range_start = x * 4 + 3;
          break;
        case MASH(1, 0, 0, 1, 0):
          current_row_ranges.emplace_back(
              ImageRange(current_range_start, x * 4 + 0));
          current_row_ranges.emplace_back(ImageRange(x * 4 + 2, x * 4 + 3));
          current_range_start = x * 4 + 2;
          in_range = false;
          break;
        case MASH(1, 0, 0, 1, 1):
          current_row_ranges.emplace_back(
              ImageRange(current_range_start, x * 4 + 0));
          current_range_start = x * 4 + 2;
          break;
        case MASH(1, 0, 1, 0, 0):
          current_row_ranges.emplace_back(
              ImageRange(current_range_start, x * 4 + 0));
          current_row_ranges.emplace_back(ImageRange(x * 4 + 1, x * 4 + 2));
          current_range_start = x * 4 + 1;
          in_range = false;
          break;
        case MASH(1, 0, 1, 0, 1):
          current_row_ranges.emplace_back(
              ImageRange(current_range_start, x * 4 + 0));
          current_row_ranges.emplace_back(ImageRange(x * 4 + 1, x * 4 + 2));
          current_range_start = x * 4 + 3;
          break;
        case MASH(1, 0, 1, 1, 0):
          current_row_ranges.emplace_back(
              ImageRange(current_range_start, x * 4 + 0));
          current_row_ranges.emplace_back(ImageRange(x * 4 + 1, x * 4 + 3));
          current_range_start = x * 4 + 1;
          in_range = false;
          break;
        case MASH(1, 0, 1, 1, 1):
          current_row_ranges.emplace_back(
              ImageRange(current_range_start, x * 4 + 0));
          current_range_start = x * 4 + 1;
          break;
        case MASH(1, 1, 0, 0, 0):
          current_row_ranges.emplace_back(
              ImageRange(current_range_start, x * 4 + 1));
          in_range = false;
          break;
        case MASH(1, 1, 0, 0, 1):
          current_row_ranges.emplace_back(
              ImageRange(current_range_start, x * 4 + 1));
          current_range_start = x * 4 + 3;
          break;
        case MASH(1, 1, 0, 1, 0):
          current_row_ranges.emplace_back(
              ImageRange(current_range_start, x * 4 + 1));
          current_row_ranges.emplace_back(ImageRange(x * 4 + 2, x * 4 + 3));
          current_range_start = x * 4 + 2;
          in_range = false;
          break;
        case MASH(1, 1, 0, 1, 1):
          current_row_ranges.emplace_back(
              ImageRange(current_range_start, x * 4 + 1));
          current_range_start = x * 4 + 2;
          break;
        case MASH(1, 1, 1, 0, 0):
          current_row_ranges.emplace_back(
              ImageRange(current_range_start, x * 4 + 2));
          in_range = false;
          break;
        case MASH(1, 1, 1, 0, 1):
          current_row_ranges.emplace_back(
              ImageRange(current_range_start, x * 4 + 2));
          current_range_start = x * 4 + 3;
          break;
        case MASH(1, 1, 1, 1, 0):
          current_row_ranges.emplace_back(
              ImageRange(current_range_start, x * 4 + 3));
          in_range = false;
          break;
        case MASH(1, 1, 1, 1, 1):
          break;
      }
    }
    if (in_range) {
      current_row_ranges.emplace_back(ImageRange(current_range_start, fmt.w));
    }
    result.push_back(current_row_ranges);
  }
  return RangeImage(0, std::move(result));
}

#undef MASH

}  // namespace vision
}  // namespace aos

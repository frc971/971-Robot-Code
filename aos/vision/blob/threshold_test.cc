#include "aos/vision/blob/threshold.h"

#include <vector>

#include "aos/vision/blob/range_image.h"
#include "aos/vision/image/image_types.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace aos {
namespace vision {
namespace testing {

class YuyvYThresholdTest : public ::testing::Test {
};

// Verifies that a simple image is thresholded correctly.
//
// Specifically, we want to get this result from the thresholding:
//  --+--
//  +---+
//  -+++-
//  +++-+
//  -----
//  ++-++
//  +++++
//  +-+-+
TEST_F(YuyvYThresholdTest, SimpleImage) {
  ImageFormat format;
  format.w = 5;
  format.h = 8;

  std::vector<std::vector<ImageRange>> expected_ranges;
  std::vector<char> image;
  image.resize(5 * 8 * 2);
  //  --+--
  image[0 * 2 + 0 * 10] = 0;
  image[1 * 2 + 0 * 10] = 0;
  image[2 * 2 + 0 * 10] = 128;
  image[3 * 2 + 0 * 10] = 127;
  image[4 * 2 + 0 * 10] = 0;
  expected_ranges.push_back({{{2, 3}}});
  //  +---+
  image[0 * 2 + 1 * 10] = 128;
  image[1 * 2 + 1 * 10] = 0;
  image[2 * 2 + 1 * 10] = 0;
  image[3 * 2 + 1 * 10] = 10;
  image[4 * 2 + 1 * 10] = 255;
  expected_ranges.push_back({{{0, 1}, {4, 5}}});
  //  -+++-
  image[0 * 2 + 2 * 10] = 73;
  image[1 * 2 + 2 * 10] = 250;
  image[2 * 2 + 2 * 10] = 251;
  image[3 * 2 + 2 * 10] = 252;
  image[4 * 2 + 2 * 10] = 45;
  expected_ranges.push_back({{{1, 4}}});
  //  +++-+
  image[0 * 2 + 3 * 10] = 128;
  image[1 * 2 + 3 * 10] = 134;
  image[2 * 2 + 3 * 10] = 250;
  image[3 * 2 + 3 * 10] = 0;
  image[4 * 2 + 3 * 10] = 230;
  expected_ranges.push_back({{{0, 3}, {4, 5}}});
  //  -----
  image[0 * 2 + 4 * 10] = 7;
  image[1 * 2 + 4 * 10] = 120;
  image[2 * 2 + 4 * 10] = 127;
  image[3 * 2 + 4 * 10] = 0;
  image[4 * 2 + 4 * 10] = 50;
  expected_ranges.push_back({{}});
  //  ++-++
  image[0 * 2 + 5 * 10] = 140;
  image[1 * 2 + 5 * 10] = 140;
  image[2 * 2 + 5 * 10] = 0;
  image[3 * 2 + 5 * 10] = 140;
  image[4 * 2 + 5 * 10] = 140;
  expected_ranges.push_back({{{0, 2}, {3, 5}}});
  //  +++++
  image[0 * 2 + 6 * 10] = 128;
  image[1 * 2 + 6 * 10] = 128;
  image[2 * 2 + 6 * 10] = 128;
  image[3 * 2 + 6 * 10] = 128;
  image[4 * 2 + 6 * 10] = 128;
  expected_ranges.push_back({{{0, 5}}});
  //  +-+-+
  image[0 * 2 + 7 * 10] = 200;
  image[1 * 2 + 7 * 10] = 0;
  image[2 * 2 + 7 * 10] = 200;
  image[3 * 2 + 7 * 10] = 0;
  image[4 * 2 + 7 * 10] = 200;
  expected_ranges.push_back({{{0, 1}, {2, 3}, {4, 5}}});
  const RangeImage expected_result(0, std::move(expected_ranges));

  const auto slow_result = SlowYuyvYThreshold(format, image.data(), 127);
  ASSERT_EQ(expected_result, slow_result);
}

}  // namespace testing
}  // namespace vision
}  // namespace aos

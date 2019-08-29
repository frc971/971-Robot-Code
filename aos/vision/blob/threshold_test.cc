#include "aos/vision/blob/threshold.h"

#include <random>
#include <vector>

#include "aos/vision/blob/range_image.h"
#include "aos/vision/image/image_types.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace aos {
namespace vision {
namespace testing {

class YuyvYThresholdTest : public ::testing::Test {
 public:
  std::vector<char> RandomImage(ImageFormat format) {
    std::vector<char> result;
    result.resize(format.w * format.h * 2);
    std::uniform_int_distribution<char> distribution(
        std::numeric_limits<char>::min(), std::numeric_limits<char>::max());
    for (size_t i = 0; i < result.size(); ++i) {
      result[i] = distribution(generator_);
    }
    return result;
  }

 private:
  std::minstd_rand generator_;
};

// Verifies that a simple image is thresholded correctly.
//
// Specifically, we want to get this result from the thresholding:
//  --+-----
//  +------+
//  -++++++-
//  +++-++++
//  --------
//  ++++-+++
//  ++++++++
//  +-+-+--+
TEST_F(YuyvYThresholdTest, SimpleImage) {
  ImageFormat format;
  format.w = 8;
  format.h = 8;

  std::vector<std::vector<ImageRange>> expected_ranges;
  std::vector<char> image;
  image.resize(8 * 8 * 2);
  //  --+-----
  image[0 * 2 + 0 * 16] = static_cast<char>(0);
  image[1 * 2 + 0 * 16] = static_cast<char>(0);
  image[2 * 2 + 0 * 16] = static_cast<char>(128);
  image[3 * 2 + 0 * 16] = static_cast<char>(127);
  image[4 * 2 + 0 * 16] = static_cast<char>(0);
  image[5 * 2 + 0 * 16] = static_cast<char>(0);
  image[6 * 2 + 0 * 16] = static_cast<char>(0);
  image[7 * 2 + 0 * 16] = static_cast<char>(0);
  expected_ranges.push_back({{{2, 3}}});
  //  +------+
  image[0 * 2 + 1 * 16] = static_cast<char>(128);
  image[1 * 2 + 1 * 16] = static_cast<char>(0);
  image[2 * 2 + 1 * 16] = static_cast<char>(0);
  image[3 * 2 + 1 * 16] = static_cast<char>(10);
  image[4 * 2 + 1 * 16] = static_cast<char>(30);
  image[5 * 2 + 1 * 16] = static_cast<char>(50);
  image[6 * 2 + 1 * 16] = static_cast<char>(70);
  image[7 * 2 + 1 * 16] = static_cast<char>(255);
  expected_ranges.push_back({{{0, 1}, {7, 8}}});
  //  -++++++-
  image[0 * 2 + 2 * 16] = static_cast<char>(73);
  image[1 * 2 + 2 * 16] = static_cast<char>(246);
  image[2 * 2 + 2 * 16] = static_cast<char>(247);
  image[3 * 2 + 2 * 16] = static_cast<char>(248);
  image[4 * 2 + 2 * 16] = static_cast<char>(249);
  image[5 * 2 + 2 * 16] = static_cast<char>(250);
  image[6 * 2 + 2 * 16] = static_cast<char>(250);
  image[7 * 2 + 2 * 16] = static_cast<char>(45);
  expected_ranges.push_back({{{1, 7}}});
  //  +++-++++
  image[0 * 2 + 3 * 16] = static_cast<char>(128);
  image[1 * 2 + 3 * 16] = static_cast<char>(134);
  image[2 * 2 + 3 * 16] = static_cast<char>(250);
  image[3 * 2 + 3 * 16] = static_cast<char>(0);
  image[4 * 2 + 3 * 16] = static_cast<char>(230);
  image[5 * 2 + 3 * 16] = static_cast<char>(230);
  image[6 * 2 + 3 * 16] = static_cast<char>(230);
  image[7 * 2 + 3 * 16] = static_cast<char>(210);
  expected_ranges.push_back({{{0, 3}, {4, 8}}});
  //  --------
  image[0 * 2 + 4 * 16] = static_cast<char>(7);
  image[1 * 2 + 4 * 16] = static_cast<char>(120);
  image[2 * 2 + 4 * 16] = static_cast<char>(127);
  image[3 * 2 + 4 * 16] = static_cast<char>(0);
  image[4 * 2 + 4 * 16] = static_cast<char>(50);
  image[5 * 2 + 4 * 16] = static_cast<char>(80);
  image[6 * 2 + 4 * 16] = static_cast<char>(110);
  image[7 * 2 + 4 * 16] = static_cast<char>(25);
  expected_ranges.push_back({{}});
  //  ++++-+++
  image[0 * 2 + 5 * 16] = static_cast<char>(140);
  image[1 * 2 + 5 * 16] = static_cast<char>(140);
  image[2 * 2 + 5 * 16] = static_cast<char>(140);
  image[3 * 2 + 5 * 16] = static_cast<char>(140);
  image[4 * 2 + 5 * 16] = static_cast<char>(0);
  image[5 * 2 + 5 * 16] = static_cast<char>(140);
  image[6 * 2 + 5 * 16] = static_cast<char>(140);
  image[7 * 2 + 5 * 16] = static_cast<char>(140);
  expected_ranges.push_back({{{0, 4}, {5, 8}}});
  //  ++++++++
  image[0 * 2 + 6 * 16] = static_cast<char>(128);
  image[1 * 2 + 6 * 16] = static_cast<char>(128);
  image[2 * 2 + 6 * 16] = static_cast<char>(128);
  image[3 * 2 + 6 * 16] = static_cast<char>(128);
  image[4 * 2 + 6 * 16] = static_cast<char>(128);
  image[5 * 2 + 6 * 16] = static_cast<char>(128);
  image[6 * 2 + 6 * 16] = static_cast<char>(128);
  image[7 * 2 + 6 * 16] = static_cast<char>(128);
  expected_ranges.push_back({{{0, 8}}});
  //  +-+-+--+
  image[0 * 2 + 7 * 16] = static_cast<char>(200);
  image[1 * 2 + 7 * 16] = static_cast<char>(0);
  image[2 * 2 + 7 * 16] = static_cast<char>(200);
  image[3 * 2 + 7 * 16] = static_cast<char>(0);
  image[4 * 2 + 7 * 16] = static_cast<char>(200);
  image[5 * 2 + 7 * 16] = static_cast<char>(0);
  image[6 * 2 + 7 * 16] = static_cast<char>(0);
  image[7 * 2 + 7 * 16] = static_cast<char>(200);
  expected_ranges.push_back({{{0, 1}, {2, 3}, {4, 5}, {7, 8}}});
  const RangeImage expected_result(0, std::move(expected_ranges));

  const auto slow_result = SlowYuyvYThreshold(format, image.data(), 127);
  ASSERT_EQ(expected_result, slow_result);
  const auto fast_result = FastYuyvYThreshold(format, image.data(), 127);
  ASSERT_EQ(expected_result, fast_result);
}

// Verifies that a couple of completely random images match.
TEST_F(YuyvYThresholdTest, Random) {
  for (int i = 0; i < 10; ++i) {
    ImageFormat small_format;
    small_format.w = 16;
    small_format.h = 16;
    const auto small_image = RandomImage(small_format);
    const auto slow_result =
        SlowYuyvYThreshold(small_format, small_image.data(), 127);
    const auto fast_result =
        FastYuyvYThreshold(small_format, small_image.data(), 127);
    ASSERT_EQ(slow_result, fast_result);
  }
  for (int i = 0; i < 10; ++i) {
    ImageFormat large_format;
    large_format.w = 1024;
    large_format.h = 512;
    const auto large_image = RandomImage(large_format);
    const auto slow_result =
        SlowYuyvYThreshold(large_format, large_image.data(), 127);
    const auto fast_result =
        FastYuyvYThreshold(large_format, large_image.data(), 127);
    ASSERT_EQ(slow_result, fast_result);
  }
}

// Verifies that changing the U and V values doesn't affect the result.
TEST_F(YuyvYThresholdTest, UVIgnored) {
  ImageFormat format;
  format.w = 32;
  format.h = 20;
  const auto baseline_image = RandomImage(format);
  const auto baseline_result =
      SlowYuyvYThreshold(format, baseline_image.data(), 127);
  for (int i = 0; i < 5; ++i) {
    auto tweaked_image = RandomImage(format);
    for (int y = 0; y < format.h; ++y) {
      for (int x = 0; x < format.w; ++x) {
        tweaked_image[x * 2 + y * format.w * 2] =
            baseline_image[x * 2 + y * format.w * 2];
      }
    }

    const auto slow_result =
        SlowYuyvYThreshold(format, tweaked_image.data(), 127);
    ASSERT_EQ(baseline_result, slow_result);
    const auto fast_result =
        FastYuyvYThreshold(format, tweaked_image.data(), 127);
    ASSERT_EQ(baseline_result, fast_result);
  }
}

}  // namespace testing
}  // namespace vision
}  // namespace aos

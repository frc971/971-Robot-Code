#include "y2020/vision/sift/fast_gaussian.h"

#include "gtest/gtest.h"
#include <opencv2/imgproc.hpp>

#include "y2020/vision/sift/fast_gaussian_all.h"

namespace frc971 {
namespace vision {
namespace testing {

class FastGaussianTest : public ::testing::Test {
 public:
  cv::Mat RandomImage(int width = 500, int height = 500, int type = CV_8UC3) {
    cv::Mat result(width, height, type);
    cv::randu(result, 0, 255);
    return result;
  }

  void ExpectEqual(const cv::Mat &a, const cv::Mat &b, double epsilon = 1e-10) {
    const cv::Mat difference = a - b;
    double min, max;
    cv::minMaxLoc(difference, &min, &max);
    EXPECT_GE(min, -epsilon);
    EXPECT_LE(max, epsilon);
  }
};

// Verifies that the default GaussianBlur parameters work out to 15x15 with
// sigma of 1.6.
TEST_F(FastGaussianTest, DefaultGaussianSize) {
  const auto image = RandomImage(500, 500, CV_32FC3);
  cv::Mat default_blurred, explicitly_blurred;
  cv::GaussianBlur(image, default_blurred, cv::Size(), 1.6, 1.6);
  cv::GaussianBlur(image, explicitly_blurred, cv::Size(15, 15), 1.6, 1.6);
  ExpectEqual(default_blurred, explicitly_blurred);
}

// Verifies that with 8U just a 9x9 blur is as much as you get, except for a bit
// of rounding.
TEST_F(FastGaussianTest, GaussianSize8U) {
  const auto image = RandomImage(500, 500, CV_8UC3);
  cv::Mat big_blurred, little_blurred;
  cv::GaussianBlur(image, big_blurred, cv::Size(15, 15), 1.6, 1.6);
  cv::GaussianBlur(image, little_blurred, cv::Size(9, 9), 1.6, 1.6);
  ExpectEqual(big_blurred, little_blurred, 3);
}

// Verifies that FastGaussian and cv::GaussianBlur give the same result.
TEST_F(FastGaussianTest, FastGaussian) {
  const auto image = RandomImage(480, 640, CV_16SC1);
  cv::Mat slow, fast, fast_direct;
  static constexpr double kSigma = 1.9465878414647133;
  static constexpr int kSize = 13;

  cv::GaussianBlur(image, slow, cv::Size(kSize, kSize), kSigma, kSigma);
  FastGaussian(image, &fast, kSigma);

  // Explicitly call the generated code, to verify that our chosen parameters do
  // in fact result in using the generated one.
  fast_direct.create(slow.size(), slow.type());
  ASSERT_EQ(0,
            DoGeneratedFastGaussian(MatToHalide<const int16_t>(image),
                                    MatToHalide<int16_t>(fast_direct), kSigma));

  // 1500/65536 = 0.0228, which is under 3%, which is pretty close.
  ExpectEqual(slow, fast, 1500);
  // The wrapper should be calling the exact same code, so it should end up with
  // the exact same result.
  ExpectEqual(fast, fast_direct, 0);
}

}  // namespace testing
}  // namespace vision
}  // namespace frc971

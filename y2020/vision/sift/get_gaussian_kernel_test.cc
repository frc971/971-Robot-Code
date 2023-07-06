#include "y2020/vision/sift/get_gaussian_kernel.h"

#include <tuple>

#include "glog/logging.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>

namespace frc971 {
namespace vision {
namespace testing {

class GetGaussianKernelTest
    : public ::testing::TestWithParam<std::tuple<int, double>> {
 public:
  int ksize() const { return std::get<0>(GetParam()); }
  double sigma() const { return std::get<1>(GetParam()); }
};

TEST_P(GetGaussianKernelTest, EqualsOpencv) {
  const cv::Mat opencv = cv::getGaussianKernel(ksize(), sigma(), CV_32F);
  CHECK(opencv.isContinuous());
  std::vector<float> opencv_vector(opencv.total());
  memcpy(opencv_vector.data(), opencv.data, opencv.total() * sizeof(float));
  EXPECT_THAT(GetGaussianKernel(ksize(), sigma()),
              ::testing::Pointwise(::testing::FloatEq(), opencv_vector));
}

INSTANTIATE_TEST_SUITE_P(Values, GetGaussianKernelTest,
                         ::testing::Combine(::testing::Values(1, 3, 7, 13, 21),
                                            ::testing::Values(0.01f, 0.1f,
                                                              0.9f)));

}  // namespace testing
}  // namespace vision
}  // namespace frc971

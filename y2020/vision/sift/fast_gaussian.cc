#include "y2020/vision/sift/fast_gaussian.h"

#include <iomanip>

#include <opencv2/imgproc.hpp>

#include "y2020/vision/sift/fast_gaussian_all.h"

namespace frc971 {
namespace vision {
namespace {

void CheckNonOverlapping(const cv::Mat &a, const cv::Mat &b) {
  CHECK(a.data > b.data + b.total() * b.elemSize() || a.data < b.data)
      << ": images may not overlap";
  CHECK(b.data > a.data + a.total() * a.elemSize() || b.data < a.data)
      << ": images may not overlap";
}

// An easy toggle to always fall back to the slow implementations, to verify the
// results are the same.
constexpr bool kUseFast = true;

// An easy toggle to print the result of all operations, for verifying that the
// halide code is doing what we expect.
constexpr bool kPrintAll = false;

// We deliberately don't generate code for images smaller than this, so don't
// print warnings about them.
//
// The opencv implementations are so fast below this size, the build time to
// generate halide versions isn't worthwhile.
constexpr int kMinWarnSize = 80;

bool IsSmall(cv::Size size) {
  return size.height <= kMinWarnSize && size.width <= kMinWarnSize;
}

}  // namespace

void FastGaussian(const cv::Mat &source, cv::Mat *destination, double sigma) {
  CHECK_EQ(source.type(), CV_16SC1);

  destination->create(source.size(), source.type());
  CheckNonOverlapping(source, *destination);

  int result = 1;
  if (kUseFast) {
    result = DoGeneratedFastGaussian(MatToHalide<const int16_t>(source),
                                     MatToHalide<int16_t>(*destination), sigma);
  }
  if (kPrintAll) {
    LOG(INFO) << result << ": " << source.rows << " " << source.cols << " "
              << std::setprecision(17) << sigma;
  }
  if (result == 0) {
    return;
  }
  if (!IsSmall(source.size())) {
    LOG(WARNING) << "slow gaussian blur: " << source.rows << " " << source.cols
                 << " " << std::setprecision(17) << sigma;
  }
  CHECK_EQ(result, 1);

  cv::GaussianBlur(source, *destination, cv::Size(), sigma, sigma,
                   cv::BORDER_REPLICATE);
}

void FastSubtract(const cv::Mat &a, const cv::Mat &b, cv::Mat *destination) {
  CHECK(a.size() == b.size());
  destination->create(a.size(), a.type());
  CheckNonOverlapping(a, *destination);
  CheckNonOverlapping(b, *destination);

  int result = 1;
  if (kUseFast) {
    result = DoGeneratedFastSubtract(MatToHalide<const int16_t>(a),
                                     MatToHalide<const int16_t>(b),
                                     MatToHalide<int16_t>(*destination));
  }
  if (kPrintAll) {
    LOG(INFO) << result << ": " << a.rows << " " << a.cols;
  }
  if (result == 0) {
    return;
  }
  if (!IsSmall(a.size())) {
    LOG(WARNING) << "slow subtract: " << a.rows << " " << a.cols;
  }
  CHECK_EQ(result, 1);

  cv::subtract(a, b, *destination);
}

void FastGaussianAndSubtract(const cv::Mat &source, cv::Mat *blurred,
                             cv::Mat *difference, double sigma) {
  CHECK_EQ(source.type(), CV_16SC1);
  blurred->create(source.size(), source.type());
  difference->create(source.size(), source.type());

  int result = 1;
  if (kUseFast) {
    result = DoGeneratedFastGaussianAndSubtract(
        MatToHalide<const int16_t>(source), MatToHalide<int16_t>(*blurred),
        MatToHalide<int16_t>(*difference), sigma);
  }
  if (kPrintAll) {
    LOG(INFO) << result << ": " << source.rows << " " << source.cols << " "
              << std::setprecision(17) << sigma;
  }
  if (result == 0) {
    return;
  }
  if (!IsSmall(source.size())) {
    LOG(WARNING) << "slow gaussian blur: " << source.rows << " " << source.cols
                 << " " << std::setprecision(17) << sigma;
  }
  CHECK_EQ(result, 1);

  cv::GaussianBlur(source, *blurred, cv::Size(), sigma, sigma,
                   cv::BORDER_REPLICATE);
  cv::subtract(*blurred, source, *difference);
}

}  // namespace vision
}  // namespace frc971

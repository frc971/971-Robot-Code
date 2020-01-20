#ifndef Y2020_VISION_SIFT_FAST_GAUSSIAN_H_
#define Y2020_VISION_SIFT_FAST_GAUSSIAN_H_

#include <type_traits>

#include <opencv2/core/mat.hpp>
#include "HalideBuffer.h"
#include "glog/logging.h"

namespace frc971 {
namespace vision {

// Returns a Halide buffer representing the data in mat.
template <typename T>
inline Halide::Runtime::Buffer<T, 2> MatToHalide(const cv::Mat &mat) {
  CHECK_EQ(cv::DataType<typename std::remove_const<T>::type>::type, mat.type());
  // Verify that at<T>(row, col) accesses this address:
  //   data + sizeof(T) * (row * cols + col)
  CHECK_EQ(mat.elemSize(), sizeof(T));
  CHECK_EQ(mat.elemSize1(), sizeof(T));
  CHECK_EQ(mat.step1(0), static_cast<size_t>(mat.cols));
  CHECK_EQ(mat.step1(1), 1u);
  CHECK_EQ(mat.dims, 2);
  CHECK(mat.isContinuous());
  return Halide::Runtime::Buffer<T, 2>(reinterpret_cast<T *>(mat.data),
                                       mat.cols, mat.rows);
}

// Performs a gaussian blur with the specified sigma, truncated to a reasonable
// width. Attempts to use faster implementations, but will fall back to
// cv::GaussianBlur otherwise. Only handles a limited set of Mat formats.
//
// source and destination may not overlap.
//
// Always uses BORDER_REPLICATE mode.
void FastGaussian(const cv::Mat &source, cv::Mat *destination, double sigma);
void FastSubtract(const cv::Mat &a, const cv::Mat &b, cv::Mat *destination);
void FastGaussianAndSubtract(const cv::Mat &source, cv::Mat *blurred,
                             cv::Mat *difference, double sigma);

}  // namespace vision
}  // namespace vision

#endif  // Y2020_VISION_SIFT_FAST_GAUSSIAN_H_

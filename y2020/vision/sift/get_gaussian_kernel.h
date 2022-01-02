#ifndef Y2020_VISION_SIFT_GET_GAUSSIAN_KERNEL_H_
#define Y2020_VISION_SIFT_GET_GAUSSIAN_KERNEL_H_

#include <cmath>
#include <vector>

namespace frc971 {
namespace vision {

// A reimplementation of cv::getGaussianKernel for CV_32F without external
// dependencies. See fast_gaussian_halide_generator.sh for details why we want
// this.
inline std::vector<float> GetGaussianKernel(int ksize, double sigma) {
  std::vector<float> result;
  float total = 0;
  for (int i = 0; i < ksize; ++i) {
    const float x = i - (ksize - 1) / 2;
    result.push_back(std::exp(-x * x / (2 * sigma * sigma)));
    total += result.back();
  }
  for (float &r : result) {
    r /= total;
  }
  return result;
}

}  // namespace vision
}  // namespace frc971

#endif  // Y2020_VISION_SIFT_GET_GAUSSIAN_KERNEL_H_

#include <memory>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "aos/init.h"
#include "aos/time/time.h"
#include "y2020/vision/sift/fast_gaussian.h"
#include "glog/logging.h"
#include "y2020/vision/sift/sift971.h"

DEFINE_string(image, "", "Image to test with");

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);

  cv::setNumThreads	(4);

  const cv::Mat raw_image = cv::imread(FLAGS_image);
  CHECK(!raw_image.empty()) << ": Failed to read: " << FLAGS_image;
  CHECK_EQ(CV_8UC3, raw_image.type());
#if 0
  cv::Mat color_image;
  raw_image.convertTo(color_image, CV_32F, 1.0/255.0);
  cv::Mat image;
  cv::cvtColor(color_image, image, cv::COLOR_BGR2GRAY);
#else
  cv::Mat gray_image;
  cv::cvtColor(raw_image, gray_image, cv::COLOR_BGR2GRAY);
  cv::Mat float_image;
#if 0
  gray_image.convertTo(float_image, CV_32F, 0.00390625);
#else
  float_image = gray_image;
#endif
  cv::Mat image;
  cv::resize(float_image, image, cv::Size(1280, 720), 0, 0, cv::INTER_AREA);
#endif
#if 0
#if 0
  cv::namedWindow("source", cv::WINDOW_AUTOSIZE);
  cv::imshow("source", raw_image);
  cv::namedWindow("converted", cv::WINDOW_AUTOSIZE);
  cv::imshow("converted", image);
#endif

  cv::Mat slow_blurred, fast_blurred;
  const double sigma = 3.0900155872895909;
  cv::GaussianBlur(image, slow_blurred, cv::Size(9, 9), sigma, sigma);
  frc971::vision::FastGaussian(image, &fast_blurred, sigma);
  cv::namedWindow("slow", cv::WINDOW_AUTOSIZE);
  cv::imshow("slow", slow_blurred);
  cv::namedWindow("fast", cv::WINDOW_AUTOSIZE);
  cv::imshow("fast", fast_blurred);
  cv::waitKey(0);
  return 0;
#endif

  LOG(INFO);
  std::unique_ptr<frc971::vision::SIFT971_Impl> sift(new frc971::vision::SIFT971_Impl());
  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptors;
  LOG(INFO) << "detectAndCompute on " << image.rows << "x" << image.cols;
  sift->detectAndCompute(image, cv::noArray(), keypoints, descriptors);
  LOG(INFO);

#if 0
  return 0;
#endif

  static constexpr int kIterations = 40;
  const auto start = aos::monotonic_clock::now();
  for (int i = 0; i < kIterations; ++i) {
    keypoints.clear();
    descriptors.release();
    sift->detectAndCompute(image, cv::noArray(), keypoints, descriptors);
  }
  const auto end = aos::monotonic_clock::now();
  LOG(INFO)
      << "Took: "
      << (std::chrono::duration<double>(end - start) / kIterations).count();
  // Should be ~352 for FRC-Image4-cleaned.png downscaled to 640x360.
  // 376 in DoG_TYPE_SHORT mode.
  // 344 now with 1280x720 non-upscaled.
  LOG(INFO) << "found " << keypoints.size() << " and " << descriptors.size();
}

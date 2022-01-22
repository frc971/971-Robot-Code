#ifndef Y2022_BLOB_DETECTOR_H_
#define Y2022_BLOB_DETECTOR_H_

#include <opencv2/imgproc.hpp>

namespace y2022 {
namespace vision {

class BlobDetector {
 public:
  BlobDetector() {}
  // Given an image, threshold it to find "green" pixels
  // Input: Color image
  // Output: Grayscale (binarized) image with green pixels set to 255
  static cv::Mat ThresholdImage(cv::Mat rgb_image);

  // Given binary image, extract blobs
  static std::vector<std::vector<cv::Point>> FindBlobs(cv::Mat threshold_image);

  // Filter blobs to get rid of noise, too large items, etc.
  static std::vector<std::vector<cv::Point>> FilterBlobs(
      std::vector<std::vector<cv::Point>> blobs);

  // Draw Blobs on image
  // Optionally draw all blobs and filtered blobs
  static void DrawBlobs(cv::Mat view_image,
                        std::vector<std::vector<cv::Point>> filtered_blobs,
                        std::vector<std::vector<cv::Point>> unfiltered_blobs);

  // Extract stats for each blob
  static std::vector<std::vector<cv::Point>> ComputeStats(
      std::vector<std::vector<cv::Point>>);

  static void ExtractBlobs(
      cv::Mat rgb_image, cv::Mat binarized_image, cv::Mat blob_image,
      std::vector<std::vector<cv::Point>> &filtered_blobs,
      std::vector<std::vector<cv::Point>> &unfiltered_blobs,
      std::vector<std::vector<cv::Point>> &blob_stats);
};
}  // namespace vision
}  // namespace y2022

#endif  // Y2022_BLOB_DETECTOR_H_

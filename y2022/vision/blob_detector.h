#ifndef Y2022_BLOB_DETECTOR_H_
#define Y2022_BLOB_DETECTOR_H_

#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>

namespace y2022 {
namespace vision {

class BlobDetector {
 public:
  struct BlobStats {
    cv::Point centroid;
    double aspect_ratio;
    double area;
    size_t num_points;
  };

  BlobDetector() {}
  // Given an image, threshold it to find "green" pixels
  // Input: Color image
  // Output: Grayscale (binarized) image with green pixels set to 255
  static cv::Mat ThresholdImage(cv::Mat rgb_image);

  // Given binary image, extract blobs
  static std::vector<std::vector<cv::Point>> FindBlobs(cv::Mat threshold_image);

  // Extract stats for each blob
  static std::vector<BlobStats> ComputeStats(
      std::vector<std::vector<cv::Point>> blobs);

  // Filter blobs to get rid of noise, too small/large items, and blobs that
  // aren't in a circle. Returns a pair of filtered blobs and the average
  // of their centroids.
  static std::pair<std::vector<std::vector<cv::Point>>, cv::Point> FilterBlobs(
      std::vector<std::vector<cv::Point>> blobs,
      std::vector<BlobStats> blob_stats);

  // Draw Blobs on image
  // Optionally draw all blobs and filtered blobs
  static void DrawBlobs(
      cv::Mat view_image,
      const std::vector<std::vector<cv::Point>> &filtered_blobs,
      const std::vector<std::vector<cv::Point>> &unfiltered_blobs,
      const std::vector<BlobStats> &blob_stats, cv::Point centroid);

  static void ExtractBlobs(
      cv::Mat rgb_image, cv::Mat &binarized_image,
      std::vector<std::vector<cv::Point>> &filtered_blobs,
      std::vector<std::vector<cv::Point>> &unfiltered_blobs,
      std::vector<BlobStats> &blob_stats, cv::Point &centroid);
};
}  // namespace vision
}  // namespace y2022

#endif  // Y2022_BLOB_DETECTOR_H_

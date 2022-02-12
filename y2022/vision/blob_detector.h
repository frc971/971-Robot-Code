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

  struct BlobResult {
    cv::Mat binarized_image;
    std::vector<std::vector<cv::Point>> filtered_blobs, unfiltered_blobs;
    std::vector<BlobStats> blob_stats;
    // In sorted order from left to right on the circle
    std::vector<cv::Point> filtered_centroids;
    cv::Point centroid;
  };

  BlobDetector() {}

  // Given an image, threshold it to find "green" pixels
  // Input: Color image
  // Output: Grayscale (binarized) image with green pixels set to 255
  static cv::Mat ThresholdImage(cv::Mat bgr_image);

  // Given binary image, extract blobs
  static std::vector<std::vector<cv::Point>> FindBlobs(cv::Mat threshold_image);

  // Extract stats for each blob
  static std::vector<BlobStats> ComputeStats(
      const std::vector<std::vector<cv::Point>> &blobs);

  // Filter blobs to get rid of noise, too small/large items, and blobs that
  // aren't in a circle. Finds the filtered blobs, centroids, and the absolute
  // centroid.
  static void FilterBlobs(BlobResult *blob_result);

  // Draw Blobs on image
  // Optionally draw all blobs and filtered blobs
  static void DrawBlobs(const BlobResult &blob_result, cv::Mat view_image);

  static void ExtractBlobs(cv::Mat bgr_image, BlobResult *blob_result);
};
}  // namespace vision
}  // namespace y2022

#endif  // Y2022_BLOB_DETECTOR_H_

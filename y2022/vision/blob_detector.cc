#include "y2022/vision/blob_detector.h"

#include "aos/network/team_number.h"

DEFINE_uint64(green_delta, 50,
              "Required difference between green pixels vs. red and blue");
DEFINE_bool(use_outdoors, false,
            "If true, change thresholds to handle outdoor illumination");

namespace y2022 {
namespace vision {

cv::Mat BlobDetector::ThresholdImage(cv::Mat rgb_image) {
  cv::Mat binarized_image(cv::Size(rgb_image.cols, rgb_image.rows), CV_8UC1);
  for (int row = 0; row < rgb_image.rows; row++) {
    for (int col = 0; col < rgb_image.cols; col++) {
      cv::Vec3b pixel = rgb_image.at<cv::Vec3b>(row, col);
      uint8_t blue = pixel.val[0];
      uint8_t green = pixel.val[1];
      uint8_t red = pixel.val[2];
      // Simple filter that looks for green pixels sufficiently brigher than
      // red and blue
      if ((green > blue + 30) && (green > red + 50)) {
        binarized_image.at<uint8_t>(row, col) = 255;
      } else {
        binarized_image.at<uint8_t>(row, col) = 0;
      }
    }
  }

  return binarized_image;
}

std::vector<std::vector<cv::Point>> BlobDetector::FindBlobs(
    cv::Mat binarized_image) {
  // find the contours (blob outlines)
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(binarized_image, contours, hierarchy, cv::RETR_CCOMP,
                   cv::CHAIN_APPROX_SIMPLE);

  return contours;
}

std::vector<BlobDetector::BlobStats> BlobDetector::ComputeStats(
    std::vector<std::vector<cv::Point>> blobs) {
  std::vector<BlobDetector::BlobStats> blob_stats;
  for (auto blob : blobs) {
    // Make the blob convex before finding bounding box
    std::vector<cv::Point> convex_blob;
    cv::convexHull(blob, convex_blob);
    auto blob_size = cv::boundingRect(convex_blob).size();
    cv::Moments moments = cv::moments(convex_blob);

    const auto centroid =
        cv::Point(moments.m10 / moments.m00, moments.m01 / moments.m00);
    const double aspect_ratio =
        static_cast<double>(blob_size.width) / blob_size.height;
    const double area = moments.m00;
    const size_t points = blob.size();

    blob_stats.emplace_back(BlobStats{centroid, aspect_ratio, area, points});
  }
  return blob_stats;
}

// Filter blobs to get rid of noise, too large items, etc.
std::vector<std::vector<cv::Point>> BlobDetector::FilterBlobs(
    std::vector<std::vector<cv::Point>> blobs,
    std::vector<BlobDetector::BlobStats> blob_stats) {
  std::vector<std::vector<cv::Point>> filtered_blobs;
  auto blob_it = blobs.begin();
  auto stats_it = blob_stats.begin();
  while (blob_it < blobs.end() && stats_it < blob_stats.end()) {
    constexpr int kMaxY = 400;
    constexpr double kTapeAspectRatio = 5.0 / 2.0;
    constexpr double kAspectRatioThreshold = 1.5;
    constexpr double kMinArea = 10;
    constexpr size_t kMinPoints = 2;
    // Remove all blobs that are at the bottom of the image, have a different
    // aspect ratio than the tape, or have too little area or points
    if ((stats_it->centroid.y <= kMaxY) &&
        (std::abs(kTapeAspectRatio - stats_it->aspect_ratio) <
         kAspectRatioThreshold) &&
        (stats_it->area >= kMinArea) && (stats_it->points >= kMinPoints)) {
      filtered_blobs.push_back(*blob_it);
    }
    blob_it++;
    stats_it++;
  }
  return filtered_blobs;
}

void BlobDetector::DrawBlobs(
    cv::Mat view_image,
    const std::vector<std::vector<cv::Point>> &unfiltered_blobs,
    const std::vector<std::vector<cv::Point>> &filtered_blobs,
    const std::vector<BlobStats> &blob_stats) {
  CHECK_GT(view_image.cols, 0);
  if (unfiltered_blobs.size() > 0) {
    // Draw blobs unfilled, with red color border
    drawContours(view_image, unfiltered_blobs, -1, cv::Scalar(0, 0, 255), 0);
  }

  drawContours(view_image, filtered_blobs, -1, cv::Scalar(0, 255, 0),
               cv::FILLED);

  for (auto stats : blob_stats) {
    cv::circle(view_image, stats.centroid, 2, cv::Scalar(255, 0, 0),
               cv::FILLED);
  }
}

void BlobDetector::ExtractBlobs(
    cv::Mat rgb_image, cv::Mat binarized_image, cv::Mat blob_image,
    std::vector<std::vector<cv::Point>> &filtered_blobs,
    std::vector<std::vector<cv::Point>> &unfiltered_blobs,
    std::vector<BlobStats> &blob_stats) {
  binarized_image = ThresholdImage(rgb_image);
  unfiltered_blobs = FindBlobs(binarized_image);
  blob_stats = ComputeStats(unfiltered_blobs);
  filtered_blobs = FilterBlobs(unfiltered_blobs, blob_stats);
  DrawBlobs(blob_image, unfiltered_blobs, filtered_blobs, blob_stats);
}

}  // namespace vision
}  // namespace y2022

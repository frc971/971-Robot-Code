#include "y2022/vision/blob_detector.h"

#include <opencv2/imgproc.hpp>

#include "aos/network/team_number.h"

DEFINE_uint64(green_delta, 50,
              "Required difference between green pixels vs. red and blue");
DEFINE_bool(use_outdoors, false,
            "If true, change thresholds to handle outdoor illumination");

namespace y2022 {
namespace vision {

cv::Mat BlobDetector::ThresholdImage(cv::Mat rgb_image) {
  cv::Mat gray_image(cv::Size(rgb_image.cols, rgb_image.rows), CV_8UC1);
  for (int row = 0; row < rgb_image.rows; row++) {
    for (int col = 0; col < rgb_image.cols; col++) {
      cv::Vec3b pixel = rgb_image.at<cv::Vec3b>(row, col);
      uint8_t blue = pixel.val[0];
      uint8_t green = pixel.val[1];
      uint8_t red = pixel.val[2];
      // Simple filter that looks for green pixels sufficiently brigher than
      // red and blue
      if ((green > blue + 30) && (green > red + 50)) {
        gray_image.at<uint8_t>(row, col) = 255;
      } else {
        gray_image.at<uint8_t>(row, col) = 0;
      }
    }
  }

  return gray_image;
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

// Filter blobs to get rid of noise, too large items, etc.
std::vector<std::vector<cv::Point>> BlobDetector::FilterBlobs(
    std::vector<std::vector<cv::Point>> blobs) {
  // TODO: Put in some filters

  std::vector<std::vector<cv::Point>> filtered_blobs;
  for (auto blob : blobs) {
    // for now, let's remove all blobs that are at the bottom of the image
    if (blob[0].y < 400) {
      filtered_blobs.push_back(blob);
    } else {
      // LOG(INFO) << "Found and removed blob";
    }
  }
  return filtered_blobs;
}

void BlobDetector::DrawBlobs(
    cv::Mat view_image, std::vector<std::vector<cv::Point>> unfiltered_blobs,
    std::vector<std::vector<cv::Point>> filtered_blobs) {
  CHECK_GT(view_image.cols, 0);
  if (unfiltered_blobs.size() > 0) {
    // Draw blobs unfilled, with red color border
    drawContours(view_image, unfiltered_blobs, -1, cv::Scalar(0, 0, 255), 0);
  }

  drawContours(view_image, filtered_blobs, -1, cv::Scalar(0, 255, 0),
               cv::FILLED);
}

std::vector<std::vector<cv::Point>> BlobDetector::ComputeStats(
    std::vector<std::vector<cv::Point>> blobs) {
  // Placeholder for now for this
  // TODO<Jim>: need to compute stats on blobs, like centroid, aspect
  // ratio, bounding box
  return blobs;
}

void BlobDetector::ExtractBlobs(
    cv::Mat rgb_image, cv::Mat binarized_image, cv::Mat blob_image,
    std::vector<std::vector<cv::Point>> &filtered_blobs,
    std::vector<std::vector<cv::Point>> &unfiltered_blobs,
    std::vector<std::vector<cv::Point>> &blob_stats) {
  binarized_image = ThresholdImage(rgb_image);
  unfiltered_blobs = FindBlobs(binarized_image);
  filtered_blobs = FilterBlobs(unfiltered_blobs);
  DrawBlobs(blob_image, unfiltered_blobs, filtered_blobs);
  blob_stats = ComputeStats(filtered_blobs);
}

}  // namespace vision
}  // namespace y2022

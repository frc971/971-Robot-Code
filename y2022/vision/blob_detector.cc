#include "y2022/vision/blob_detector.h"

#include <cmath>
#include <optional>
#include <string>

#include "aos/network/team_number.h"
#include "aos/time/time.h"
#include "opencv2/features2d.hpp"
#include "opencv2/imgproc.hpp"
#include "y2022/vision/geometry.h"

DEFINE_uint64(red_delta, 100,
              "Required difference between green pixels vs. red");
DEFINE_uint64(blue_delta, 50,
              "Required difference between green pixels vs. blue");

DEFINE_bool(use_outdoors, false,
            "If true, change thresholds to handle outdoor illumination");
DEFINE_uint64(outdoors_red_delta, 100,
              "Difference between green pixels vs. red, when outdoors");
DEFINE_uint64(outdoors_blue_delta, 1,
              "Difference between green pixels vs. blue, when outdoors");

namespace y2022 {
namespace vision {

cv::Mat BlobDetector::ThresholdImage(cv::Mat bgr_image) {
  size_t red_delta = FLAGS_red_delta;
  size_t blue_delta = FLAGS_blue_delta;

  if (FLAGS_use_outdoors) {
    red_delta = FLAGS_outdoors_red_delta;
    blue_delta = FLAGS_outdoors_blue_delta;
  }

  cv::Mat binarized_image(cv::Size(bgr_image.cols, bgr_image.rows), CV_8UC1);
  for (int row = 0; row < bgr_image.rows; row++) {
    for (int col = 0; col < bgr_image.cols; col++) {
      cv::Vec3b pixel = bgr_image.at<cv::Vec3b>(row, col);
      uint8_t blue = pixel.val[0];
      uint8_t green = pixel.val[1];
      uint8_t red = pixel.val[2];
      // Simple filter that looks for green pixels sufficiently brigher than
      // red and blue
      if ((green > blue + blue_delta) && (green > red + red_delta)) {
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
    const std::vector<std::vector<cv::Point>> &blobs) {
  std::vector<BlobDetector::BlobStats> blob_stats;
  for (auto blob : blobs) {
    auto blob_size = cv::boundingRect(blob).size();
    cv::Moments moments = cv::moments(blob);

    const auto centroid =
        cv::Point(moments.m10 / moments.m00, moments.m01 / moments.m00);
    const double aspect_ratio =
        static_cast<double>(blob_size.width) / blob_size.height;
    const double area = moments.m00;
    const size_t num_points = blob.size();

    blob_stats.emplace_back(
        BlobStats{centroid, aspect_ratio, area, num_points});
  }
  return blob_stats;
}

void BlobDetector::FilterBlobs(BlobResult *blob_result) {
  std::vector<std::vector<cv::Point>> filtered_blobs;
  std::vector<BlobStats> filtered_stats;

  auto blob_it = blob_result->unfiltered_blobs.begin();
  auto stats_it = blob_result->blob_stats.begin();
  while (blob_it < blob_result->unfiltered_blobs.end() &&
         stats_it < blob_result->blob_stats.end()) {
    constexpr double kTapeAspectRatio = 5.0 / 2.0;
    constexpr double kAspectRatioThreshold = 1.6;
    constexpr double kMinArea = 10;
    constexpr size_t kMinNumPoints = 6;

    // Remove all blobs that are at the bottom of the image, have a different
    // aspect ratio than the tape, or have too little area or points.
    if ((std::abs(1.0 - kTapeAspectRatio / stats_it->aspect_ratio) <
         kAspectRatioThreshold) &&
        (stats_it->area >= kMinArea) &&
        (stats_it->num_points >= kMinNumPoints)) {
      filtered_blobs.push_back(*blob_it);
      filtered_stats.push_back(*stats_it);
    }
    blob_it++;
    stats_it++;
  }

  // Threshold for mean distance from a blob centroid to a circle.
  constexpr double kCircleDistanceThreshold = 10.0;
  // We should only expect to see blobs between these angles on a circle.
  constexpr double kDegToRad = M_PI / 180.0;
  constexpr double kMinBlobAngle = 50.0 * kDegToRad;
  constexpr double kMaxBlobAngle = M_PI - kMinBlobAngle;
  std::vector<std::vector<cv::Point>> blob_circle;
  Circle circle;
  std::vector<cv::Point2d> centroids;

  // If we see more than this number of blobs after filtering based on
  // color/size, the circle fit may detect noise so just return no blobs.
  constexpr size_t kMinFilteredBlobs = 3;
  constexpr size_t kMaxFilteredBlobs = 50;
  if (filtered_blobs.size() >= kMinFilteredBlobs &&
      filtered_blobs.size() <= kMaxFilteredBlobs) {
    constexpr size_t kRansacIterations = 15;
    for (size_t i = 0; i < kRansacIterations; i++) {
      // Pick 3 random blobs and see how many fit on their circle
      const size_t j = std::rand() % filtered_blobs.size();
      const size_t k = std::rand() % filtered_blobs.size();
      const size_t l = std::rand() % filtered_blobs.size();

      // Restart if the random indices clash
      if ((j == k) || (j == l) || (k == l)) {
        i--;
        continue;
      }

      std::vector<std::vector<cv::Point>> current_blobs{
          filtered_blobs[j], filtered_blobs[k], filtered_blobs[l]};
      std::vector<cv::Point2d> current_centroids{filtered_stats[j].centroid,
                                                 filtered_stats[k].centroid,
                                                 filtered_stats[l].centroid};
      const std::optional<Circle> current_circle =
          Circle::Fit(current_centroids);

      // Make sure that a circle could be created from the points
      if (!current_circle) {
        continue;
      }

      // Only try to fit points to this circle if all of these are between
      // certain angles.
      if (current_circle->InAngleRange(current_centroids[0], kMinBlobAngle,
                                       kMaxBlobAngle) &&
          current_circle->InAngleRange(current_centroids[1], kMinBlobAngle,
                                       kMaxBlobAngle) &&
          current_circle->InAngleRange(current_centroids[2], kMinBlobAngle,
                                       kMaxBlobAngle)) {
        for (size_t m = 0; m < filtered_blobs.size(); m++) {
          // Add this blob to the list if it is close to the circle, is on the
          // top half,  and isn't one of the other blobs
          if ((m != j) && (m != k) && (m != l) &&
              current_circle->InAngleRange(filtered_stats[m].centroid,
                                           kMinBlobAngle, kMaxBlobAngle) &&
              (current_circle->DistanceTo(filtered_stats[m].centroid) <
               kCircleDistanceThreshold)) {
            current_blobs.emplace_back(filtered_blobs[m]);
            current_centroids.emplace_back(filtered_stats[m].centroid);
          }
        }

        if (current_blobs.size() > blob_circle.size()) {
          blob_circle = current_blobs;
          circle = *current_circle;
          centroids = current_centroids;
        }
      }
    }
  }

  cv::Point avg_centroid(-1, -1);
  if (centroids.size() > 0) {
    for (auto centroid : centroids) {
      avg_centroid.x += centroid.x;
      avg_centroid.y += centroid.y;
    }
    avg_centroid.x /= centroids.size();
    avg_centroid.y /= centroids.size();

    for (auto centroid : centroids) {
      blob_result->filtered_centroids.emplace_back(
          static_cast<int>(centroid.x), static_cast<int>(centroid.y));
    }

    // Sort the filtered centroids to make them go from left to right
    std::sort(blob_result->filtered_centroids.begin(),
              blob_result->filtered_centroids.end(),
              [&circle](cv::Point p, cv::Point q) {
                // If the angle is greater, it is more left and should be
                // considered "less" for sorting
                return circle.AngleOf(p) > circle.AngleOf(q);
              });
  }

  blob_result->filtered_blobs = blob_circle;
  blob_result->centroid = avg_centroid;
}

void BlobDetector::DrawBlobs(const BlobResult &blob_result,
                             cv::Mat view_image) {
  CHECK_GT(view_image.cols, 0);
  if (blob_result.unfiltered_blobs.size() > 0) {
    // Draw blobs unfilled, with red color border
    cv::drawContours(view_image, blob_result.unfiltered_blobs, -1,
                     cv::Scalar(0, 0, 255), 0);
  }

  cv::drawContours(view_image, blob_result.filtered_blobs, -1,
                   cv::Scalar(0, 100, 0), cv::FILLED);

  static constexpr double kCircleRadius = 2.0;
  // Draw blob centroids
  for (auto stats : blob_result.blob_stats) {
    cv::circle(view_image, stats.centroid, kCircleRadius,
               cv::Scalar(0, 215, 255), cv::FILLED);
  }
  for (auto centroid : blob_result.filtered_centroids) {
    cv::circle(view_image, centroid, kCircleRadius, cv::Scalar(0, 255, 0),
               cv::FILLED);
  }

  // Draw average centroid
  cv::circle(view_image, blob_result.centroid, kCircleRadius,
             cv::Scalar(255, 255, 0), cv::FILLED);
}

void BlobDetector::ExtractBlobs(cv::Mat bgr_image,
                                BlobDetector::BlobResult *blob_result) {
  auto start = aos::monotonic_clock::now();
  blob_result->binarized_image = ThresholdImage(bgr_image);
  blob_result->unfiltered_blobs = FindBlobs(blob_result->binarized_image);
  blob_result->blob_stats = ComputeStats(blob_result->unfiltered_blobs);
  FilterBlobs(blob_result);
  auto end = aos::monotonic_clock::now();
  VLOG(2) << "Blob detection elapsed time: "
          << std::chrono::duration<double, std::milli>(end - start).count()
          << " ms";
}

}  // namespace vision
}  // namespace y2022

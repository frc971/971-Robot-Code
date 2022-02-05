#include "y2022/vision/blob_detector.h"

#include <cmath>
#include <optional>
#include <string>

#include "aos/network/team_number.h"
#include "aos/time/time.h"
#include "opencv2/features2d.hpp"
#include "opencv2/imgproc.hpp"

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
      if ((green > blue + FLAGS_green_delta) &&
          (green > red + FLAGS_green_delta)) {
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

namespace {

// Linear equation in the form ax + by = c
struct Line {
 public:
  double a, b, c;

  std::optional<cv::Point2d> Intersection(const Line &l) const {
    // Use Cramer's rule to solve for the intersection
    const double denominator = Determinant(a, b, l.a, l.b);
    const double numerator_x = Determinant(c, b, l.c, l.b);
    const double numerator_y = Determinant(a, c, l.a, l.c);

    std::optional<cv::Point2d> intersection = std::nullopt;
    // Return nullopt if the denominator is 0, meaning the same slopes
    if (denominator != 0) {
      intersection =
          cv::Point2d(numerator_x / denominator, numerator_y / denominator);
    }

    return intersection;
  }

 private:  // Determinant of [[a, b], [c, d]]
  static double Determinant(double a, double b, double c, double d) {
    return (a * d) - (b * c);
  }
};

struct Circle {
 public:
  cv::Point2d center;
  double radius;

  static std::optional<Circle> Fit(std::vector<cv::Point2d> centroids) {
    CHECK_EQ(centroids.size(), 3ul);
    // For the 3 points, we have 3 equations in the form
    // (x - h)^2 + (y - k)^2 = r^2
    // Manipulate them to solve for the center and radius
    // (x1 - h)^2 + (y1 - k)^2 = r^2 ->
    // x1^2 + h^2 - 2x1h + y1^2 + k^2 - 2y1k = r^2
    // Also, (x2 - h)^2 + (y2 - k)^2 = r^2
    // Subtracting these two, we get
    // x1^2 - x2^2 - 2h(x1 - x2) + y1^2 - y2^2 - 2k(y1 - y2) = 0 ->
    // h(x1 - x2) + k(y1 - y2) = (-x1^2 + x2^2 - y1^2 + y2^2) / -2
    // Doing the same with equations 1 and 3, we get the second linear equation
    // h(x1 - x3) + k(y1 - y3) = (-x1^2 + x3^2 - y1^2 + y3^2) / -2
    // Now, we can solve for their intersection and find the center
    const auto l =
        Line{centroids[0].x - centroids[1].x, centroids[0].y - centroids[1].y,
             (-std::pow(centroids[0].x, 2) + std::pow(centroids[1].x, 2) -
              std::pow(centroids[0].y, 2) + std::pow(centroids[1].y, 2)) /
                 -2.0};
    const auto m =
        Line{centroids[0].x - centroids[2].x, centroids[0].y - centroids[2].y,
             (-std::pow(centroids[0].x, 2) + std::pow(centroids[2].x, 2) -
              std::pow(centroids[0].y, 2) + std::pow(centroids[2].y, 2)) /
                 -2.0};
    const auto center = l.Intersection(m);

    std::optional<Circle> circle = std::nullopt;
    if (center) {
      // Now find the radius
      const double radius = cv::norm(centroids[0] - *center);
      circle = Circle{*center, radius};
    }
    return circle;
  }

  double DistanceTo(cv::Point2d p) const {
    // Translate the point so that the circle orgin can be (0, 0)
    const auto p_prime = cv::Point2d(p.y - center.y, p.x - center.x);
    // Now, the distance is simply the difference between distance from the
    // origin to p' and the radius.
    return std::abs(cv::norm(p_prime) - radius);
  }

  // Inverted because y-coordinates go backwards
  bool OnTopHalf(cv::Point2d p) const { return p.y <= center.y; }
};

}  // namespace

std::pair<std::vector<std::vector<cv::Point>>, cv::Point>
BlobDetector::FilterBlobs(std::vector<std::vector<cv::Point>> blobs,
                          std::vector<BlobDetector::BlobStats> blob_stats) {
  std::vector<std::vector<cv::Point>> filtered_blobs;
  std::vector<BlobStats> filtered_stats;

  auto blob_it = blobs.begin();
  auto stats_it = blob_stats.begin();
  while (blob_it < blobs.end() && stats_it < blob_stats.end()) {
    // To estimate the maximum y, we can figure out the y value of the blobs
    // when the camera is the farthest from the target, at the field corner.
    // We can solve for the pitch of the blob:
    // blob_pitch = atan((height_tape - height_camera) / depth) + camera_pitch
    // The triangle with the height of the tape above the camera and the camera
    // depth is similar to the one with the focal length in y pixels and the y
    // coordinate offset from the center of the image.
    // Therefore y_offset = focal_length_y * tan(blob_pitch), and
    // y = -(y_offset - offset_y)
    constexpr int kMaxY = 400;
    constexpr double kTapeAspectRatio = 5.0 / 2.0;
    constexpr double kAspectRatioThreshold = 1.5;
    constexpr double kMinArea = 10;
    constexpr size_t kMinPoints = 6;

    // Remove all blobs that are at the bottom of the image, have a different
    // aspect ratio than the tape, or have too little area or points
    // TODO(milind): modify to take into account that blobs will be on the side.
    if ((stats_it->centroid.y <= kMaxY) &&
        (std::abs(kTapeAspectRatio - stats_it->aspect_ratio) <
         kAspectRatioThreshold) &&
        (stats_it->area >= kMinArea) && (stats_it->points >= kMinPoints)) {
      filtered_blobs.push_back(*blob_it);
      filtered_stats.push_back(*stats_it);
    }
    blob_it++;
    stats_it++;
  }

  // Threshold for mean distance from a blob centroid to a circle.
  constexpr double kCircleDistanceThreshold = 5.0;
  std::vector<std::vector<cv::Point>> blob_circle;
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
      const std::optional<Circle> circle = Circle::Fit(current_centroids);

      // Make sure that a circle could be created from the points
      if (!circle) {
        continue;
      }

      // Only try to fit points to this circle if all of these are on the top
      // half, like how the blobs should be
      if (circle->OnTopHalf(current_centroids[0]) &&
          circle->OnTopHalf(current_centroids[1]) &&
          circle->OnTopHalf(current_centroids[2])) {
        for (size_t m = 0; m < filtered_blobs.size(); m++) {
          // Add this blob to the list if it is close to the circle, is on the
          // top half,  and isn't one of the other blobs
          if ((m != i) && (m != j) && (m != k) &&
              circle->OnTopHalf(filtered_stats[m].centroid) &&
              (circle->DistanceTo(filtered_stats[m].centroid) <
               kCircleDistanceThreshold)) {
            current_blobs.emplace_back(filtered_blobs[m]);
            current_centroids.emplace_back(filtered_stats[m].centroid);
          }
        }

        if (current_blobs.size() > blob_circle.size()) {
          blob_circle = current_blobs;
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
  }

  return {blob_circle, avg_centroid};
}

void BlobDetector::DrawBlobs(
    cv::Mat view_image,
    const std::vector<std::vector<cv::Point>> &unfiltered_blobs,
    const std::vector<std::vector<cv::Point>> &filtered_blobs,
    const std::vector<BlobStats> &blob_stats, cv::Point centroid) {
  CHECK_GT(view_image.cols, 0);
  if (unfiltered_blobs.size() > 0) {
    // Draw blobs unfilled, with red color border
    cv::drawContours(view_image, unfiltered_blobs, -1, cv::Scalar(0, 0, 255),
                     0);
  }

  cv::drawContours(view_image, filtered_blobs, -1, cv::Scalar(0, 255, 0),
                   cv::FILLED);

  // Draw blob centroids
  for (auto stats : blob_stats) {
    cv::circle(view_image, stats.centroid, 2, cv::Scalar(255, 0, 0),
               cv::FILLED);
  }

  // Draw average centroid
  cv::circle(view_image, centroid, 3, cv::Scalar(255, 255, 0), cv::FILLED);
}

void BlobDetector::ExtractBlobs(
    cv::Mat rgb_image, cv::Mat &binarized_image, cv::Mat blob_image,
    std::vector<std::vector<cv::Point>> &filtered_blobs,
    std::vector<std::vector<cv::Point>> &unfiltered_blobs,
    std::vector<BlobStats> &blob_stats, cv::Point &centroid) {
  auto start = aos::monotonic_clock::now();
  binarized_image = ThresholdImage(rgb_image);
  unfiltered_blobs = FindBlobs(binarized_image);
  blob_stats = ComputeStats(unfiltered_blobs);
  auto filtered_pair = FilterBlobs(unfiltered_blobs, blob_stats);
  filtered_blobs = filtered_pair.first;
  centroid = filtered_pair.second;
  auto end = aos::monotonic_clock::now();
  LOG(INFO) << "Blob detection elapsed time: "
            << std::chrono::duration<double, std::milli>(end - start).count()
            << " ms";
  DrawBlobs(blob_image, unfiltered_blobs, filtered_blobs, blob_stats, centroid);
}

}  // namespace vision
}  // namespace y2022

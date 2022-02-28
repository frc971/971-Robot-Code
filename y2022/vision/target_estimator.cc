#include "y2022/vision/target_estimator.h"

#include "absl/strings/str_format.h"
#include "aos/time/time.h"
#include "ceres/ceres.h"
#include "frc971/control_loops/quaternion_utils.h"
#include "opencv2/core/core.hpp"
#include "opencv2/core/eigen.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc.hpp"

DEFINE_bool(freeze_roll, true, "If true, don't solve for roll");
DEFINE_bool(freeze_pitch, false, "If true, don't solve for pitch");
DEFINE_bool(freeze_yaw, false, "If true, don't solve for yaw");
DEFINE_bool(freeze_camera_height, true,
            "If true, don't solve for camera height");
DEFINE_bool(freeze_angle_to_camera, true,
            "If true, don't solve for polar angle to camera");

DEFINE_uint64(max_num_iterations, 200,
              "Maximum number of iterations for the ceres solver");
DEFINE_bool(solver_output, false,
            "If true, log the solver progress and results");

namespace y2022::vision {

std::vector<cv::Point3d> TargetEstimator::ComputeTapePoints() {
  std::vector<cv::Point3d> tape_points;
  tape_points.reserve(kNumPiecesOfTape);

  constexpr size_t kNumVisiblePiecesOfTape = 5;
  for (size_t i = 0; i < kNumVisiblePiecesOfTape; i++) {
    // The center piece of tape is at 0 rad, so the angle indices are offset
    // by the number of pieces of tape on each side of it
    const double theta_index =
        static_cast<double>(i) - ((kNumVisiblePiecesOfTape - 1) / 2);
    // The polar angle is a multiple of the angle between tape centers
    double theta = theta_index * ((2.0 * M_PI) / kNumPiecesOfTape);
    tape_points.emplace_back(kUpperHubRadius * std::cos(theta),
                             kUpperHubRadius * std::sin(theta), kTapeHeight);
  }

  return tape_points;
}

const std::vector<cv::Point3d> TargetEstimator::kTapePoints =
    ComputeTapePoints();

TargetEstimator::TargetEstimator(cv::Mat intrinsics, cv::Mat extrinsics)
    : centroids_(),
      image_(std::nullopt),
      roll_(0.0),
      pitch_(0.0),
      yaw_(M_PI),
      distance_(3.0),
      angle_to_camera_(0.0),
      // TODO(milind): add IMU height
      camera_height_(extrinsics.at<double>(2, 3)) {
  cv::cv2eigen(intrinsics, intrinsics_);
  cv::cv2eigen(extrinsics, extrinsics_);
}

namespace {
void SetBoundsOrFreeze(double *param, bool freeze, double min, double max,
                       ceres::Problem *problem) {
  if (freeze) {
    problem->SetParameterization(
        param, new ceres::SubsetParameterization(1, std::vector<int>{0}));
  } else {
    problem->SetParameterLowerBound(param, 0, min);
    problem->SetParameterUpperBound(param, 0, max);
  }
}
}  // namespace

void TargetEstimator::Solve(const std::vector<cv::Point> &centroids,
                            std::optional<cv::Mat> image) {
  auto start = aos::monotonic_clock::now();

  centroids_ = centroids;
  image_ = image;

  ceres::Problem problem;

  // Set up the only cost function (also known as residual). This uses
  // auto-differentiation to obtain the derivative (jacobian).
  ceres::CostFunction *cost_function =
      new ceres::AutoDiffCostFunction<TargetEstimator, ceres::DYNAMIC, 1, 1, 1,
                                      1, 1, 1>(this, centroids_.size() * 2,
                                               ceres::DO_NOT_TAKE_OWNERSHIP);

  // TODO(milind): add loss function when we get more noisy data
  problem.AddResidualBlock(cost_function, nullptr, &roll_, &pitch_, &yaw_,
                           &distance_, &angle_to_camera_, &camera_height_);

  // TODO(milind): seed values at localizer output, and constrain to be close to
  // that.

  // Constrain the rotation, otherwise there can be multiple solutions.
  // There shouldn't be too much roll or pitch
  constexpr double kMaxRoll = 0.1;
  SetBoundsOrFreeze(&roll_, FLAGS_freeze_roll, -kMaxRoll, kMaxRoll, &problem);

  constexpr double kPitch = -35.0 * M_PI / 180.0;
  constexpr double kMaxPitchDelta = 0.15;
  SetBoundsOrFreeze(&pitch_, FLAGS_freeze_pitch, kPitch - kMaxPitchDelta,
                    kPitch + kMaxPitchDelta, &problem);
  // Constrain the yaw to where the target would be visible
  constexpr double kMaxYawDelta = M_PI / 4.0;
  SetBoundsOrFreeze(&yaw_, FLAGS_freeze_yaw, M_PI - kMaxYawDelta,
                    M_PI + kMaxYawDelta, &problem);

  constexpr double kMaxHeightDelta = 0.1;
  SetBoundsOrFreeze(&camera_height_, FLAGS_freeze_camera_height,
                    camera_height_ - kMaxHeightDelta,
                    camera_height_ + kMaxHeightDelta, &problem);

  // Distances shouldn't be too close to the target or too far
  constexpr double kMinDistance = 1.0;
  constexpr double kMaxDistance = 10.0;
  SetBoundsOrFreeze(&distance_, false, kMinDistance, kMaxDistance, &problem);

  // Keep the angle between +/- half of the angle between piece of tape
  constexpr double kMaxAngle = ((2.0 * M_PI) / kNumPiecesOfTape) / 2.0;
  SetBoundsOrFreeze(&angle_to_camera_, FLAGS_freeze_angle_to_camera, -kMaxAngle,
                    kMaxAngle, &problem);

  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = FLAGS_solver_output;
  options.gradient_tolerance = 1e-12;
  options.function_tolerance = 1e-16;
  options.parameter_tolerance = 1e-12;
  options.max_num_iterations = FLAGS_max_num_iterations;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  auto end = aos::monotonic_clock::now();
  LOG(INFO) << "Target estimation elapsed time: "
            << std::chrono::duration<double, std::milli>(end - start).count()
            << " ms";

  if (FLAGS_solver_output) {
    LOG(INFO) << summary.FullReport();

    LOG(INFO) << "roll: " << roll_;
    LOG(INFO) << "pitch: " << pitch_;
    LOG(INFO) << "yaw: " << yaw_;
    LOG(INFO) << "angle to target (based on yaw): " << angle_to_target();
    LOG(INFO) << "angle to camera (polar): " << angle_to_camera_;
    LOG(INFO) << "distance (polar): " << distance_;
    LOG(INFO) << "camera height: " << camera_height_;
  }
}

namespace {
// Hacks to extract a double from a scalar, which is either a ceres jet or a
// double. Only used for debugging and displaying.
template <typename S>
double ScalarToDouble(S s) {
  const double *ptr = reinterpret_cast<double *>(&s);
  return *ptr;
}

template <typename S>
cv::Point2d ScalarPointToDouble(cv::Point_<S> p) {
  return cv::Point2d(ScalarToDouble(p.x), ScalarToDouble(p.y));
}
}  // namespace

template <typename S>
bool TargetEstimator::operator()(const S *const roll, const S *const pitch,
                                 const S *const yaw, const S *const distance,
                                 const S *const theta,
                                 const S *const camera_height,
                                 S *residual) const {
  using Vector3s = Eigen::Matrix<S, 3, 1>;
  using Affine3s = Eigen::Transform<S, 3, Eigen::Affine>;

  Eigen::AngleAxis<S> roll_angle(*roll, Vector3s::UnitX());
  Eigen::AngleAxis<S> pitch_angle(*pitch, Vector3s::UnitY());
  Eigen::AngleAxis<S> yaw_angle(*yaw, Vector3s::UnitZ());
  // Construct the rotation and translation of the camera in the hub's frame
  Eigen::Quaternion<S> R_camera_hub = yaw_angle * pitch_angle * roll_angle;
  Vector3s T_camera_hub(*distance * ceres::cos(*theta),
                        *distance * ceres::sin(*theta), *camera_height);

  Affine3s H_camera_hub = Eigen::Translation<S, 3>(T_camera_hub) * R_camera_hub;

  std::vector<cv::Point_<S>> tape_points_proj;
  for (cv::Point3d tape_point_hub : kTapePoints) {
    Vector3s tape_point_hub_eigen(S(tape_point_hub.x), S(tape_point_hub.y),
                                  S(tape_point_hub.z));

    // With X, Y, Z being world axes and x, y, z being camera axes,
    // x = Y, y = Z, z = -X
    static const Eigen::Matrix3d kCameraAxisConversion =
        (Eigen::Matrix3d() << 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0)
            .finished();
    // Project the 3d tape point onto the image using the transformation and
    // intrinsics
    Vector3s tape_point_proj =
        intrinsics_ * (kCameraAxisConversion *
                       (H_camera_hub.inverse() * tape_point_hub_eigen));

    // Normalize the projected point
    tape_points_proj.emplace_back(tape_point_proj.x() / tape_point_proj.z(),
                                  tape_point_proj.y() / tape_point_proj.z());
    VLOG(1) << "Projected tape point: "
            << ScalarPointToDouble(
                   tape_points_proj[tape_points_proj.size() - 1]);
  }

  for (size_t i = 0; i < centroids_.size(); i++) {
    const auto distance = DistanceFromTape(i, tape_points_proj);
    // Set the residual to the (x, y) distance of the centroid from the
    // nearest projected piece of tape
    residual[i * 2] = distance.x;
    residual[(i * 2) + 1] = distance.y;
  }

  if (image_.has_value()) {
    // Draw the current stage of the solving
    cv::Mat image = image_->clone();
    for (size_t i = 0; i < tape_points_proj.size() - 1; i++) {
      cv::line(image, ScalarPointToDouble(tape_points_proj[i]),
               ScalarPointToDouble(tape_points_proj[i + 1]),
               cv::Scalar(255, 255, 255));
      cv::circle(image, ScalarPointToDouble(tape_points_proj[i]), 2,
                 cv::Scalar(255, 20, 147), cv::FILLED);
      cv::circle(image, ScalarPointToDouble(tape_points_proj[i + 1]), 2,
                 cv::Scalar(255, 20, 147), cv::FILLED);
    }
    cv::imshow("image", image);
    cv::waitKey(10);
  }

  return true;
}

namespace {
template <typename S>
cv::Point_<S> Distance(cv::Point p, cv::Point_<S> q) {
  return cv::Point_<S>(S(p.x) - q.x, S(p.y) - q.y);
}

template <typename S>
bool Less(cv::Point_<S> distance_1, cv::Point_<S> distance_2) {
  return (ceres::pow(distance_1.x, 2) + ceres::pow(distance_1.y, 2) <
          ceres::pow(distance_2.x, 2) + ceres::pow(distance_2.y, 2));
}
}  // namespace

template <typename S>
cv::Point_<S> TargetEstimator::DistanceFromTape(
    size_t centroid_index,
    const std::vector<cv::Point_<S>> &tape_points) const {
  // Figure out the middle index in the tape points
  size_t middle_index = centroids_.size() / 2;
  if (centroids_.size() % 2 == 0) {
    // There are two possible middles in this case. Figure out which one fits
    // the current better
    const auto tape_middle = tape_points[tape_points.size() / 2];
    const auto middle_distance_1 =
        Distance(centroids_[(centroids_.size() / 2) - 1], tape_middle);
    const auto middle_distance_2 =
        Distance(centroids_[centroids_.size() / 2], tape_middle);
    if (Less(middle_distance_1, middle_distance_2)) {
      middle_index--;
    }
  }

  auto distance = cv::Point_<S>(std::numeric_limits<S>::infinity(),
                                std::numeric_limits<S>::infinity());
  if (centroid_index == middle_index) {
    // Fix the middle centroid so the solver can't go too far off
    distance =
        Distance(centroids_[middle_index], tape_points[tape_points.size() / 2]);
  } else {
    // Give the other centroids some freedom in case some are split into pieces
    for (auto tape_point : tape_points) {
      const auto current_distance =
          Distance(centroids_[centroid_index], tape_point);
      if (Less(current_distance, distance)) {
        distance = current_distance;
      }
    }
  }

  return distance;
}

namespace {
void DrawEstimateValues(double distance, double angle_to_target,
                        double angle_to_camera, double roll, double pitch,
                        double yaw, cv::Mat view_image) {
  constexpr int kTextX = 10;
  int text_y = 330;
  constexpr int kTextSpacing = 35;

  const auto kTextColor = cv::Scalar(0, 255, 255);
  constexpr double kFontScale = 1.0;

  cv::putText(view_image, absl::StrFormat("Distance: %.3f", distance),
              cv::Point(kTextX, text_y += kTextSpacing),
              cv::FONT_HERSHEY_DUPLEX, kFontScale, kTextColor, 2);
  cv::putText(view_image,
              absl::StrFormat("Angle to target: %.3f", angle_to_target),
              cv::Point(kTextX, text_y += kTextSpacing),
              cv::FONT_HERSHEY_DUPLEX, kFontScale, kTextColor, 2);
  cv::putText(view_image,
              absl::StrFormat("Angle to camera: %.3f", angle_to_camera),
              cv::Point(kTextX, text_y += kTextSpacing),
              cv::FONT_HERSHEY_DUPLEX, kFontScale, kTextColor, 2);

  cv::putText(
      view_image,
      absl::StrFormat("Roll: %.3f, pitch: %.3f, yaw: %.3f", roll, pitch, yaw),
      cv::Point(kTextX, text_y += kTextSpacing), cv::FONT_HERSHEY_DUPLEX,
      kFontScale, kTextColor, 2);
}
}  // namespace

void TargetEstimator::DrawEstimate(const TargetEstimate &target_estimate,
                                   cv::Mat view_image) {
  DrawEstimateValues(target_estimate.distance(),
                     target_estimate.angle_to_target(),
                     target_estimate.angle_to_camera(),
                     target_estimate.rotation_camera_hub()->roll(),
                     target_estimate.rotation_camera_hub()->pitch(),
                     target_estimate.rotation_camera_hub()->yaw(), view_image);
}

void TargetEstimator::DrawEstimate(cv::Mat view_image) const {
  DrawEstimateValues(distance_, angle_to_target(), angle_to_camera_, roll_,
                     pitch_, yaw_, view_image);
}

}  // namespace y2022::vision

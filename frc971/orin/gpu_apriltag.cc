#include "frc971/orin/gpu_apriltag.h"

#include <chrono>

#include "third_party/apriltag/apriltag.h"
#include "third_party/apriltag/apriltag_pose.h"
#include "third_party/apriltag/tag16h5.h"
#include "third_party/apriltag/tag36h11.h"
#include <opencv2/highgui.hpp>

#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/logging/logging.h"
#include "aos/realtime.h"
#include "frc971/constants/constants_sender_lib.h"
#include "frc971/orin/apriltag.h"
#include "frc971/vision/calibration_generated.h"
#include "frc971/vision/charuco_lib.h"
#include "frc971/vision/vision_util_lib.h"

DEFINE_bool(debug, false, "If true, write debug images.");
DEFINE_double(
    max_expected_distortion, 0.314,
    "Maximum expected value for unscaled distortion factors. Will scale "
    "distortion factors so that this value (and a higher distortion) maps to "
    "1.0.");
DEFINE_double(min_decision_margin, 50.0,
              "Minimum decision margin (confidence) for an apriltag detection");
DEFINE_int32(pixel_border, 150,
             "Size of image border within which to reject detected corners");
DEFINE_uint64(pose_estimation_iterations, 50,
              "Number of iterations for apriltag pose estimation.");

namespace frc971::apriltag {

// Set max age on image for processing at 20 ms.  For 60Hz, we should be
// processing at least every 16.7ms
constexpr aos::monotonic_clock::duration kMaxImageAge =
    std::chrono::milliseconds(50);

namespace chrono = std::chrono;

CameraMatrix GetCameraMatrix(
    const frc971::vision::calibration::CameraCalibration *calibration) {
  auto intrinsics = calibration->intrinsics();
  return CameraMatrix{
      .fx = intrinsics->Get(0),
      .cx = intrinsics->Get(2),
      .fy = intrinsics->Get(4),
      .cy = intrinsics->Get(5),
  };
}

DistCoeffs GetDistCoeffs(
    const frc971::vision::calibration::CameraCalibration *calibration) {
  auto dist_coeffs = calibration->dist_coeffs();
  return DistCoeffs{
      .k1 = dist_coeffs->Get(0),
      .k2 = dist_coeffs->Get(1),
      .p1 = dist_coeffs->Get(2),
      .p2 = dist_coeffs->Get(3),
      .k3 = dist_coeffs->Get(4),
  };
}

ApriltagDetector::ApriltagDetector(
    aos::EventLoop *event_loop, std::string_view channel_name,
    const frc971::vision::calibration::CameraCalibration *calibration,
    size_t width, size_t height)
    : tag_family_(tag36h11_create()),
      tag_detector_(MakeTagDetector(tag_family_)),
      node_name_(event_loop->node()->name()->string_view()),
      calibration_(calibration),
      intrinsics_(frc971::vision::CameraIntrinsics(calibration_)),
      extrinsics_(frc971::vision::CameraExtrinsics(calibration_)),
      dist_coeffs_(frc971::vision::CameraDistCoeffs(calibration_)),
      distortion_camera_matrix_(GetCameraMatrix(calibration_)),
      distortion_coefficients_(GetDistCoeffs(calibration_)),
      gpu_detector_(width, height, tag_detector_, distortion_camera_matrix_,
                    distortion_coefficients_),
      image_callback_(
          event_loop, channel_name,
          [this](cv::Mat image_color_mat,
                 const aos::monotonic_clock::time_point eof) {
            HandleImage(image_color_mat, eof);
          },
          kMaxImageAge),
      target_map_sender_(
          event_loop->MakeSender<frc971::vision::TargetMap>(channel_name)),
      image_annotations_sender_(
          event_loop->MakeSender<foxglove::ImageAnnotations>(channel_name)),
      rejections_(0) {
  image_callback_.set_format(frc971::vision::ImageCallback::Format::YUYV2);

  projection_matrix_ = cv::Mat::zeros(3, 4, CV_64F);
  intrinsics_.rowRange(0, 3).colRange(0, 3).copyTo(
      projection_matrix_.rowRange(0, 3).colRange(0, 3));
}

ApriltagDetector::~ApriltagDetector() {
  apriltag_detector_destroy(tag_detector_);
  free(tag_family_);
}

apriltag_detector_t *ApriltagDetector::MakeTagDetector(
    apriltag_family_t *tag_family) {
  apriltag_detector_t *tag_detector = apriltag_detector_create();

  apriltag_detector_add_family_bits(tag_detector, tag_family, 1);

  tag_detector->nthreads = 6;
  tag_detector->wp = workerpool_create(tag_detector->nthreads);
  tag_detector->qtp.min_white_black_diff = 5;
  tag_detector->debug = FLAGS_debug;

  return tag_detector;
}

flatbuffers::Offset<frc971::vision::TargetPoseFbs>
ApriltagDetector::BuildTargetPose(const Detection &detection,
                                  flatbuffers::FlatBufferBuilder *fbb) {
  const auto T =
      Eigen::Translation3d(detection.pose.t->data[0], detection.pose.t->data[1],
                           detection.pose.t->data[2]);
  const auto position_offset =
      frc971::vision::CreatePosition(*fbb, T.x(), T.y(), T.z());

  // Aprilrobotics stores the rotation matrix in row-major order
  const auto orientation = Eigen::Quaterniond(
      Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(detection.pose.R->data));
  const auto orientation_offset = frc971::vision::CreateQuaternion(
      *fbb, orientation.w(), orientation.x(), orientation.y(), orientation.z());

  return frc971::vision::CreateTargetPoseFbs(
      *fbb, detection.det.id, position_offset, orientation_offset,
      detection.det.decision_margin, detection.pose_error,
      detection.distortion_factor, detection.pose_error_ratio);
}

bool ApriltagDetector::UndistortDetection(apriltag_detection_t *det) const {
  // Copy the undistorted points into det
  bool converged = true;
  for (size_t i = 0; i < 4; i++) {
    double u = det->p[i][0];
    double v = det->p[i][1];

    converged &= GpuDetector::UnDistort(&u, &v, &distortion_camera_matrix_,
                                        &distortion_coefficients_);
    det->p[i][0] = u;
    det->p[i][1] = v;
  }
  return converged;
}

double ApriltagDetector::ComputeDistortionFactor(
    const std::vector<cv::Point2f> &orig_corners,
    const std::vector<cv::Point2f> &corners) {
  CHECK_EQ(orig_corners.size(), 4ul);
  CHECK_EQ(corners.size(), 4ul);

  double avg_distance = 0.0;
  for (size_t i = 0; i < corners.size(); i++) {
    avg_distance += cv::norm(orig_corners[i] - corners[i]);
  }
  avg_distance /= corners.size();

  // Normalize avg_distance by dividing by the image diagonal,
  // and then the maximum expected distortion
  double distortion_factor =
      avg_distance /
      cv::norm(cv::Point2d(image_size_.width, image_size_.height));
  return std::min(distortion_factor / FLAGS_max_expected_distortion, 1.0);
}

std::vector<cv::Point2f> ApriltagDetector::MakeCornerVector(
    const apriltag_detection_t *det) {
  std::vector<cv::Point2f> corner_points;
  corner_points.emplace_back(det->p[0][0], det->p[0][1]);
  corner_points.emplace_back(det->p[1][0], det->p[1][1]);
  corner_points.emplace_back(det->p[2][0], det->p[2][1]);
  corner_points.emplace_back(det->p[3][0], det->p[3][1]);

  return corner_points;
}

void ApriltagDetector::DestroyPose(apriltag_pose_t *pose) const {
  matd_destroy(pose->R);
  matd_destroy(pose->t);
}

void ApriltagDetector::HandleImage(cv::Mat color_image,
                                   aos::monotonic_clock::time_point eof) {
  const aos::monotonic_clock::time_point start_time =
      aos::monotonic_clock::now();
  gpu_detector_.Detect(color_image.data);
  image_size_ = color_image.size();
  cv::Mat image_copy;
  if (FLAGS_visualize) {
    // TODO: Need to figure out how to extract displayable color image from this
    image_copy = color_image.clone();
  }

  const zarray_t *detections = gpu_detector_.Detections();

  aos::monotonic_clock::time_point end_time = aos::monotonic_clock::now();

  const uint32_t min_x = FLAGS_pixel_border;
  const uint32_t max_x = color_image.cols - FLAGS_pixel_border;
  const uint32_t min_y = FLAGS_pixel_border;
  const uint32_t max_y = color_image.rows - FLAGS_pixel_border;

  // Define variables for storing / visualizing the output
  std::vector<Detection> results;
  auto builder = image_annotations_sender_.MakeBuilder();
  std::vector<flatbuffers::Offset<foxglove::PointsAnnotation>> foxglove_corners;

  for (int i = 0; i < zarray_size(detections); ++i) {
    apriltag_detection_t *gpu_detection;

    zarray_get(detections, i, &gpu_detection);

    bool valid = gpu_detection->decision_margin > FLAGS_min_decision_margin;

    if (valid) {
      // Reject tags that are too close to the boundary, since they often
      // lead to corrupt matches since part of the tag is cut off
      if (gpu_detection->p[0][0] < min_x || gpu_detection->p[0][0] > max_x ||
          gpu_detection->p[1][0] < min_x || gpu_detection->p[1][0] > max_x ||
          gpu_detection->p[2][0] < min_x || gpu_detection->p[2][0] > max_x ||
          gpu_detection->p[3][0] < min_x || gpu_detection->p[3][0] > max_x ||
          gpu_detection->p[0][1] < min_y || gpu_detection->p[0][1] > max_y ||
          gpu_detection->p[1][1] < min_y || gpu_detection->p[1][1] > max_y ||
          gpu_detection->p[2][1] < min_y || gpu_detection->p[2][1] > max_y ||
          gpu_detection->p[3][1] < min_y || gpu_detection->p[3][1] > max_y) {
        VLOG(1) << "Rejecting detection because corner is outside pixel border";

        // Send rejected corner points to foxglove in red
        std::vector<cv::Point2f> rejected_corner_points =
            MakeCornerVector(gpu_detection);
        foxglove_corners.push_back(frc971::vision::BuildPointsAnnotation(
            builder.fbb(), eof, rejected_corner_points,
            std::vector<double>{1.0, 0.0, 0.0, 0.5}));
        rejections_++;
        continue;
      }

      AOS_LOG(INFO,
              "Found GPU %s tag number %d hamming %d margin %f  (%f, %f), (%f, "
              "%f), (%f, %f), (%f, %f) in %f ms\n",
              valid ? "valid" : "invalid", gpu_detection->id,
              gpu_detection->hamming, gpu_detection->decision_margin,
              gpu_detection->p[0][0], gpu_detection->p[0][1],
              gpu_detection->p[1][0], gpu_detection->p[1][1],
              gpu_detection->p[2][0], gpu_detection->p[2][1],
              gpu_detection->p[3][0], gpu_detection->p[3][1],
              std::chrono::duration<float, std::milli>(end_time - start_time)
                  .count());

      VLOG(1) << "Found tag number " << gpu_detection->id
              << " hamming: " << gpu_detection->hamming
              << " margin: " << gpu_detection->decision_margin;

      // First create an apriltag_detection_info_t struct using your known
      // parameters.
      apriltag_detection_info_t info;
      info.tagsize = 6.5 * 0.0254;

      info.fx = intrinsics_.at<double>(0, 0);
      info.fy = intrinsics_.at<double>(1, 1);
      info.cx = intrinsics_.at<double>(0, 2);
      info.cy = intrinsics_.at<double>(1, 2);

      // Send original corner points in green
      std::vector<cv::Point2f> orig_corner_points =
          MakeCornerVector(gpu_detection);
      foxglove_corners.push_back(frc971::vision::BuildPointsAnnotation(
          builder.fbb(), eof, orig_corner_points,
          std::vector<double>{0.0, 1.0, 0.0, 0.5}));

      bool converged = UndistortDetection(gpu_detection);

      if (!converged) {
        VLOG(1) << "Rejecting detection because Undistort failed to coverge";

        // Send corner points rejected to to lack of convergence in orange
        std::vector<cv::Point2f> rejected_corner_points =
            MakeCornerVector(gpu_detection);
        foxglove_corners.push_back(frc971::vision::BuildPointsAnnotation(
            builder.fbb(), eof, rejected_corner_points,
            std::vector<double>{1.0, 0.65, 0.0, 0.5}));
        rejections_++;
        continue;
      }

      // We're setting this here to use the undistorted corner points in pose
      // estimation.
      info.det = gpu_detection;

      const aos::monotonic_clock::time_point before_pose_estimation =
          aos::monotonic_clock::now();

      apriltag_pose_t pose_1;
      apriltag_pose_t pose_2;
      double pose_error_1;
      double pose_error_2;
      estimate_tag_pose_orthogonal_iteration(&info, &pose_error_1, &pose_1,
                                             &pose_error_2, &pose_2,
                                             FLAGS_pose_estimation_iterations);

      const aos::monotonic_clock::time_point after_pose_estimation =
          aos::monotonic_clock::now();
      VLOG(1) << "Took "
              << chrono::duration<double>(after_pose_estimation -
                                          before_pose_estimation)
                     .count()
              << " seconds for pose estimation";
      VLOG(1) << "Pose err 1: " << std::setprecision(20) << std::fixed
              << pose_error_1 << " " << (pose_error_1 < 1e-6 ? "Good" : "Bad");
      VLOG(1) << "Pose err 2: " << std::setprecision(20) << std::fixed
              << pose_error_2 << " " << (pose_error_2 < 1e-6 ? "Good" : "Bad");

      // Send undistorted corner points in pink
      std::vector<cv::Point2f> corner_points = MakeCornerVector(gpu_detection);
      foxglove_corners.push_back(frc971::vision::BuildPointsAnnotation(
          builder.fbb(), eof, corner_points,
          std::vector<double>{1.0, 0.75, 0.8, 1.0}));

      double distortion_factor =
          ComputeDistortionFactor(orig_corner_points, corner_points);

      // We get two estimates for poses.
      // Choose the one with the lower pose estimation error
      bool use_pose_1 = (pose_error_1 < pose_error_2);
      auto best_pose = (use_pose_1 ? pose_1 : pose_2);
      auto secondary_pose = (use_pose_1 ? pose_2 : pose_1);
      double best_pose_error = (use_pose_1 ? pose_error_1 : pose_error_2);
      double secondary_pose_error = (use_pose_1 ? pose_error_2 : pose_error_1);

      CHECK_NE(best_pose_error, std::numeric_limits<double>::infinity())
          << "Got no valid pose estimations, this should not be possible.";
      double pose_error_ratio = best_pose_error / secondary_pose_error;

      // Destroy the secondary pose if we got one
      if (secondary_pose_error != std::numeric_limits<double>::infinity()) {
        DestroyPose(&secondary_pose);
      }

      results.emplace_back(Detection{.det = *gpu_detection,
                                     .pose = best_pose,
                                     .pose_error = best_pose_error,
                                     .distortion_factor = distortion_factor,
                                     .pose_error_ratio = pose_error_ratio});

      if (FLAGS_visualize) {
        // Draw raw (distorted) corner points in green
        cv::line(image_copy, orig_corner_points[0], orig_corner_points[1],
                 cv::Scalar(0, 255, 0), 2);
        cv::line(image_copy, orig_corner_points[1], orig_corner_points[2],
                 cv::Scalar(0, 255, 0), 2);
        cv::line(image_copy, orig_corner_points[2], orig_corner_points[3],
                 cv::Scalar(0, 255, 0), 2);
        cv::line(image_copy, orig_corner_points[3], orig_corner_points[0],
                 cv::Scalar(0, 255, 0), 2);

        // Draw undistorted corner points in red
        cv::line(image_copy, corner_points[0], corner_points[1],
                 cv::Scalar(0, 0, 255), 2);
        cv::line(image_copy, corner_points[2], corner_points[1],
                 cv::Scalar(0, 0, 255), 2);
        cv::line(image_copy, corner_points[2], corner_points[3],
                 cv::Scalar(0, 0, 255), 2);
        cv::line(image_copy, corner_points[0], corner_points[3],
                 cv::Scalar(0, 0, 255), 2);
      }

      VLOG(1) << "Found tag number " << gpu_detection->id
              << " hamming: " << gpu_detection->hamming
              << " margin: " << gpu_detection->decision_margin;
    } else {
      rejections_++;
    }
  }

  if (FLAGS_visualize) {
    // Display the result
    // Rotate by 180 degrees to make it upright
    // TODO: Make this an option?
    bool flip_image_ = true;
    if (flip_image_) {
      cv::rotate(image_copy, image_copy, 1);
    }
    // TODO: Need to fix image display to handle YUYV images
    //    cv::imshow(absl::StrCat("ApriltagDetector Image ", node_name_),
    //               color_image);
    //    cv::waitKey(1);
  }

  const auto corners_offset = builder.fbb()->CreateVector(foxglove_corners);
  foxglove::ImageAnnotations::Builder annotation_builder(*builder.fbb());
  annotation_builder.add_points(corners_offset);
  builder.CheckOk(builder.Send(annotation_builder.Finish()));

  auto map_builder = target_map_sender_.MakeBuilder();
  std::vector<flatbuffers::Offset<frc971::vision::TargetPoseFbs>> target_poses;
  for (auto &detection : results) {
    auto *fbb = map_builder.fbb();
    auto pose = BuildTargetPose(detection, fbb);
    DestroyPose(&detection.pose);
    target_poses.emplace_back(pose);
  }
  const auto target_poses_offset =
      map_builder.fbb()->CreateVector(target_poses);
  auto target_map_builder =
      map_builder.MakeBuilder<frc971::vision::TargetMap>();
  target_map_builder.add_target_poses(target_poses_offset);
  target_map_builder.add_monotonic_timestamp_ns(eof.time_since_epoch().count());
  target_map_builder.add_rejections(rejections_);
  map_builder.CheckOk(map_builder.Send(target_map_builder.Finish()));

  // TODO: Do we need to clean this up?
  // apriltag_detections_destroy(detections);

  end_time = aos::monotonic_clock::now();

  if (FLAGS_debug) {
    timeprofile_display(tag_detector_->tp);
  }

  VLOG(2) << "Took " << chrono::duration<double>(end_time - start_time).count()
          << " seconds to detect overall";

  return;
  // TODO: Need to have proper return here
  //    return {.detections = results, .rejections = rejections_};
}

}  // namespace frc971::apriltag

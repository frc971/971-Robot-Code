#include "frc971/vision/calibrate_multi_cameras_lib.h"

#include <numeric>
#include <string>

#include "aos/configuration.h"
#include "aos/events/logging/log_reader.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/init.h"
#include "aos/util/mcap_logger.h"
#include "frc971/control_loops/pose.h"
#include "frc971/control_loops/quaternion_utils.h"
#include "frc971/vision/calibration_generated.h"
#include "frc971/vision/charuco_lib.h"
#include "frc971/vision/extrinsics_calibration.h"
#include "frc971/vision/target_mapper.h"
#include "frc971/vision/vision_util_lib.h"
#include "frc971/vision/visualize_robot.h"
// clang-format off
// OpenCV eigen files must be included after Eigen includes
#include "opencv2/aruco.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/core/eigen.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc.hpp"
// clang-format on
#include "frc971/constants/constants_sender_lib.h"

ABSL_FLAG(bool, alt_view, false,
          "If true, show visualization from field level, rather than above");
ABSL_FLAG(float, max_pose_error, 5e-5,
          "Throw out target poses with a higher pose error than this");
ABSL_FLAG(std::string, output_folder,
          (std::getenv("TEST_TMPDIR") != nullptr ? std::getenv("TEST_TMPDIR")
                                                 : "/tmp/"),
          "Directory in which to store the updated calibration files");
ABSL_FLAG(std::string, target_type, "charuco_diamond",
          "Type of target being used [aruco, charuco, charuco_diamond]");
ABSL_FLAG(int32_t, team_number, 0,
          "Required: Use the calibration for a node with this team number");
ABSL_FLAG(
    uint64_t, wait_key, 1,
    "Time in ms to wait between images (0 to wait indefinitely until click)");

namespace frc971::vision {
using frc971::vision::PoseUtils;
using frc971::vision::TargetMap;
using frc971::vision::TargetMapper;
using frc971::vision::VisualizeRobot;

std::vector<CameraNode> CreateNodeList() {
  std::vector<CameraNode> list;

  list.push_back({.node_name = "imu", .camera_number = 0});
  list.push_back({.node_name = "imu", .camera_number = 1});
  list.push_back({.node_name = "orin1", .camera_number = 1});
  list.push_back({.node_name = "orin1", .camera_number = 0});

  return list;
}

std::map<std::string, int> CreateOrderingMap(
    std::vector<CameraNode> &node_list) {
  std::map<std::string, int> map;

  for (uint i = 0; i < node_list.size(); i++) {
    map.insert({node_list.at(i).camera_name(), i});
  }

  return map;
}

// Change reference frame from camera to robot
Eigen::Affine3d CameraToRobotDetection(Eigen::Affine3d H_camera_target,
                                       Eigen::Affine3d extrinsics) {
  const Eigen::Affine3d H_robot_camera = extrinsics;
  const Eigen::Affine3d H_robot_target = H_robot_camera * H_camera_target;
  return H_robot_target;
}

Eigen::Affine3d ComputeAveragePose(
    std::vector<TimestampedCameraDetection> &pose_list,
    Eigen::Vector3d *translation_variance, Eigen::Vector3d *rotation_variance) {
  std::vector<Eigen::Vector3d> translation_list;
  std::vector<Eigen::Vector4d> rotation_list;

  for (TimestampedCameraDetection pose : pose_list) {
    translation_list.push_back(pose.H_camera_target.translation());
    Eigen::Quaterniond quat(pose.H_camera_target.rotation().matrix());
    rotation_list.push_back(
        Eigen::Vector4d(quat.x(), quat.y(), quat.z(), quat.w()));
  }
  return frc971::vision::ComputeAveragePose(
      translation_list, rotation_list, translation_variance, rotation_variance);
}

// Do outlier rejection.  Given a list of poses, compute the
// mean and standard deviation, and throw out those more than
// FLAGS_outlier_std_devs standard deviations away from the mean.
// Repeat for the desired number of iterations or until we don't throw
// out any more outliers
void RemoveOutliers(std::vector<TimestampedCameraDetection> &pose_list,
                    int remove_outliers_iterations) {
  for (int i = 1; i <= remove_outliers_iterations; i++) {
    Eigen::Vector3d translation_variance, rotation_variance;
    Eigen::Affine3d avg_pose = ComputeAveragePose(
        pose_list, &translation_variance, &rotation_variance);

    size_t original_size = pose_list.size();
    auto IsPoseOutlier = [&](const auto &pose) {
      Eigen::Affine3d H_delta = avg_pose * pose.H_camera_target.inverse();
      // Compute the z-score for each dimension, and use the max to
      // decide on outliers.  This is similar to the Mahalanobis
      // distance (scale by inverse of the covariance matrix), but we're
      // treating each dimension independently
      Eigen::Matrix3d translation_sigma = translation_variance.asDiagonal();
      Eigen::Matrix3d rotation_sigma = rotation_variance.asDiagonal();
      Eigen::Vector3d delta_rotation =
          PoseUtils::RotationMatrixToEulerAngles(H_delta.rotation().matrix());
      double max_translation_z_score = std::sqrt(
          H_delta.translation()
              .cwiseProduct(translation_sigma.inverse() * H_delta.translation())
              .maxCoeff());
      double max_rotation_z_score = std::sqrt(static_cast<double>(
          (delta_rotation.array() *
           (rotation_sigma.inverse() * delta_rotation).array())
              .maxCoeff()));
      double z_score = std::max(max_translation_z_score, max_rotation_z_score);
      // Remove observations that vary significantly from the mean
      if (z_score > absl::GetFlag(FLAGS_outlier_std_devs)) {
        VLOG(1) << "Removing outlier with z_score " << z_score
                << " relative to std dev = "
                << absl::GetFlag(FLAGS_outlier_std_devs);
        return true;
      }
      return false;
    };
    pose_list.erase(
        std::remove_if(pose_list.begin(), pose_list.end(), IsPoseOutlier),
        pose_list.end());

    VLOG(1) << "Iteration #" << i << ": removed "
            << (original_size - pose_list.size())
            << " outlier constraints out of " << original_size
            << " total\nStd Dev's are: "
            << translation_variance.array().sqrt().transpose() << "m and "
            << rotation_variance.array().sqrt().transpose() * 180.0 / M_PI
            << "deg";
    if (original_size - pose_list.size() == 0) {
      VLOG(1) << "At step " << i
              << ", ending outlier rejection early due to convergence at "
              << pose_list.size() << " elements.\nStd Dev's are: "
              << translation_variance.array().sqrt().transpose() << "m and "
              << rotation_variance.array().sqrt().transpose() * 180.0 / M_PI
              << "deg";
      break;
    }
  }
}

bool PoseIsValid(const TargetMapper::TargetPose &target_pose, double pose_error,
                 double pose_error_ratio, std::string_view camera_name) {
  // Skip detections with invalid ids
  if (static_cast<TargetMapper::TargetId>(target_pose.id) <
          absl::GetFlag(FLAGS_min_target_id) ||
      static_cast<TargetMapper::TargetId>(target_pose.id) >
          absl::GetFlag(FLAGS_max_target_id)) {
    VLOG(1) << "Skipping tag from " << camera_name << " with invalid id of "
            << target_pose.id;
    return false;
  }

  // Skip detections with high pose errors
  if (pose_error > absl::GetFlag(FLAGS_max_pose_error)) {
    LOG(INFO) << "Skipping tag from " << camera_name << " with id "
              << target_pose.id << " due to pose error of " << pose_error;
    return false;
  }
  // Skip detections with high pose error ratios
  if (pose_error_ratio > absl::GetFlag(FLAGS_max_pose_error_ratio)) {
    LOG(INFO) << "Skipping tag from " << camera_name << " with id "
              << target_pose.id << " due to pose error ratio of "
              << pose_error_ratio;
    return false;
  }

  return true;
}

// Take in list of poses from a camera observation and add to running list
// One of two options:
// 1) We see two boards in one view-- store this to get an estimate of
// the offset between the two boards
// 2) We see just one board-- save this and try to pair it with a previous
// observation from another camera
void HandlePoses(
    cv::Mat rgb_image, std::vector<TargetMapper::TargetPose> target_poses,
    aos::distributed_clock::time_point distributed_eof, std::string camera_name,
    TimestampedCameraDetection &last_observation,
    std::vector<std::pair<TimestampedCameraDetection,
                          TimestampedCameraDetection>> &detection_list,
    std::vector<TimestampedCameraDetection> &two_board_extrinsics_list,
    VisualizeRobot &vis_robot_,
    std::map<std::string, cv::Scalar> const &camera_colors,
    std::map<std::string, int> &ordering_map, double image_period_ms,
    int display_count) {
  // This (H_world_board) is used to transform points for visualization
  // Assumes targets are aligned with x->right, y->up, z->out of board
  Eigen::Affine3d H_world_board;
  H_world_board = Eigen::Translation3d::Identity() *
                  Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitX());
  if (absl::GetFlag(FLAGS_alt_view)) {
    // Don't rotate -- this is like viewing from the side
    H_world_board = Eigen::Translation3d(0.0, 0.0, 3.0);
  }

  bool draw_vis = false;
  CHECK_LE(target_poses.size(), 2u)
      << "Can't handle more than two tags in field of view";
  if (target_poses.size() == 2) {
    draw_vis = true;
    VLOG(1) << "Saw two boards in same view from " << camera_name;
    int from_index = 0;
    int to_index = 1;
    // Handle when we see two boards at once
    // We'll store them referenced to the lower id board
    if (target_poses[from_index].id > target_poses[to_index].id) {
      std::swap<int>(from_index, to_index);
    }

    // Create "from" (A) and "to" (B) transforms
    Eigen::Affine3d H_camera_boardA =
        PoseUtils::Pose3dToAffine3d(target_poses[from_index].pose);
    Eigen::Affine3d H_camera_boardB =
        PoseUtils::Pose3dToAffine3d(target_poses[to_index].pose);

    Eigen::Affine3d H_boardA_boardB =
        H_camera_boardA.inverse() * H_camera_boardB;

    TimestampedCameraDetection boardA_boardB{
        .time = distributed_eof,
        .H_camera_target = H_boardA_boardB,
        .camera_name = camera_name,
        .board_id = target_poses[from_index].id};

    VLOG(1) << "Two boards seen by " << camera_name << ".  Map from board "
            << target_poses[from_index].id << " to "
            << target_poses[to_index].id << " is\n"
            << H_boardA_boardB.matrix();
    // Store this observation of the transform between two boards
    two_board_extrinsics_list.push_back(boardA_boardB);

    // Also, store one of the observations, so we can potentially
    // match against the next single-target observation that comes in
    TimestampedCameraDetection new_observation{
        .time = distributed_eof,
        .H_camera_target = H_camera_boardA,
        .camera_name = camera_name,
        .board_id = target_poses[from_index].id};
    last_observation = new_observation;

    if (absl::GetFlag(FLAGS_visualize)) {
      vis_robot_.DrawFrameAxes(
          H_world_board,
          std::string("Board ") + std::to_string(target_poses[from_index].id),
          cv::Scalar(0, 255, 0));
      vis_robot_.DrawFrameAxes(
          H_world_board * boardA_boardB.H_camera_target,
          std::string("Board ") + std::to_string(target_poses[to_index].id),
          cv::Scalar(255, 0, 0));
      vis_robot_.DrawRobotOutline(H_world_board * H_camera_boardA.inverse(),
                                  camera_name, camera_colors.at(camera_name));
    }
  } else if (target_poses.size() == 1) {
    VLOG(1) << camera_name << " saw single board " << target_poses[0].id;
    Eigen::Affine3d H_camera2_board2 =
        PoseUtils::Pose3dToAffine3d(target_poses[0].pose);
    TimestampedCameraDetection new_observation{
        .time = distributed_eof,
        .H_camera_target = H_camera2_board2,
        .camera_name = camera_name,
        .board_id = target_poses[0].id};

    // Only take two observations if they're within half an image cycle of
    // each other (i.e., as close in time as possible).  And, if two
    // consecutive observations are from the same camera, just replace with
    // the newest one
    if ((new_observation.camera_name != last_observation.camera_name) &&
        (std::abs((distributed_eof - last_observation.time).count()) <
         image_period_ms / 2.0 * 1000000.0)) {
      // Sort by camera numbering, since this is how we will handle them
      std::pair<TimestampedCameraDetection, TimestampedCameraDetection>
          new_pair;
      if (ordering_map.at(last_observation.camera_name) <
          ordering_map.at(new_observation.camera_name)) {
        new_pair = std::pair(last_observation, new_observation);
      } else if (ordering_map.at(last_observation.camera_name) >
                 ordering_map.at(new_observation.camera_name)) {
        new_pair = std::pair(new_observation, last_observation);
      }

      detection_list.push_back(new_pair);

      // This bit is just for visualization and checking purposes-- use the
      // first two-board observation to figure out the current estimate
      // between the two cameras (this could be incorrect, but it keeps it
      // constant)
      if (absl::GetFlag(FLAGS_visualize) &&
          two_board_extrinsics_list.size() > 0) {
        draw_vis = true;
        TimestampedCameraDetection &first_two_board_ext =
            two_board_extrinsics_list.front();
        Eigen::Affine3d &H_boardA_boardB = first_two_board_ext.H_camera_target;
        int boardA_id = first_two_board_ext.board_id;

        TimestampedCameraDetection camera1_boardA = new_pair.first;
        TimestampedCameraDetection camera2_boardA = new_pair.second;
        Eigen::Affine3d H_camera1_boardA = camera1_boardA.H_camera_target;
        Eigen::Affine3d H_camera2_boardA = camera2_boardA.H_camera_target;
        // If camera1_boardA doesn't currently point to boardA, then fix it
        if (camera1_boardA.board_id != boardA_id) {
          H_camera1_boardA = H_camera1_boardA * H_boardA_boardB.inverse();
        }
        // If camera2_boardA doesn't currently point to boardA, then fix it
        if (camera2_boardA.board_id != boardA_id) {
          H_camera2_boardA = H_camera2_boardA * H_boardA_boardB.inverse();
        }
        VLOG(1) << "Camera " << camera1_boardA.camera_name << " seeing board "
                << camera1_boardA.board_id << " and camera "
                << camera2_boardA.camera_name << " seeing board "
                << camera2_boardA.board_id;

        // Draw the two poses of the cameras, and the locations of the
        // boards We use "Board A" as the origin (with everything relative
        // to H_world_board)
        vis_robot_.DrawRobotOutline(
            H_world_board * H_camera1_boardA.inverse(),
            camera1_boardA.camera_name,
            camera_colors.at(camera1_boardA.camera_name));
        vis_robot_.DrawRobotOutline(
            H_world_board * H_camera2_boardA.inverse(),
            camera2_boardA.camera_name,
            camera_colors.at(camera2_boardA.camera_name));
        vis_robot_.DrawFrameAxes(
            H_world_board,
            std::string("Board ") +
                std::to_string(first_two_board_ext.board_id),
            cv::Scalar(0, 255, 0));
        vis_robot_.DrawFrameAxes(H_world_board * H_boardA_boardB, "Board B",
                                 cv::Scalar(255, 0, 0));

        VLOG(1) << "Storing observation between " << new_pair.first.camera_name
                << ", target " << new_pair.first.board_id << " and "
                << new_pair.second.camera_name << ", target "
                << new_pair.second.board_id;
      } else if (two_board_extrinsics_list.size() == 0) {
        VLOG(1) << "Not drawing observation yet, since we don't have a two "
                   "board estimate";
      }
    } else {
      if (new_observation.camera_name == last_observation.camera_name) {
        VLOG(2) << "Updating repeated observation for " << camera_name;
      } else {
        VLOG(1) << "Storing observation for " << camera_name << " at time "
                << distributed_eof << " since last observation was "
                << std::abs((distributed_eof - last_observation.time).count()) /
                       1000000.0
                << "ms ago";
      }
      last_observation = new_observation;
    }
  }

  if (absl::GetFlag(FLAGS_visualize)) {
    if (!rgb_image.empty()) {
      std::string image_name = camera_name + " Image";
      cv::Mat rgb_small;
      cv::resize(rgb_image, rgb_small, cv::Size(), 0.5, 0.5);
      cv::imshow(image_name, rgb_small);
      cv::waitKey(absl::GetFlag(FLAGS_wait_key));
    }

    if (draw_vis) {
      cv::putText(vis_robot_.image_,
                  "Display #" + std::to_string(display_count++),
                  cv::Point(600, 10), cv::FONT_HERSHEY_PLAIN, 1.0,
                  cv::Scalar(255, 255, 255));
      cv::imshow("Overhead View", vis_robot_.image_);
      cv::waitKey(absl::GetFlag(FLAGS_wait_key));
      vis_robot_.ClearImage();
    }
  }
}

void HandleTargetMap(
    const TargetMap &map, aos::distributed_clock::time_point distributed_eof,
    std::string camera_name,
    std::map<std::string, aos::distributed_clock::time_point> &last_eofs_debug,
    TimestampedCameraDetection &last_observation,
    std::vector<std::pair<TimestampedCameraDetection,
                          TimestampedCameraDetection>> &detection_list,
    std::vector<TimestampedCameraDetection> &two_board_extrinsics_list,
    VisualizeRobot &vis_robot_,
    std::map<std::string, cv::Scalar> const &camera_colors,
    std::map<std::string, int> &ordering_map, double image_period_ms,
    int display_count) {
  VLOG(1) << "Got april tag map call from camera " << camera_name;
  // Create empty RGB image in this case
  cv::Mat rgb_image;
  std::vector<TargetMapper::TargetPose> target_poses;
  VLOG(1) << ": Diff since last image from " << camera_name << " is "
          << (distributed_eof - last_eofs_debug.at(camera_name)).count() /
                 1000000.0
          << "ms";

  if (last_eofs_debug.find(camera_name) == last_eofs_debug.end()) {
    last_eofs_debug[camera_name] = distributed_eof;
  }

  for (const auto *target_pose_fbs : *map.target_poses()) {
    const TargetMapper::TargetPose target_pose =
        TargetMapper::TargetPoseFromFbs(*target_pose_fbs);

    if (!PoseIsValid(target_pose, target_pose_fbs->pose_error(),
                     target_pose_fbs->pose_error_ratio(), camera_name)) {
      continue;
    }

    target_poses.emplace_back(target_pose);

    Eigen::Affine3d H_camera_target =
        PoseUtils::Pose3dToAffine3d(target_pose.pose);
    VLOG(1) << camera_name << " saw target " << target_pose.id
            << " from TargetMap at timestamp " << distributed_eof
            << " with pose = " << H_camera_target.matrix();
    LOG(INFO) << "pose info for target " << target_pose_fbs->id()
              << ": \nconfidence: " << target_pose_fbs->confidence()
              << ", pose_error: " << target_pose_fbs->pose_error()
              << ", pose_error_ratio: " << target_pose_fbs->pose_error_ratio()
              << ", dist_factor: " << target_pose_fbs->distortion_factor();
  }

  last_eofs_debug[camera_name] = distributed_eof;
  HandlePoses(rgb_image, target_poses, distributed_eof, camera_name,
              last_observation, detection_list, two_board_extrinsics_list,
              vis_robot_, camera_colors, ordering_map, image_period_ms,
              display_count);
}

// TODO<jim>: This currently uses the Charuco lib to extract the target poses,
// but our code running on device uses the GPU-based april tag detection and
// extraction.  We should probably be only using the GPU-based extraction for
// HandleImage(...)
void HandleImage(
    aos::EventLoop *event_loop, cv::Mat rgb_image,
    const aos::monotonic_clock::time_point eof,
    aos::distributed_clock::time_point distributed_eof,
    frc971::vision::CharucoExtractor &charuco_extractor,
    std::string camera_name, TimestampedCameraDetection &last_observation,
    std::vector<std::pair<TimestampedCameraDetection,
                          TimestampedCameraDetection>> &detection_list,
    std::vector<TimestampedCameraDetection> &two_board_extrinsics_list,
    VisualizeRobot &vis_robot_,
    std::map<std::string, cv::Scalar> &camera_colors,
    std::map<std::string, int> &ordering_map, double image_period_ms,
    int display_count) {
  std::vector<cv::Vec4i> charuco_ids;
  std::vector<std::vector<cv::Point2f>> charuco_corners;
  bool valid = false;
  std::vector<Eigen::Vector3d> rvecs_eigen;
  std::vector<Eigen::Vector3d> tvecs_eigen;
  // Why eof vs. distributed_eof?
  charuco_extractor.ProcessImage(rgb_image, eof, event_loop->monotonic_now(),
                                 charuco_ids, charuco_corners, valid,
                                 rvecs_eigen, tvecs_eigen);
  if (rvecs_eigen.size() > 0 && !valid) {
    LOG(WARNING) << "Charuco extractor returned not valid";
    return;
  }

  std::vector<TargetMapper::TargetPose> target_poses;
  for (size_t i = 0; i < tvecs_eigen.size(); i++) {
    Eigen::Quaterniond rotation(
        frc971::controls::ToQuaternionFromRotationVector(rvecs_eigen[i]));
    ceres::examples::Pose3d pose(Eigen::Vector3d(tvecs_eigen[i]), rotation);
    TargetMapper::TargetPose target_pose{charuco_ids[i][0], pose};

    // Since we're using charuco_lib for poses, it doesn't have a pose_error or
    // pose_error_ratio, so just use 0.0 for each here to fake it
    if (!PoseIsValid(target_pose, 0.0, 0.0, camera_name)) {
      continue;
    }

    target_poses.emplace_back(target_pose);

    Eigen::Affine3d H_camera_target = PoseUtils::Pose3dToAffine3d(pose);
    VLOG(2) << camera_name << " saw target " << target_pose.id
            << " from image at timestamp " << distributed_eof
            << " with pose = " << H_camera_target.matrix();
  }
  HandlePoses(rgb_image, target_poses, distributed_eof, camera_name,
              last_observation, detection_list, two_board_extrinsics_list,
              vis_robot_, camera_colors, ordering_map, image_period_ms,
              display_count);
}

void WriteExtrinsicFile(Eigen::Affine3d extrinsic, CameraNode camera_node,
                        const calibration::CameraCalibration *original_cal) {
  // Write out this extrinsic to a file
  flatbuffers::FlatBufferBuilder fbb;
  flatbuffers::Offset<flatbuffers::Vector<float>> data_offset =
      fbb.CreateVector(frc971::vision::MatrixToVector(extrinsic.matrix()));
  calibration::TransformationMatrix::Builder matrix_builder(fbb);
  matrix_builder.add_data(data_offset);
  flatbuffers::Offset<calibration::TransformationMatrix>
      extrinsic_calibration_offset = matrix_builder.Finish();

  calibration::CameraCalibration::Builder calibration_builder(fbb);
  calibration_builder.add_fixed_extrinsics(extrinsic_calibration_offset);
  const aos::realtime_clock::time_point realtime_now =
      aos::realtime_clock::now();
  calibration_builder.add_calibration_timestamp(
      realtime_now.time_since_epoch().count());
  fbb.Finish(calibration_builder.Finish());
  aos::FlatbufferDetachedBuffer<calibration::CameraCalibration>
      solved_extrinsics = fbb.Release();

  aos::FlatbufferDetachedBuffer<frc971::vision::calibration::CameraCalibration>
      cal_copy = aos::RecursiveCopyFlatBuffer(original_cal);
  cal_copy.mutable_message()->clear_fixed_extrinsics();
  cal_copy.mutable_message()->clear_calibration_timestamp();
  aos::FlatbufferDetachedBuffer<calibration::CameraCalibration>
      merged_calibration = aos::MergeFlatBuffers(&cal_copy.message(),
                                                 &solved_extrinsics.message());

  std::stringstream time_ss;
  time_ss << realtime_now;

  const std::string calibration_filename = frc971::vision::CalibrationFilename(
      absl::GetFlag(FLAGS_output_folder), camera_node.node_name,
      absl::GetFlag(FLAGS_team_number), camera_node.camera_number,
      cal_copy.message().camera_id()->data(), time_ss.str());
  LOG(INFO) << calibration_filename << " -> "
            << aos::FlatbufferToJson(merged_calibration, {.multi_line = true});

  aos::util::WriteStringToFileOrDie(
      calibration_filename,
      aos::FlatbufferToJson(merged_calibration, {.multi_line = true}));
}

void ExtrinsicsMain(std::vector<CameraNode> &node_list,
                    std::function<const calibration::CameraCalibration *(
                        aos::EventLoop *const, std::string, int)>
                        find_calibration,
                    std::map<std::string, cv::Scalar> const &camera_colors,
                    aos::logger::LogReader *reader,
                    std::map<std::string, int> &ordering_map,
                    int remove_outliers_iterations, double image_period_ms) {
  static std::map<std::string, aos::distributed_clock::time_point>
      last_eofs_debug;
  int display_count = 1;

  TimestampedCameraDetection last_observation;
  std::vector<std::pair<TimestampedCameraDetection, TimestampedCameraDetection>>
      detection_list;
  std::vector<TimestampedCameraDetection> two_board_extrinsics_list;
  VisualizeRobot vis_robot;

  vis_robot = VisualizeRobot(cv::Size(1000, 1000));
  vis_robot.ClearImage();
  const double kFocalLength = 1000.0;
  const int kImageWidth = 1000;
  vis_robot.SetDefaultViewpoint(kImageWidth, kFocalLength);
  LOG(INFO) << "COPYTHIS, count, camera_name, target_id, timestamp, mag_T, "
               "mag_R_deg, "
               "confidence, pose_error, pose_error_ratio, distortion_factor";

  VLOG(1) << "Using target type " << absl::GetFlag(FLAGS_target_type);

  std::vector<const calibration::CameraCalibration *> calibration_list;

  std::vector<std::unique_ptr<aos::EventLoop>> detection_event_loops;
  std::vector<frc971::vision::CharucoExtractor *> charuco_extractors;
  std::vector<Eigen::Affine3d> default_extrinsics;

  for (const CameraNode &camera_node : node_list) {
    const aos::Node *node = aos::configuration::GetNode(
        reader->configuration(), camera_node.node_name.c_str());

    detection_event_loops.emplace_back(
        reader->event_loop_factory()->MakeEventLoop(
            (camera_node.camera_name() + "_detection").c_str(), node));

    aos::EventLoop *const event_loop = detection_event_loops.back().get();

    // Get the calibration for this orin/camera pair
    const calibration::CameraCalibration *calibration = find_calibration(
        event_loop, camera_node.node_name, camera_node.camera_number);
    calibration_list.push_back(calibration);

    // Extract the extrinsics from the calibration, and save as "defaults"
    cv::Mat extrinsics_cv =
        frc971::vision::CameraExtrinsics(calibration).value();
    Eigen::Matrix4d extrinsics_matrix;
    cv::cv2eigen(extrinsics_cv, extrinsics_matrix);
    const auto ext_H_robot_camera = Eigen::Affine3d(extrinsics_matrix);
    default_extrinsics.emplace_back(ext_H_robot_camera);

    VLOG(1) << "Got extrinsics for " << camera_node.camera_name() << " as\n"
            << default_extrinsics.back().matrix();

    event_loop->MakeWatcher(
        camera_node.camera_name(),
        [&reader, event_loop, camera_node, &last_observation, &detection_list,
         &two_board_extrinsics_list, &vis_robot, &camera_colors, &ordering_map,
         &image_period_ms, display_count](const TargetMap &map) {
          aos::distributed_clock::time_point camera_distributed_time =
              reader->event_loop_factory()
                  ->GetNodeEventLoopFactory(event_loop->node())
                  ->ToDistributedClock(aos::monotonic_clock::time_point(
                      aos::monotonic_clock::duration(
                          map.monotonic_timestamp_ns())));

          HandleTargetMap(map, camera_distributed_time,
                          camera_node.camera_name(), last_eofs_debug,
                          last_observation, detection_list,
                          two_board_extrinsics_list, vis_robot, camera_colors,
                          ordering_map, image_period_ms, display_count);
        });
    VLOG(1) << "Created watcher for using the detection event loop for "
            << camera_node.camera_name();

    // Display images, if they exist
    std::string camera_name = camera_node.camera_name();
    detection_event_loops.back()->MakeWatcher(
        camera_name.c_str(),
        [camera_name](const frc971::vision::CameraImage &image) {
          cv::Mat image_color_mat(cv::Size(image.cols(), image.rows()), CV_8UC2,
                                  (void *)image.data()->data());
          cv::Mat bgr_image(cv::Size(image.cols(), image.rows()), CV_8UC3);
          cv::cvtColor(image_color_mat, bgr_image, cv::COLOR_YUV2BGR_YUYV);
          cv::resize(bgr_image, bgr_image, cv::Size(500, 500));
          cv::imshow(camera_name.c_str(), bgr_image);
          cv::waitKey(1);
        });
  }

  reader->event_loop_factory()->Run();

  CHECK_GT(two_board_extrinsics_list.size(), 0u)
      << "Must have at least one view of both boards";
  int base_target_id = two_board_extrinsics_list[0].board_id;
  VLOG(1) << "Base id for two_board_extrinsics_list is " << base_target_id;

  int pre_outlier_num = two_board_extrinsics_list.size();
  RemoveOutliers(two_board_extrinsics_list, remove_outliers_iterations);

  LOG(INFO) << "Started with " << pre_outlier_num
            << " observations.  After OUTLIER rejection, "
            << two_board_extrinsics_list.size() << " observations remaining";
  Eigen::Affine3d H_boardA_boardB_avg =
      ComputeAveragePose(two_board_extrinsics_list);
  LOG(INFO) << "Estimate of two board pose using all nodes with "
            << two_board_extrinsics_list.size() << " observations is:\n"
            << H_boardA_boardB_avg.matrix() << "\nOr translation of "
            << H_boardA_boardB_avg.translation().transpose()
            << " (m) and rotation (r,p,y) of "
            << PoseUtils::RotationMatrixToEulerAngles(
                   H_boardA_boardB_avg.rotation().matrix())
                       .transpose() *
                   180.0 / M_PI
            << " (deg)";

  // Do quick check to see what averaged two-board pose for
  // each camera is individually, and compare with overall average
  for (auto camera_node : node_list) {
    std::vector<TimestampedCameraDetection> pose_list;
    for (auto ext : two_board_extrinsics_list) {
      CHECK_EQ(base_target_id, ext.board_id)
          << " All boards should have same reference id";
      if (ext.camera_name == camera_node.camera_name()) {
        pose_list.push_back(ext);
      }
    }
    CHECK(pose_list.size() > 0)
        << "Didn't get any two_board extrinsics for camera "
        << camera_node.camera_name();
    Eigen::Vector3d translation_variance, rotation_variance;
    Eigen::Affine3d avg_pose_from_camera = ComputeAveragePose(
        pose_list, &translation_variance, &rotation_variance);

    Eigen::Vector3d translation_std_dev = translation_variance.array().sqrt();
    LOG(INFO) << camera_node.camera_name() << " has average pose from "
              << pose_list.size() << " views of two targets of \n"
              << avg_pose_from_camera.matrix()
              << "\nTranslation standard deviation is "
              << translation_std_dev.transpose();
    double stdev_norm = translation_std_dev.norm();
    double threshold = 0.03;  // 3 cm threshold on translation variation
    if (stdev_norm > threshold) {
      LOG(INFO) << "WARNING: |STD_DEV| is " << stdev_norm * 100 << " > "
                << threshold * 100 << " cm!!!!\nStd dev vector (in m) is "
                << translation_std_dev.transpose();
    }

    Eigen::Vector3d rotation_std_dev = rotation_variance.array().sqrt();
    LOG(INFO) << camera_node.camera_name()
              << " with rotational standard deviation of: "
              << rotation_std_dev.transpose() << " (radians)";
    double rot_stdev_norm = rotation_std_dev.norm();
    double rot_threshold = 3 * M_PI / 180.0;  // Warn if more than 3 degrees
    if (rot_stdev_norm > rot_threshold) {
      LOG(INFO) << "WARNING: ROTATIONAL STD DEV is "
                << rot_stdev_norm * 180.0 / M_PI << " > "
                << rot_threshold * 180.0 / M_PI
                << " degrees!!!!\nStd dev vector (in deg) is "
                << (rotation_std_dev * 180.0 / M_PI).transpose();
    }
    // Check if a particular camera deviates significantly from the overall
    // average Any of these factors could indicate a problem with that camera
    Eigen::Affine3d delta_from_overall =
        H_boardA_boardB_avg * avg_pose_from_camera.inverse();
    LOG(INFO) << camera_node.camera_name()
              << " had estimate different from pooled average of\n"
              << "|dT| = " << delta_from_overall.translation().norm()
              << "m  and |dR| = "
              << (PoseUtils::RotationMatrixToEulerAngles(
                      delta_from_overall.rotation().matrix()) *
                  180.0 / M_PI)
                     .norm()
              << " deg";
  }

  // Next, compute the relative camera poses
  LOG(INFO) << "Got " << detection_list.size() << " extrinsic observations";
  std::vector<TimestampedCameraDetection> H_camera1_camera2_list;
  std::vector<Eigen::Affine3d> updated_extrinsics;
  // Use the first node's extrinsics as our base, and fix from there
  updated_extrinsics.push_back(default_extrinsics[0]);
  LOG(INFO) << "Default extrinsic for camera " << node_list.at(0).camera_name()
            << " is\n"
            << default_extrinsics[0].matrix();
  for (uint i = 0; i < node_list.size() - 1; i++) {
    H_camera1_camera2_list.clear();
    // Go through the list, and find successive pairs of
    // cameras. We'll be calculating and writing the second
    // of the pair's (the i+1'th camera) extrinsics
    for (auto [pose1, pose2] : detection_list) {
      // Note that this assumes our poses are ordered by the node_list
      CHECK(!((pose1.camera_name == node_list.at(i + 1).camera_name()) &&
              (pose2.camera_name == node_list.at(i).camera_name())))
          << "The camera ordering on our detections is incorrect!";
      if ((pose1.camera_name == node_list.at(i).camera_name()) &&
          (pose2.camera_name == node_list.at(i + 1).camera_name())) {
        Eigen::Affine3d H_camera1_boardA = pose1.H_camera_target;
        // If pose1 isn't referenced to base_target_id, correct that
        if (pose1.board_id != base_target_id) {
          // pose1.H_camera_target references boardB, so map back to boardA
          H_camera1_boardA =
              pose1.H_camera_target * H_boardA_boardB_avg.inverse();
        }

        // Now, get the camera2->boardA map (notice it's boardA, same as
        // camera1, so we can compute the difference between the cameras with
        // both referenced to boardA)
        Eigen::Affine3d H_camera2_boardA = pose2.H_camera_target;
        // If pose2 isn't referenced to boardA (base_target_id), correct that
        if (pose2.board_id != base_target_id) {
          // pose2.H_camera_target references boardB, so map back to boardA
          H_camera2_boardA =
              pose2.H_camera_target * H_boardA_boardB_avg.inverse();
        }

        // Compute camera1->camera2 map
        Eigen::Affine3d H_camera1_camera2 =
            H_camera1_boardA * H_camera2_boardA.inverse();
        // Set the list up as TimestampedCameraDetection's to use the outlier
        // removal function
        TimestampedCameraDetection camera1_camera2{
            .H_camera_target = H_camera1_camera2,
            .camera_name = node_list.at(i + 1).camera_name()};
        H_camera1_camera2_list.push_back(camera1_camera2);
        VLOG(1) << "Map from camera " << pose1.camera_name << " and tag "
                << pose1.board_id << " with observation: \n"
                << pose1.H_camera_target.matrix() << "\n to camera "
                << pose2.camera_name << " and tag " << pose2.board_id
                << " with observation: \n"
                << pose2.H_camera_target.matrix() << "\ngot map as\n"
                << H_camera1_camera2.matrix();

        Eigen::Affine3d H_world_board;
        H_world_board = Eigen::Translation3d::Identity() *
                        Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitX());
        if (absl::GetFlag(FLAGS_alt_view)) {
          H_world_board = Eigen::Translation3d(0.0, 0.0, 3.0);
        }

        VLOG(2) << "Camera1 " << pose1.camera_name << " in world frame is \n"
                << (H_world_board * H_camera1_boardA.inverse()).matrix();
        VLOG(2) << "Camera2 " << pose2.camera_name << " in world frame is \n"
                << (H_world_board * H_camera2_boardA.inverse()).matrix();
      }
    }
    // TODO<Jim>: If we don't get any matches, we could just use default
    // extrinsics
    CHECK(H_camera1_camera2_list.size() > 0)
        << "Failed with zero poses for node " << node_list.at(i).camera_name()
        << " and " << node_list.at(i + 1).camera_name();
    if (H_camera1_camera2_list.size() > 0) {
      Eigen::Affine3d H_camera1_camera2_avg =
          ComputeAveragePose(H_camera1_camera2_list);
      LOG(INFO) << "From " << node_list.at(i).camera_name() << " to "
                << node_list.at(i + 1).camera_name() << " found "
                << H_camera1_camera2_list.size()
                << " observations, and the average pose is:\n"
                << H_camera1_camera2_avg.matrix();

      RemoveOutliers(H_camera1_camera2_list, remove_outliers_iterations);

      H_camera1_camera2_avg = ComputeAveragePose(H_camera1_camera2_list);
      LOG(INFO) << "After outlier rejection, from "
                << node_list.at(i).camera_name() << " to "
                << node_list.at(i + 1).camera_name() << " found "
                << H_camera1_camera2_list.size()
                << " observations, and the average pose is:\n"
                << H_camera1_camera2_avg.matrix();

      Eigen::Affine3d H_camera1_camera2_default =
          default_extrinsics[i].inverse() * default_extrinsics[i + 1];
      LOG(INFO) << "Compare this to that from default values:\n"
                << H_camera1_camera2_default.matrix();
      Eigen::Affine3d H_camera1_camera2_diff =
          H_camera1_camera2_avg * H_camera1_camera2_default.inverse();
      LOG(INFO)
          << "Difference between averaged and default delta poses "
             "has |T| = "
          << H_camera1_camera2_diff.translation().norm() << "m and |R| = "
          << Eigen::AngleAxisd(H_camera1_camera2_diff.rotation()).angle()
          << " radians ("
          << Eigen::AngleAxisd(H_camera1_camera2_diff.rotation()).angle() *
                 180.0 / M_PI
          << " degrees)";

      // Next extrinsic is just previous one * avg_delta_pose
      Eigen::Affine3d next_extrinsic =
          updated_extrinsics.back() * H_camera1_camera2_avg;
      updated_extrinsics.push_back(next_extrinsic);
      LOG(INFO) << "Default Extrinsic for " << node_list.at(i + 1).camera_name()
                << " is \n"
                << default_extrinsics[i + 1].matrix();
      LOG(INFO) << "--> Updated Extrinsic for "
                << node_list.at(i + 1).camera_name() << " is \n"
                << next_extrinsic.matrix();

      WriteExtrinsicFile(next_extrinsic, node_list[i + 1],
                         calibration_list[i + 1]);

      if (absl::GetFlag(FLAGS_visualize)) {
        // Draw each of the updated extrinsic camera locations
        vis_robot.SetDefaultViewpoint(1000.0, 1500.0);
        vis_robot.DrawFrameAxes(
            updated_extrinsics.back(), node_list.at(i + 1).camera_name(),
            camera_colors.at(node_list.at(i + 1).camera_name()));
      }
    }
  }
  if (absl::GetFlag(FLAGS_visualize)) {
    // And don't forget to draw the base camera location
    vis_robot.DrawFrameAxes(updated_extrinsics[0],
                            node_list.at(0).camera_name(),
                            camera_colors.at(node_list.at(0).camera_name()));
    cv::imshow("Extrinsic visualization", vis_robot.image_);
    // Add a bit extra time, so that we don't go right through the extrinsics
    cv::waitKey(3000);
    cv::waitKey(0);
  }

  // Cleanup
  for (uint i = 0; i < charuco_extractors.size(); i++) {
    delete charuco_extractors[i];
  }
}
}  // namespace frc971::vision

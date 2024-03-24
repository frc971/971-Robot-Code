#include <numeric>

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
#include "frc971/vision/vision_util_lib.h"
#include "y2024/constants/simulated_constants_sender.h"
#include "y2024/vision/vision_util.h"

DEFINE_bool(alt_view, false,
            "If true, show visualization from field level, rather than above");
DEFINE_string(config, "",
              "If set, override the log's config file with this one.");
DEFINE_string(constants_path, "y2024/constants/constants.json",
              "Path to the constant file");
DEFINE_double(max_pose_error, 5e-5,
              "Throw out target poses with a higher pose error than this");
DEFINE_double(
    max_pose_error_ratio, 0.4,
    "Throw out target poses with a higher pose error ratio than this");
DEFINE_string(output_folder, "/tmp",
              "Directory in which to store the updated calibration files");
DEFINE_string(target_type, "charuco_diamond",
              "Type of target being used [aruco, charuco, charuco_diamond]");
DEFINE_int32(team_number, 0,
             "Required: Use the calibration for a node with this team number");
DEFINE_uint64(
    wait_key, 1,
    "Time in ms to wait between images (0 to wait indefinitely until click)");
DEFINE_bool(robot, false,
            "If true we're calibrating extrinsics for the robot, use the "
            "correct node path for the robot.");

DECLARE_int32(min_target_id);
DECLARE_int32(max_target_id);

// Calibrate extrinsic relationship between cameras using two targets
// seen jointly between cameras.  Uses two types of information: 1)
// when a single camera sees two targets, we estimate the pose between
// targets, and 2) when two separate cameras each see a target, we can
// use the pose between targets to estimate the pose between cameras.

// We then create the extrinsics for the robot by starting with the
// given extrinsic for camera 1 (between imu/robot and camera frames)
// and then map each subsequent camera based on the data collected and
// the extrinsic poses computed here.

// TODO<Jim>: Not currently using estimate from first camera to last camera--
// should do full estimation, and probably also include camera->imu extrinsics
// from all cameras, not just first camera

namespace y2024::vision {
using frc971::vision::DataAdapter;
using frc971::vision::ImageCallback;
using frc971::vision::PoseUtils;
using frc971::vision::TargetMap;
using frc971::vision::TargetMapper;
using frc971::vision::VisualizeRobot;
namespace calibration = frc971::vision::calibration;

static constexpr double kImagePeriodMs =
    1.0 / 60.0 * 1000.0;  // Image capture period in ms

std::vector<CameraNode> node_list(y2024::vision::CreateNodeList());

std::map<std::string, int> ordering_map(
    y2024::vision::CreateOrderingMap(node_list));

// Change reference frame from camera to robot
Eigen::Affine3d CameraToRobotDetection(Eigen::Affine3d H_camera_target,
                                       Eigen::Affine3d extrinsics) {
  const Eigen::Affine3d H_robot_camera = extrinsics;
  const Eigen::Affine3d H_robot_target = H_robot_camera * H_camera_target;
  return H_robot_target;
}

struct TimestampedCameraDetection {
  aos::distributed_clock::time_point time;
  // Pose of target relative to robot
  Eigen::Affine3d H_camera_target;
  // name of pi
  std::string camera_name;
  int board_id;
};

TimestampedCameraDetection last_observation;
std::vector<std::pair<TimestampedCameraDetection, TimestampedCameraDetection>>
    detection_list;
std::vector<TimestampedCameraDetection> two_board_extrinsics_list;
VisualizeRobot vis_robot_;

// TODO<jim>: Implement variance calcs
Eigen::Affine3d ComputeAveragePose(
    std::vector<Eigen::Vector3d> &translation_list,
    std::vector<Eigen::Vector4d> &rotation_list,
    Eigen::Vector3d *translation_variance = nullptr,
    Eigen::Vector3d *rotation_variance = nullptr) {
  Eigen::Vector3d avg_translation =
      std::accumulate(translation_list.begin(), translation_list.end(),
                      Eigen::Vector3d{0, 0, 0}) /
      translation_list.size();

  // TODO<Jim>: Use QuaternionMean from quaternion_utils.cc (but this
  // requires a fixed number of quaternions to be averaged
  Eigen::Vector4d avg_rotation =
      std::accumulate(rotation_list.begin(), rotation_list.end(),
                      Eigen::Vector4d{0, 0, 0, 0}) /
      rotation_list.size();
  // Normalize, so it's a valid quaternion
  avg_rotation = avg_rotation / avg_rotation.norm();
  Eigen::Quaterniond avg_rotation_q{avg_rotation[0], avg_rotation[1],
                                    avg_rotation[2], avg_rotation[3]};
  Eigen::Translation3d translation(avg_translation);
  Eigen::Affine3d return_pose = translation * avg_rotation_q;
  if (translation_variance != nullptr) {
    *translation_variance = Eigen::Vector3d();
  }
  if (rotation_variance != nullptr) {
    *rotation_variance = Eigen::Vector3d();
  }

  return return_pose;
}

Eigen::Affine3d ComputeAveragePose(
    std::vector<Eigen::Affine3d> &pose_list,
    Eigen::Vector3d *translation_variance = nullptr,
    Eigen::Vector3d *rotation_variance = nullptr) {
  std::vector<Eigen::Vector3d> translation_list;
  std::vector<Eigen::Vector4d> rotation_list;

  for (Eigen::Affine3d pose : pose_list) {
    translation_list.push_back(pose.translation());
    Eigen::Quaterniond quat(pose.rotation().matrix());
    rotation_list.push_back(
        Eigen::Vector4d(quat.w(), quat.x(), quat.y(), quat.z()));
  }

  return ComputeAveragePose(translation_list, rotation_list,
                            translation_variance, rotation_variance);
}

Eigen::Affine3d ComputeAveragePose(
    std::vector<TimestampedCameraDetection> &pose_list,
    Eigen::Vector3d *translation_variance = nullptr,
    Eigen::Vector3d *rotation_variance = nullptr) {
  std::vector<Eigen::Vector3d> translation_list;
  std::vector<Eigen::Vector4d> rotation_list;

  for (TimestampedCameraDetection pose : pose_list) {
    translation_list.push_back(pose.H_camera_target.translation());
    Eigen::Quaterniond quat(pose.H_camera_target.rotation().matrix());
    rotation_list.push_back(
        Eigen::Vector4d(quat.w(), quat.x(), quat.y(), quat.z()));
  }
  return ComputeAveragePose(translation_list, rotation_list,
                            translation_variance, rotation_variance);
}

void HandlePoses(cv::Mat rgb_image,
                 std::vector<TargetMapper::TargetPose> target_poses,
                 aos::distributed_clock::time_point distributed_eof,
                 std::string camera_name) {
  // This is used to transform points for visualization
  // Assumes targets are aligned with x->right, y->up, z->out of board
  Eigen::Affine3d H_world_board;
  H_world_board = Eigen::Translation3d::Identity() *
                  Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitX());
  if (FLAGS_alt_view) {
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

    if (FLAGS_visualize) {
      vis_robot_.DrawFrameAxes(
          H_world_board,
          std::string("Board ") + std::to_string(target_poses[from_index].id),
          cv::Scalar(0, 255, 0));
      vis_robot_.DrawFrameAxes(
          H_world_board * boardA_boardB.H_camera_target,
          std::string("Board ") + std::to_string(target_poses[to_index].id),
          cv::Scalar(255, 0, 0));
      vis_robot_.DrawRobotOutline(H_world_board * H_camera_boardA.inverse(),
                                  camera_name, kOrinColors.at(camera_name));
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
    // each other (i.e., as close in time as possible) And, if two consecutive
    // observations are from the same camera, just replace with the newest one
    if ((new_observation.camera_name != last_observation.camera_name) &&
        (std::abs((distributed_eof - last_observation.time).count()) <
         kImagePeriodMs / 2.0 * 1000000.0)) {
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
      // last two-board observation to figure out the current estimate
      // between the two cameras
      if (FLAGS_visualize && two_board_extrinsics_list.size() > 0) {
        draw_vis = true;
        TimestampedCameraDetection &last_two_board_ext =
            two_board_extrinsics_list.back();
        Eigen::Affine3d &H_boardA_boardB = last_two_board_ext.H_camera_target;
        int boardA_boardB_id = last_two_board_ext.board_id;

        TimestampedCameraDetection camera1_boardA = new_pair.first;
        TimestampedCameraDetection camera2_boardB = new_pair.second;
        // If camera1_boardA doesn't point to the first target, then swap
        // these two
        if (camera1_boardA.board_id != boardA_boardB_id) {
          camera1_boardA = new_pair.second;
          camera2_boardB = new_pair.first;
        }
        VLOG(1) << "Camera " << camera1_boardA.camera_name << " seeing board "
                << camera1_boardA.board_id << " and camera "
                << camera2_boardB.camera_name << " seeing board "
                << camera2_boardB.board_id;

        vis_robot_.DrawRobotOutline(
            H_world_board * camera1_boardA.H_camera_target.inverse(),
            camera1_boardA.camera_name,
            kOrinColors.at(camera1_boardA.camera_name));
        vis_robot_.DrawRobotOutline(
            H_world_board * H_boardA_boardB *
                camera2_boardB.H_camera_target.inverse(),
            camera2_boardB.camera_name,
            kOrinColors.at(camera2_boardB.camera_name));
        vis_robot_.DrawFrameAxes(
            H_world_board,
            std::string("Board ") + std::to_string(last_two_board_ext.board_id),
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

  if (FLAGS_visualize) {
    if (!rgb_image.empty()) {
      std::string image_name = camera_name + " Image";
      cv::Mat rgb_small;
      cv::resize(rgb_image, rgb_small, cv::Size(), 0.5, 0.5);
      cv::imshow(image_name, rgb_small);
      cv::waitKey(FLAGS_wait_key);
    }

    if (draw_vis) {
      cv::imshow("Overhead View", vis_robot_.image_);
      cv::waitKey(FLAGS_wait_key);
      vis_robot_.ClearImage();
    }
  }
}

void HandleTargetMap(const TargetMap &map,
                     aos::distributed_clock::time_point distributed_eof,
                     std::string camera_name) {
  VLOG(1) << "Got april tag map call from camera " << camera_name;
  // Create empty RGB image in this case
  cv::Mat rgb_image;
  std::vector<TargetMapper::TargetPose> target_poses;

  for (const auto *target_pose_fbs : *map.target_poses()) {
    // Skip detections with invalid ids
    if (static_cast<TargetMapper::TargetId>(target_pose_fbs->id()) <
            FLAGS_min_target_id ||
        static_cast<TargetMapper::TargetId>(target_pose_fbs->id()) >
            FLAGS_max_target_id) {
      VLOG(1) << "Skipping tag from " << camera_name << " with invalid id of "
              << target_pose_fbs->id();
      continue;
    }

    // Skip detections with high pose errors
    if (target_pose_fbs->pose_error() > FLAGS_max_pose_error) {
      LOG(INFO) << "Skipping tag from " << camera_name << " with id "
                << target_pose_fbs->id() << " due to pose error of "
                << target_pose_fbs->pose_error();
      continue;
    }
    // Skip detections with high pose error ratios
    if (target_pose_fbs->pose_error_ratio() > FLAGS_max_pose_error_ratio) {
      LOG(INFO) << "Skipping tag from " << camera_name << " with id "
                << target_pose_fbs->id() << " due to pose error ratio of "
                << target_pose_fbs->pose_error_ratio();
      continue;
    }

    const TargetMapper::TargetPose target_pose =
        PoseUtils::TargetPoseFromFbs(*target_pose_fbs);

    target_poses.emplace_back(target_pose);

    Eigen::Affine3d H_camera_target =
        PoseUtils::Pose3dToAffine3d(target_pose.pose);
    VLOG(1) << camera_name << " saw target " << target_pose.id
            << " from TargetMap at timestamp " << distributed_eof
            << " with pose = " << H_camera_target.matrix();
  }
  HandlePoses(rgb_image, target_poses, distributed_eof, camera_name);
}

void HandleImage(aos::EventLoop *event_loop, cv::Mat rgb_image,
                 const aos::monotonic_clock::time_point eof,
                 aos::distributed_clock::time_point distributed_eof,
                 frc971::vision::CharucoExtractor &charuco_extractor,
                 std::string camera_name) {
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
    target_poses.emplace_back(target_pose);

    Eigen::Affine3d H_camera_target = PoseUtils::Pose3dToAffine3d(pose);
    VLOG(2) << camera_name << " saw target " << target_pose.id
            << " from image at timestamp " << distributed_eof
            << " with pose = " << H_camera_target.matrix();
  }
  HandlePoses(rgb_image, target_poses, distributed_eof, camera_name);
}

void ExtrinsicsMain(int argc, char *argv[]) {
  vis_robot_ = VisualizeRobot(cv::Size(1000, 1000));
  vis_robot_.ClearImage();
  const double kFocalLength = 1000.0;
  const int kImageWidth = 1000;
  vis_robot_.SetDefaultViewpoint(kImageWidth, kFocalLength);

  std::optional<aos::FlatbufferDetachedBuffer<aos::Configuration>> config =
      (FLAGS_config.empty()
           ? std::nullopt
           : std::make_optional(aos::configuration::ReadConfig(FLAGS_config)));

  // open logfiles
  aos::logger::LogReader reader(
      aos::logger::SortParts(aos::logger::FindLogs(argc, argv)),
      config.has_value() ? &config->message() : nullptr);

  reader.RemapLoggedChannel("/imu/constants", "y2024.Constants");
  reader.RemapLoggedChannel("/orin1/constants", "y2024.Constants");
  if (FLAGS_robot) {
    reader.RemapLoggedChannel("/roborio/constants", "y2024.Constants");
  }
  reader.Register();

  y2024::SendSimulationConstants(reader.event_loop_factory(), FLAGS_team_number,
                                 FLAGS_constants_path);

  VLOG(1) << "Using target type " << FLAGS_target_type;

  std::vector<const calibration::CameraCalibration *> calibration_list;

  std::vector<std::unique_ptr<aos::EventLoop>> detection_event_loops;
  std::vector<frc971::vision::CharucoExtractor *> charuco_extractors;
  std::vector<frc971::vision::ImageCallback *> image_callbacks;
  std::vector<Eigen::Affine3d> default_extrinsics;

  for (const CameraNode &camera_node : node_list) {
    const aos::Node *node = aos::configuration::GetNode(
        reader.configuration(), camera_node.node_name.c_str());

    detection_event_loops.emplace_back(
        reader.event_loop_factory()->MakeEventLoop(
            (camera_node.camera_name() + "_detection").c_str(), node));

    aos::EventLoop *const event_loop = detection_event_loops.back().get();
    frc971::constants::ConstantsFetcher<y2024::Constants> constants_fetcher(
        event_loop);
    // Get the calibration for this orin/camera pair
    const calibration::CameraCalibration *calibration =
        y2024::vision::FindCameraCalibration(constants_fetcher.constants(),
                                             camera_node.node_name,
                                             camera_node.camera_number);
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
        [&reader, event_loop, camera_node](const TargetMap &map) {
          aos::distributed_clock::time_point camera_distributed_time =
              reader.event_loop_factory()
                  ->GetNodeEventLoopFactory(event_loop->node())
                  ->ToDistributedClock(aos::monotonic_clock::time_point(
                      aos::monotonic_clock::duration(
                          map.monotonic_timestamp_ns())));

          HandleTargetMap(map, camera_distributed_time,
                          camera_node.camera_name());
        });
    VLOG(1) << "Created watcher for using the detection event loop for "
            << camera_node.camera_name();
  }

  reader.event_loop_factory()->Run();

  // Do quick check to see what averaged two-board pose for
  // each camera is individually
  CHECK_GT(two_board_extrinsics_list.size(), 0u)
      << "Must have at least one view of both boards";
  int base_target_id = two_board_extrinsics_list[0].board_id;
  VLOG(1) << "Base id for two_board_extrinsics_list is " << base_target_id;

  for (auto camera_node : node_list) {
    std::vector<TimestampedCameraDetection> pose_list;
    for (auto ext : two_board_extrinsics_list) {
      CHECK_EQ(base_target_id, ext.board_id)
          << " All boards should have same reference id";
      if (ext.camera_name == camera_node.camera_name()) {
        pose_list.push_back(ext);
      }
    }
    Eigen::Affine3d avg_pose_from_camera = ComputeAveragePose(pose_list);
    VLOG(1) << "Estimate from " << camera_node.camera_name() << " with "
            << pose_list.size() << " observations is:\n"
            << avg_pose_from_camera.matrix();
  }
  Eigen::Affine3d H_boardA_boardB_avg =
      ComputeAveragePose(two_board_extrinsics_list);
  // TODO: Should probably do some outlier rejection
  VLOG(1) << "Estimate of two board pose using all nodes with "
          << two_board_extrinsics_list.size() << " observations is:\n"
          << H_boardA_boardB_avg.matrix() << "\n";

  // Next, compute the relative camera poses
  LOG(INFO) << "Got " << detection_list.size() << " extrinsic observations";
  std::vector<Eigen::Affine3d> H_camera1_camera2_list;
  std::vector<Eigen::Affine3d> updated_extrinsics;
  // Use the first node's extrinsics as our base, and fix from there
  updated_extrinsics.push_back(default_extrinsics[0]);
  LOG(INFO) << "Default extrinsic for camera " << node_list.at(0).camera_name()
            << " is " << default_extrinsics[0].matrix();
  for (uint i = 0; i < node_list.size() - 1; i++) {
    H_camera1_camera2_list.clear();
    // Go through the list, and find successive pairs of
    // cameras We'll be calculating and writing the second
    // of the pair (the i+1'th camera)
    for (auto [pose1, pose2] : detection_list) {
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
        H_camera1_camera2_list.push_back(H_camera1_camera2);
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
        if (FLAGS_alt_view) {
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

      // Write out this extrinsic to a file
      flatbuffers::FlatBufferBuilder fbb;
      flatbuffers::Offset<flatbuffers::Vector<float>> data_offset =
          fbb.CreateVector(
              frc971::vision::MatrixToVector(next_extrinsic.matrix()));
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

      aos::FlatbufferDetachedBuffer<
          frc971::vision::calibration::CameraCalibration>
          cal_copy = aos::RecursiveCopyFlatBuffer(calibration_list[i + 1]);
      cal_copy.mutable_message()->clear_fixed_extrinsics();
      cal_copy.mutable_message()->clear_calibration_timestamp();
      aos::FlatbufferDetachedBuffer<calibration::CameraCalibration>
          merged_calibration = aos::MergeFlatBuffers(
              &cal_copy.message(), &solved_extrinsics.message());

      std::stringstream time_ss;
      time_ss << realtime_now;

      CameraNode &camera_node = node_list[i + 1];
      const std::string calibration_filename =
          frc971::vision::CalibrationFilename(
              FLAGS_output_folder, camera_node.node_name, FLAGS_team_number,
              camera_node.camera_number, cal_copy.message().camera_id()->data(),
              time_ss.str());
      LOG(INFO) << calibration_filename << " -> "
                << aos::FlatbufferToJson(merged_calibration,
                                         {.multi_line = true});

      aos::util::WriteStringToFileOrDie(
          calibration_filename,
          aos::FlatbufferToJson(merged_calibration, {.multi_line = true}));

      if (FLAGS_visualize) {
        // Draw each of the updated extrinsic camera locations
        vis_robot_.SetDefaultViewpoint(1000.0, 1500.0);
        vis_robot_.DrawFrameAxes(
            updated_extrinsics.back(), node_list.at(i + 1).camera_name(),
            kOrinColors.at(node_list.at(i + 1).camera_name()));
      }
    }
  }
  if (FLAGS_visualize) {
    // And don't forget to draw the base camera location
    vis_robot_.DrawFrameAxes(updated_extrinsics[0],
                             node_list.at(0).camera_name(),
                             kOrinColors.at(node_list.at(0).camera_name()));
    cv::imshow("Extrinsic visualization", vis_robot_.image_);
    // Add a bit extra time, so that we don't go right through the extrinsics
    cv::waitKey(3000);
    cv::waitKey(0);
  }

  // Cleanup
  for (uint i = 0; i < image_callbacks.size(); i++) {
    delete charuco_extractors[i];
    delete image_callbacks[i];
  }
}
}  // namespace y2024::vision

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  y2024::vision::ExtrinsicsMain(argc, argv);
}

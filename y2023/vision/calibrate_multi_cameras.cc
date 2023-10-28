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
#include "frc971/vision/target_mapper.h"
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
#include "y2023/constants/simulated_constants_sender.h"
#include "y2023/vision/aprilrobotics.h"
#include "y2023/vision/vision_util.h"

DEFINE_bool(alt_view, false,
            "If true, show visualization from field level, rather than above");
DEFINE_string(config, "",
              "If set, override the log's config file with this one.");
DEFINE_string(constants_path, "y2023/constants/constants.json",
              "Path to the constant file");
DEFINE_string(target_type, "charuco_diamond",
              "Type of target being used [aruco, charuco, charuco_diamond]");
DEFINE_int32(team_number, 0,
             "Required: Use the calibration for a node with this team number");
DEFINE_uint64(
    wait_key, 1,
    "Time in ms to wait between images (0 to wait indefinitely until click)");

// Calibrate extrinsic relationship between cameras using two targets
// seen jointly between cameras.  Uses two types of information: 1)
// when a single camera sees two targets, we estimate the pose between
// targets, and 2) when two separate cameras each see a target, we can
// use the pose between targets to estimate the pose between cameras.

// We then create the extrinsics for the robot by starting with the
// given extrinsic for camera 1 (between imu/robot and camera frames)
// and then map each subsequent camera based on the data collected and
// the extrinsic poses computed here.

// TODO<Jim>: Should export a new extrinsic file for each camera
// TODO<Jim>: Not currently using estimate from pi1->pi4-- should do full
// estimation, and probably also include camera->imu extrinsics from all
// cameras, not just pi1

namespace y2023 {
namespace vision {
using frc971::vision::DataAdapter;
using frc971::vision::ImageCallback;
using frc971::vision::PoseUtils;
using frc971::vision::TargetMap;
using frc971::vision::TargetMapper;
using frc971::vision::VisualizeRobot;
namespace calibration = frc971::vision::calibration;

static constexpr double kImagePeriodMs =
    1.0 / 30.0 * 1000.0;  // Image capture period in ms

// Change reference frame from camera to robot
Eigen::Affine3d CameraToRobotDetection(Eigen::Affine3d H_camera_target,
                                       Eigen::Affine3d extrinsics) {
  const Eigen::Affine3d H_robot_camera = extrinsics;
  const Eigen::Affine3d H_robot_target = H_robot_camera * H_camera_target;
  return H_robot_target;
}

struct TimestampedPiDetection {
  aos::distributed_clock::time_point time;
  // Pose of target relative to robot
  Eigen::Affine3d H_camera_target;
  // name of pi
  std::string pi_name;
  int board_id;
};

TimestampedPiDetection last_observation;
std::vector<std::pair<TimestampedPiDetection, TimestampedPiDetection>>
    detection_list;
std::vector<TimestampedPiDetection> two_board_extrinsics_list;
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
    LOG(INFO) << *rotation_variance;
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
    std::vector<TimestampedPiDetection> &pose_list,
    Eigen::Vector3d *translation_variance = nullptr,
    Eigen::Vector3d *rotation_variance = nullptr) {
  std::vector<Eigen::Vector3d> translation_list;
  std::vector<Eigen::Vector4d> rotation_list;

  for (TimestampedPiDetection pose : pose_list) {
    translation_list.push_back(pose.H_camera_target.translation());
    Eigen::Quaterniond quat(pose.H_camera_target.rotation().matrix());
    rotation_list.push_back(
        Eigen::Vector4d(quat.w(), quat.x(), quat.y(), quat.z()));
  }
  return ComputeAveragePose(translation_list, rotation_list,
                            translation_variance, rotation_variance);
}

void ProcessImage(aos::EventLoop *event_loop, cv::Mat rgb_image,
                  const aos::monotonic_clock::time_point eof,
                  aos::distributed_clock::time_point distributed_eof,
                  frc971::vision::CharucoExtractor &charuco_extractor,
                  std::string pi_name) {
  std::vector<cv::Vec4i> charuco_ids;
  std::vector<std::vector<cv::Point2f>> charuco_corners;
  bool valid = false;
  std::vector<Eigen::Vector3d> rvecs_eigen;
  std::vector<Eigen::Vector3d> tvecs_eigen;
  charuco_extractor.ProcessImage(rgb_image, eof, event_loop->monotonic_now(),
                                 charuco_ids, charuco_corners, valid,
                                 rvecs_eigen, tvecs_eigen);

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
  if (valid) {
    CHECK_LE(tvecs_eigen.size(), 2u)
        << "Can't handle more than two tags in field of view";
    if (tvecs_eigen.size() == 2) {
      draw_vis = true;
      VLOG(2) << "Saw two boards in same view from " << pi_name;
      // Handle when we see two boards at once
      // We'll store them referenced to the lower id board
      int from_index = 0;
      int to_index = 1;
      if (charuco_ids[from_index][0] > charuco_ids[to_index][0]) {
        std::swap<int>(from_index, to_index);
      }

      // Create "from" (A) and "to" (B) transforms
      Eigen::Quaternion<double> rotationA(
          frc971::controls::ToQuaternionFromRotationVector(
              rvecs_eigen[from_index]));
      Eigen::Translation3d translationA(tvecs_eigen[from_index]);
      Eigen::Affine3d H_camera_boardA = translationA * rotationA;

      Eigen::Quaternion<double> rotationB(
          frc971::controls::ToQuaternionFromRotationVector(
              rvecs_eigen[to_index]));
      Eigen::Translation3d translationB(tvecs_eigen[to_index]);
      Eigen::Affine3d H_camera_boardB = translationB * rotationB;

      Eigen::Affine3d H_boardA_boardB =
          H_camera_boardA.inverse() * H_camera_boardB;

      TimestampedPiDetection boardA_boardB{
          .time = distributed_eof,
          .H_camera_target = H_boardA_boardB,
          .pi_name = pi_name,
          .board_id = charuco_ids[from_index][0]};

      VLOG(1) << "Map from board " << from_index << " to " << to_index
              << " is\n"
              << H_boardA_boardB.matrix();
      // Store this observation of the transform between two boards
      two_board_extrinsics_list.push_back(boardA_boardB);

      if (FLAGS_visualize) {
        vis_robot_.DrawFrameAxes(
            H_world_board,
            std::string("board ") + std::to_string(charuco_ids[from_index][0]),
            cv::Scalar(0, 255, 0));
        vis_robot_.DrawFrameAxes(
            H_world_board * boardA_boardB.H_camera_target,
            std::string("board ") + std::to_string(charuco_ids[to_index][0]),
            cv::Scalar(255, 0, 0));
        VLOG(2) << "Detection map from camera " << pi_name << " to board "
                << charuco_ids[from_index][0] << " is\n"
                << H_camera_boardA.matrix() << "\n and inverse is\n"
                << H_camera_boardA.inverse().matrix()
                << "\n and with world to board rotation is\n"
                << H_world_board * H_camera_boardA.inverse().matrix();
        vis_robot_.DrawRobotOutline(H_world_board * H_camera_boardA.inverse(),
                                    pi_name, cv::Scalar(0, 0, 255));
      }

    } else {
      VLOG(1) << "Saw single board in camera " << pi_name;
      Eigen::Quaternion<double> rotation(
          frc971::controls::ToQuaternionFromRotationVector(rvecs_eigen[0]));
      Eigen::Translation3d translation(tvecs_eigen[0]);
      Eigen::Affine3d H_camera2_board2 = translation * rotation;
      TimestampedPiDetection new_observation{
          .time = distributed_eof,
          .H_camera_target = H_camera2_board2,
          .pi_name = pi_name,
          .board_id = charuco_ids[0][0]};

      VLOG(2) << "Checking versus last result from " << last_observation.pi_name
              << " at time " << last_observation.time << " with delta time = "
              << std::abs((distributed_eof - last_observation.time).count());
      // Now, check if this new observation can be paired with a
      // previous single board observation.

      // Only take two observations if they're near enough to each
      // other in time.  This should be within +/- kImagePeriodMs of
      // each other (e.g., +/-16.666ms for 30 Hz capture).  This
      // should make sure we're always getting the closest images, and
      // not miss too many possible pairs, between cameras

      // TODO<Jim>: Should also check that (rotational) velocity of the robot is
      // small
      if (std::abs((distributed_eof - last_observation.time).count()) <
          static_cast<int>(kImagePeriodMs / 2.0 * 1000000)) {
        // Sort by pi numbering, since this is how we will handle them
        std::pair<TimestampedPiDetection, TimestampedPiDetection> new_pair;
        if (last_observation.pi_name < new_observation.pi_name) {
          new_pair = std::pair(last_observation, new_observation);
        } else if (last_observation.pi_name > new_observation.pi_name) {
          new_pair = std::pair(new_observation, last_observation);
        } else {
          LOG(WARNING) << "Got 2 observations in a row from same pi. Probably "
                          "not too much of an issue???";
        }
        detection_list.push_back(new_pair);

        // This bit is just for visualization and checking purposes-- use the
        // last two-board observation to figure out the current estimate between
        // the two cameras
        if (FLAGS_visualize && two_board_extrinsics_list.size() > 0) {
          draw_vis = true;
          TimestampedPiDetection &last_two_board_ext =
              two_board_extrinsics_list[two_board_extrinsics_list.size() - 1];
          Eigen::Affine3d &H_boardA_boardB = last_two_board_ext.H_camera_target;
          int boardA_boardB_id = last_two_board_ext.board_id;

          TimestampedPiDetection camera1_boardA = new_pair.first;
          TimestampedPiDetection camera2_boardB = new_pair.second;
          // If camera1_boardA doesn't point to the first target, then swap
          // these two
          if (camera1_boardA.board_id != boardA_boardB_id) {
            camera1_boardA = new_pair.second;
            camera2_boardB = new_pair.first;
          }
          VLOG(1) << "Camera " << camera1_boardA.pi_name << " seeing board "
                  << camera1_boardA.board_id << " and camera "
                  << camera2_boardB.pi_name << " seeing board "
                  << camera2_boardB.board_id;

          vis_robot_.DrawRobotOutline(
              H_world_board * camera1_boardA.H_camera_target.inverse(),
              camera1_boardA.pi_name, cv::Scalar(0, 0, 255));
          vis_robot_.DrawRobotOutline(
              H_world_board * H_boardA_boardB *
                  camera2_boardB.H_camera_target.inverse(),
              camera2_boardB.pi_name, cv::Scalar(128, 128, 0));
          vis_robot_.DrawFrameAxes(
              H_world_board,
              std::string("Board ") +
                  std::to_string(last_two_board_ext.board_id),
              cv::Scalar(0, 255, 0));
          vis_robot_.DrawFrameAxes(H_world_board * H_boardA_boardB, "Board B",
                                   cv::Scalar(255, 0, 0));
          Eigen::Affine3d H_camera1_camera2 =
              camera1_boardA.H_camera_target * H_boardA_boardB *
              camera2_boardB.H_camera_target.inverse();

          VLOG(1) << "Storing observation between " << new_pair.first.pi_name
                  << ", target " << new_pair.first.board_id << " and "
                  << new_pair.second.pi_name << ", target "
                  << new_pair.second.board_id
                  << " and camera-to-camera offset:\n"
                  << H_camera1_camera2.matrix();
        }
      } else {
        VLOG(2) << "Storing observation for " << pi_name << " at time "
                << distributed_eof;
        last_observation = new_observation;
      }
    }
  }

  if (FLAGS_visualize) {
    std::string image_name = pi_name + " Image";
    cv::Mat rgb_small;
    cv::resize(rgb_image, rgb_small, cv::Size(), 0.5, 0.5);
    cv::imshow(image_name, rgb_small);
    cv::waitKey(FLAGS_wait_key);

    if (draw_vis) {
      cv::imshow("View", vis_robot_.image_);
      cv::waitKey(1);
      vis_robot_.ClearImage();
    }
  }
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

  constexpr size_t kNumPis = 4;
  for (size_t i = 1; i <= kNumPis; i++) {
    reader.RemapLoggedChannel(absl::StrFormat("/pi%u/camera", i),
                              "frc971.vision.TargetMap");
    reader.RemapLoggedChannel(absl::StrFormat("/pi%u/camera", i),
                              "foxglove.ImageAnnotations");
    reader.RemapLoggedChannel(absl::StrFormat("/pi%u/constants", i),
                              "y2023.Constants");
  }
  reader.RemapLoggedChannel("/imu/constants", "y2023.Constants");
  reader.Register();

  SendSimulationConstants(reader.event_loop_factory(), FLAGS_team_number,
                          FLAGS_constants_path);

  VLOG(1) << "Using target type " << FLAGS_target_type;
  std::vector<std::string> node_list;
  node_list.push_back("pi1");
  node_list.push_back("pi2");
  node_list.push_back("pi3");
  node_list.push_back("pi4");

  std::vector<std::unique_ptr<aos::EventLoop>> detection_event_loops;
  std::vector<frc971::vision::CharucoExtractor *> charuco_extractors;
  std::vector<frc971::vision::ImageCallback *> image_callbacks;
  std::vector<Eigen::Affine3d> default_extrinsics;

  for (uint i = 0; i < node_list.size(); i++) {
    std::string node = node_list[i];
    const aos::Node *pi =
        aos::configuration::GetNode(reader.configuration(), node.c_str());

    detection_event_loops.emplace_back(
        reader.event_loop_factory()->MakeEventLoop(
            (node + "_detection").c_str(), pi));

    frc971::constants::ConstantsFetcher<y2023::Constants> constants_fetcher(
        detection_event_loops.back().get());

    const calibration::CameraCalibration *calibration =
        FindCameraCalibration(constants_fetcher.constants(), node);

    frc971::vision::TargetType target_type =
        frc971::vision::TargetTypeFromString(FLAGS_target_type);
    frc971::vision::CharucoExtractor *charuco_ext =
        new frc971::vision::CharucoExtractor(calibration, target_type);
    charuco_extractors.emplace_back(charuco_ext);

    cv::Mat extrinsics_cv = CameraExtrinsics(calibration).value();
    Eigen::Matrix4d extrinsics_matrix;
    cv::cv2eigen(extrinsics_cv, extrinsics_matrix);
    const auto ext_H_robot_pi = Eigen::Affine3d(extrinsics_matrix);
    default_extrinsics.emplace_back(ext_H_robot_pi);

    VLOG(1) << "Got extrinsics for " << node << " as\n"
            << default_extrinsics.back().matrix();

    frc971::vision::ImageCallback *image_callback =
        new frc971::vision::ImageCallback(
            detection_event_loops[i].get(), "/" + node_list[i] + "/camera",
            [&reader, &charuco_extractors, &detection_event_loops, &node_list,
             i](cv::Mat rgb_image, const aos::monotonic_clock::time_point eof) {
              aos::distributed_clock::time_point pi_distributed_time =
                  reader.event_loop_factory()
                      ->GetNodeEventLoopFactory(
                          detection_event_loops[i].get()->node())
                      ->ToDistributedClock(eof);
              ProcessImage(detection_event_loops[i].get(), rgb_image, eof,
                           pi_distributed_time, *charuco_extractors[i],
                           node_list[i]);
            });

    image_callbacks.emplace_back(image_callback);
  }

  reader.event_loop_factory()->Run();

  // Do quick check to see what averaged two-board pose for each pi is
  // individually
  CHECK_GT(two_board_extrinsics_list.size(), 0u)
      << "Must have at least one view of both boards";
  int base_target_id = two_board_extrinsics_list[0].board_id;
  VLOG(1) << "Base id for two_board_extrinsics_list is " << base_target_id;
  for (auto node : node_list) {
    std::vector<TimestampedPiDetection> pose_list;
    for (auto ext : two_board_extrinsics_list) {
      CHECK_EQ(base_target_id, ext.board_id)
          << " All boards should have same reference id";
      if (ext.pi_name == node) {
        pose_list.push_back(ext);
      }
    }
    Eigen::Affine3d avg_pose_from_pi = ComputeAveragePose(pose_list);
    VLOG(1) << "Estimate from " << node << " with " << pose_list.size()
            << " observations is:\n"
            << avg_pose_from_pi.matrix();
  }
  Eigen::Affine3d H_boardA_boardB_avg =
      ComputeAveragePose(two_board_extrinsics_list);
  // TODO: Should probably do some outlier rejection
  LOG(INFO) << "Estimate of two board pose using all nodes with "
            << two_board_extrinsics_list.size() << " observations is:\n"
            << H_boardA_boardB_avg.matrix() << "\n";

  // Next, compute the relative camera poses
  LOG(INFO) << "Got " << detection_list.size() << " extrinsic observations";
  std::vector<Eigen::Affine3d> H_camera1_camera2_list;
  std::vector<Eigen::Affine3d> updated_extrinsics;
  // Use the first node's extrinsics as our base, and fix from there
  updated_extrinsics.push_back(default_extrinsics[0]);
  LOG(INFO) << "Default extrinsic for node " << node_list[0] << " is "
            << default_extrinsics[0].matrix();
  for (uint i = 0; i < node_list.size() - 1; i++) {
    H_camera1_camera2_list.clear();
    // Go through the list, and find successive pairs of cameras
    for (auto [pose1, pose2] : detection_list) {
      if ((pose1.pi_name == node_list[i]) &&
          (pose2.pi_name == node_list[i + 1])) {
        Eigen::Affine3d H_camera1_boardA = pose1.H_camera_target;
        // If pose1 isn't referenced to base_target_id, correct that
        if (pose1.board_id != base_target_id) {
          // pose1.H_camera_target references boardB, so map back to boardA
          H_camera1_boardA =
              pose1.H_camera_target * H_boardA_boardB_avg.inverse();
        }

        // Now, get the camera2->boardA map (notice it's boardA, same as
        // camera1, so we can compute the difference based both on boardA)
        Eigen::Affine3d H_camera2_boardA = pose2.H_camera_target;
        // If pose2 isn't referenced to boardA (base_target_id), correct that
        if (pose2.board_id != base_target_id) {
          // pose2.H_camera_target references boardB, so map back to boardA
          H_camera2_boardA =
              pose1.H_camera_target * H_boardA_boardB_avg.inverse();
        }

        // Compute camera1->camera2 map
        Eigen::Affine3d H_camera1_camera2 =
            H_camera1_boardA * H_camera2_boardA.inverse();
        H_camera1_camera2_list.push_back(H_camera1_camera2);
        VLOG(1) << "Map from camera " << pose1.pi_name << " and tag "
                << pose1.board_id << " with observation: \n"
                << pose1.H_camera_target.matrix() << "\n to camera "
                << pose2.pi_name << " and tag " << pose2.board_id
                << " with observation: \n"
                << pose2.H_camera_target.matrix() << "\ngot map as\n"
                << H_camera1_camera2.matrix();

        Eigen::Affine3d H_world_board;
        H_world_board = Eigen::Translation3d::Identity() *
                        Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitX());
        if (FLAGS_alt_view) {
          H_world_board = Eigen::Translation3d(0.0, 0.0, 3.0);
        }

        VLOG(2) << "Camera1 " << pose1.pi_name << " in world frame is \n"
                << (H_world_board * H_camera1_boardA.inverse()).matrix();
        VLOG(2) << "Camera2 " << pose2.pi_name << " in world frame is \n"
                << (H_world_board * H_camera2_boardA.inverse()).matrix();
      }
    }
    // TODO<Jim>: If we don't get any matches, we could just use default
    // extrinsics
    CHECK(H_camera1_camera2_list.size() > 0)
        << "Failed with zero poses for node " << node_list[i];
    if (H_camera1_camera2_list.size() > 0) {
      Eigen::Affine3d H_camera1_camera2_avg =
          ComputeAveragePose(H_camera1_camera2_list);
      LOG(INFO) << "From " << node_list[i] << " to " << node_list[i + 1]
                << " found " << H_camera1_camera2_list.size()
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
      LOG(INFO) << "Default Extrinsic for " << node_list[i + 1] << " is \n"
                << default_extrinsics[i + 1].matrix();
      LOG(INFO) << "--> Updated Extrinsic for " << node_list[i + 1] << " is \n"
                << next_extrinsic.matrix();
    }
  }

  // Cleanup
  for (uint i = 0; i < image_callbacks.size(); i++) {
    delete charuco_extractors[i];
    delete image_callbacks[i];
  }
}
}  // namespace vision
}  // namespace y2023

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  y2023::vision::ExtrinsicsMain(argc, argv);
}

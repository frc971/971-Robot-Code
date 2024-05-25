#include <string>

#include "Eigen/Dense"
#include "opencv2/aruco.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/core/eigen.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include "aos/configuration.h"
#include "aos/events/logging/log_reader.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/init.h"
#include "aos/util/mcap_logger.h"
#include "frc971/constants/constants_sender_lib.h"
#include "frc971/control_loops/pose.h"
#include "frc971/vision/calibration_generated.h"
#include "frc971/vision/charuco_lib.h"
#include "frc971/vision/target_mapper.h"
#include "frc971/vision/vision_generated.h"
#include "frc971/vision/vision_util_lib.h"
#include "frc971/vision/visualize_robot.h"
#include "y2024_swerve/constants/simulated_constants_sender.h"
#include "y2024_swerve/vision/vision_util.h"

DEFINE_string(config, "",
              "If set, override the log's config file with this one.");
DEFINE_string(constants_path, "y2024_swerve/constants/constants.json",
              "Path to the constant file");
DEFINE_string(dump_constraints_to, "/tmp/mapping_constraints.txt",
              "Write the target constraints to this path");
DEFINE_string(dump_stats_to, "/tmp/mapping_stats.txt",
              "Write the mapping stats to this path");
DEFINE_string(field_name, "crescendo",
              "Field name, for the output json filename and flatbuffer field");
DEFINE_string(json_path, "y2024_swerve/vision/maps/target_map.json",
              "Specify path for json with initial pose guesses.");
DEFINE_double(max_pose_error, 1e-6,
              "Throw out target poses with a higher pose error than this");
DEFINE_double(
    max_pose_error_ratio, 0.4,
    "Throw out target poses with a higher pose error ratio than this");
DEFINE_string(mcap_output_path, "", "Log to output.");
DEFINE_string(output_dir, "y2024_swerve/vision/maps",
              "Directory to write solved target map to");
DEFINE_double(pause_on_distance, 2.0,
              "Pause if two consecutive implied robot positions differ by more "
              "than this many meters");
DEFINE_string(orin, "orin1",
              "Orin name to generate mcap log for; defaults to orin1.");
DEFINE_uint64(skip_to, 1,
              "Start at combined image of this number (1 is the first image)");
DEFINE_bool(solve, true, "Whether to solve for the field's target map.");
DEFINE_bool(split_field, false,
            "Whether to break solve into two sides of field");
DEFINE_int32(team_number, 0,
             "Required: Use the calibration for a node with this team number");
DEFINE_uint64(wait_key, 1,
              "Time in ms to wait between images, if no click (0 to wait "
              "indefinitely until click).");

DECLARE_int32(frozen_target_id);
DECLARE_int32(min_target_id);
DECLARE_int32(max_target_id);
DECLARE_bool(visualize_solver);

namespace y2024_swerve::vision {
using frc971::vision::DataAdapter;
using frc971::vision::ImageCallback;
using frc971::vision::PoseUtils;
using frc971::vision::TargetMap;
using frc971::vision::TargetMapper;
using frc971::vision::VisualizeRobot;
namespace calibration = frc971::vision::calibration;

// Class to handle reading target poses from a replayed log,
// displaying various debug info, and passing the poses to
// frc971::vision::TargetMapper for field mapping.
class TargetMapperReplay {
 public:
  TargetMapperReplay(aos::logger::LogReader *reader);

  // Solves for the target poses with the accumulated detections if FLAGS_solve.
  void MaybeSolve();

 private:
  static constexpr int kImageWidth = 1280;

  // Contains fixed target poses without solving, for use with visualization
  static const TargetMapper kFixedTargetMapper;

  // Map of TargetId to alliance "color" for splitting field
  static std::map<uint, std::string> kIdAllianceMap;

  // Change reference frame from camera to robot
  static Eigen::Affine3d CameraToRobotDetection(Eigen::Affine3d H_camera_target,
                                                Eigen::Affine3d extrinsics);

  // Adds april tag detections into the detection list, and handles
  // visualization
  void HandleAprilTags(const TargetMap &map,
                       aos::distributed_clock::time_point node_distributed_time,
                       std::string camera_name, Eigen::Affine3d extrinsics);
  // Gets images from the given pi and passes apriltag positions to
  // HandleAprilTags()
  void HandleNodeCaptures(
      aos::EventLoop *mapping_event_loop,
      frc971::constants::ConstantsFetcher<y2024_swerve::Constants>
          *constants_fetcher,
      int camera_number);

  aos::logger::LogReader *reader_;
  // April tag detections from all pis
  std::vector<DataAdapter::TimestampedDetection> timestamped_target_detections_;

  VisualizeRobot vis_robot_;
  // Set of camera names which are currently drawn on the display
  std::set<std::string> drawn_cameras_;
  // Number of frames displayed
  size_t display_count_;
  // Last time we drew onto the display image.
  // This is different from when we actually call imshow() to update
  // the display window
  aos::distributed_clock::time_point last_draw_time_;

  Eigen::Affine3d last_H_world_robot_;
  // Maximum distance between consecutive T_world_robot's in one display frame,
  // used to determine if we need to pause for the user to see this frame
  // clearly
  double max_delta_T_world_robot_;
  double ignore_count_;

  std::vector<std::unique_ptr<aos::EventLoop>> mapping_event_loops_;

  std::unique_ptr<aos::EventLoop> mcap_event_loop_;
  std::unique_ptr<aos::McapLogger> relogger_;
};

std::vector<CameraNode> node_list(y2024_swerve::vision::CreateNodeList());

std::map<std::string, int> camera_ordering_map(
    y2024_swerve::vision::CreateOrderingMap(node_list));

std::map<uint, std::string> TargetMapperReplay::kIdAllianceMap = {
    {1, "red"},  {2, "red"},   {3, "red"},   {4, "red"},
    {5, "red"},  {6, "blue"},  {7, "blue"},  {8, "blue"},
    {9, "blue"}, {10, "blue"}, {11, "red"},  {12, "red"},
    {13, "red"}, {14, "blue"}, {15, "blue"}, {16, "blue"}};

const auto TargetMapperReplay::kFixedTargetMapper =
    TargetMapper(FLAGS_json_path, ceres::examples::VectorOfConstraints{});

Eigen::Affine3d TargetMapperReplay::CameraToRobotDetection(
    Eigen::Affine3d H_camera_target, Eigen::Affine3d extrinsics) {
  const Eigen::Affine3d H_robot_camera = extrinsics;
  const Eigen::Affine3d H_robot_target = H_robot_camera * H_camera_target;
  return H_robot_target;
}

TargetMapperReplay::TargetMapperReplay(aos::logger::LogReader *reader)
    : reader_(reader),
      timestamped_target_detections_(),
      vis_robot_(cv::Size(1280, 1000)),
      drawn_cameras_(),
      display_count_(0),
      last_draw_time_(aos::distributed_clock::min_time),
      last_H_world_robot_(Eigen::Matrix4d::Identity()),
      max_delta_T_world_robot_(0.0) {
  reader_->RemapLoggedChannel("/orin1/constants", "y2024_swerve.Constants");
  reader_->RemapLoggedChannel("/imu/constants", "y2024_swerve.Constants");
  // If it's Box of Orins, don't remap roborio constants
  reader_->MaybeRemapLoggedChannel<Constants>("/roborio/constants");
  reader_->Register();

  SendSimulationConstants(reader_->event_loop_factory(), FLAGS_team_number,
                          FLAGS_constants_path);

  if (FLAGS_visualize_solver) {
    vis_robot_.ClearImage();
    // Set focal length to zoomed in, to view extrinsics
    const double kFocalLength = 1500.0;
    vis_robot_.SetDefaultViewpoint(kImageWidth, kFocalLength);
  }

  for (const CameraNode &camera_node : node_list) {
    const aos::Node *node = aos::configuration::GetNode(
        reader_->configuration(), camera_node.node_name.c_str());

    mapping_event_loops_.emplace_back(
        reader_->event_loop_factory()->MakeEventLoop(
            camera_node.node_name + "mapping", node));

    frc971::constants::ConstantsFetcher<y2024_swerve::Constants>
        constants_fetcher(
            mapping_event_loops_[mapping_event_loops_.size() - 1].get());
    HandleNodeCaptures(
        mapping_event_loops_[mapping_event_loops_.size() - 1].get(),
        &constants_fetcher, camera_node.camera_number);

    if (FLAGS_visualize_solver) {
      // Show the extrinsics calibration to start, for reference to confirm
      const auto *calibration = FindCameraCalibration(
          constants_fetcher.constants(),
          mapping_event_loops_.back()->node()->name()->string_view(),
          camera_node.camera_number);
      cv::Mat extrinsics_cv =
          frc971::vision::CameraExtrinsics(calibration).value();
      Eigen::Matrix4d extrinsics_matrix;
      cv::cv2eigen(extrinsics_cv, extrinsics_matrix);
      const auto extrinsics = Eigen::Affine3d(extrinsics_matrix);

      vis_robot_.DrawRobotOutline(extrinsics, camera_node.camera_name(),
                                  kOrinColors.at(camera_node.camera_name()));
    }
  }

  if (FLAGS_visualize_solver) {
    cv::imshow("Extrinsics", vis_robot_.image_);
    cv::waitKey(0);
    vis_robot_.ClearImage();
    // Reset focal length to more zoomed out view for field
    const double kFocalLength = 500.0;
    vis_robot_.SetDefaultViewpoint(kImageWidth, kFocalLength);
  }
}

// Add detected apriltag poses relative to the robot to
// timestamped_target_detections
void TargetMapperReplay::HandleAprilTags(
    const TargetMap &map,
    aos::distributed_clock::time_point node_distributed_time,
    std::string camera_name, Eigen::Affine3d extrinsics) {
  bool drew = false;
  std::stringstream label;
  label << camera_name << " - ";

  if (map.target_poses()->size() == 0) {
    VLOG(2) << "Got 0 AprilTags for camera " << camera_name;
    return;
  }

  for (const auto *target_pose_fbs : *map.target_poses()) {
    // Skip detections with invalid ids
    if (static_cast<TargetMapper::TargetId>(target_pose_fbs->id()) <
            FLAGS_min_target_id ||
        static_cast<TargetMapper::TargetId>(target_pose_fbs->id()) >
            FLAGS_max_target_id) {
      VLOG(1) << "Skipping tag with invalid id of " << target_pose_fbs->id();
      continue;
    }

    // Skip detections with high pose errors
    if (target_pose_fbs->pose_error() > FLAGS_max_pose_error) {
      VLOG(1) << "Skipping tag " << target_pose_fbs->id()
              << " due to pose error of " << target_pose_fbs->pose_error();
      continue;
    }
    // Skip detections with high pose error ratios
    if (target_pose_fbs->pose_error_ratio() > FLAGS_max_pose_error_ratio) {
      VLOG(1) << "Skipping tag " << target_pose_fbs->id()
              << " due to pose error ratio of "
              << target_pose_fbs->pose_error_ratio();
      continue;
    }

    const TargetMapper::TargetPose target_pose =
        PoseUtils::TargetPoseFromFbs(*target_pose_fbs);

    Eigen::Affine3d H_camera_target =
        Eigen::Translation3d(target_pose.pose.p) * target_pose.pose.q;
    Eigen::Affine3d H_robot_target =
        CameraToRobotDetection(H_camera_target, extrinsics);

    ceres::examples::Pose3d target_pose_camera =
        PoseUtils::Affine3dToPose3d(H_camera_target);
    double distance_from_camera = target_pose_camera.p.norm();
    double distortion_factor = target_pose_fbs->distortion_factor();

    double distance_threshold = 5.0;
    if (distance_from_camera > distance_threshold) {
      ignore_count_++;
      LOG(INFO) << "Ignored " << ignore_count_ << " AprilTags with distance "
                << distance_from_camera << " > " << distance_threshold;
      continue;
    }

    CHECK(map.has_monotonic_timestamp_ns())
        << "Need detection timestamps for mapping";

    // Detection is usable, so store it
    timestamped_target_detections_.emplace_back(
        DataAdapter::TimestampedDetection{
            .time = node_distributed_time,
            .H_robot_target = H_robot_target,
            .distance_from_camera = distance_from_camera,
            .distortion_factor = distortion_factor,
            .id = static_cast<TargetMapper::TargetId>(target_pose.id)});

    if (FLAGS_visualize_solver) {
      // If we've already drawn this camera_name in the current image,
      // display the image before clearing and adding the new poses
      if (drawn_cameras_.count(camera_name) != 0) {
        display_count_++;
        cv::putText(vis_robot_.image_,
                    "Poses #" + std::to_string(display_count_),
                    cv::Point(600, 10), cv::FONT_HERSHEY_PLAIN, 1.0,
                    cv::Scalar(255, 255, 255));

        if (display_count_ >= FLAGS_skip_to) {
          VLOG(1) << "Showing image for camera " << camera_name
                  << " since we've drawn it already";
          cv::imshow("View", vis_robot_.image_);
          // Pause if delta_T is too large, but only after first image (to make
          // sure the delta's are correct)
          if (max_delta_T_world_robot_ > FLAGS_pause_on_distance &&
              display_count_ > 1) {
            LOG(INFO) << "Pausing since the delta between robot estimates is "
                      << max_delta_T_world_robot_ << " which is > threshold of "
                      << FLAGS_pause_on_distance;
            cv::waitKey(0);
          } else {
            cv::waitKey(FLAGS_wait_key);
          }
          max_delta_T_world_robot_ = 0.0;
        } else {
          VLOG(2) << "At poses #" << std::to_string(display_count_);
        }
        vis_robot_.ClearImage();
        drawn_cameras_.clear();
      }

      Eigen::Affine3d H_world_target = PoseUtils::Pose3dToAffine3d(
          kFixedTargetMapper.GetTargetPoseById(target_pose_fbs->id())->pose);
      Eigen::Affine3d H_world_robot = H_world_target * H_robot_target.inverse();
      VLOG(2) << camera_name << ", id " << target_pose_fbs->id()
              << ", t = " << node_distributed_time
              << ", pose_error = " << target_pose_fbs->pose_error()
              << ", pose_error_ratio = " << target_pose_fbs->pose_error_ratio()
              << ", robot_pos (x,y,z) = "
              << H_world_robot.translation().transpose();

      label << "id " << target_pose_fbs->id()
            << ": err (% of max): " << target_pose_fbs->pose_error() << " ("
            << (target_pose_fbs->pose_error() / FLAGS_max_pose_error)
            << ") err_ratio: " << target_pose_fbs->pose_error_ratio() << " ";

      vis_robot_.DrawRobotOutline(H_world_robot, camera_name,
                                  kOrinColors.at(camera_name));
      vis_robot_.DrawFrameAxes(H_world_target,
                               std::to_string(target_pose_fbs->id()),
                               kOrinColors.at(camera_name));

      double delta_T_world_robot =
          (H_world_robot.translation() - last_H_world_robot_.translation())
              .norm();
      max_delta_T_world_robot_ =
          std::max(delta_T_world_robot, max_delta_T_world_robot_);

      VLOG(1) << "Drew in info for camera " << camera_name << " and target #"
              << target_pose_fbs->id();
      drew = true;
      last_draw_time_ = node_distributed_time;
      last_H_world_robot_ = H_world_robot;
    }
  }
  if (FLAGS_visualize_solver) {
    if (drew) {
      // Collect all the labels from a given camera, and add the text
      // TODO: Need to fix this one
      int position_number = camera_ordering_map[camera_name];
      cv::putText(vis_robot_.image_, label.str(),
                  cv::Point(10, 30 + 20 * position_number),
                  cv::FONT_HERSHEY_PLAIN, 1.0, kOrinColors.at(camera_name));
      drawn_cameras_.emplace(camera_name);
    } else if (node_distributed_time - last_draw_time_ >
                   std::chrono::milliseconds(30) &&
               display_count_ >= FLAGS_skip_to && drew) {
      // TODO: Check on 30ms value-- does this make sense?
      double delta_t = (node_distributed_time - last_draw_time_).count() / 1e6;
      VLOG(1) << "Last result was " << delta_t << "ms ago";
      cv::putText(vis_robot_.image_, "No detections in last 30ms",
                  cv::Point(10, 0), cv::FONT_HERSHEY_PLAIN, 1.0,
                  kOrinColors.at(camera_name));
      // Display and clear the image if we haven't draw in a while
      VLOG(1) << "Displaying image due to time lapse";
      cv::imshow("View", vis_robot_.image_);
      cv::waitKey(FLAGS_wait_key);
      max_delta_T_world_robot_ = 0.0;
      drawn_cameras_.clear();
    }
  }
}

void TargetMapperReplay::HandleNodeCaptures(
    aos::EventLoop *mapping_event_loop,
    frc971::constants::ConstantsFetcher<y2024_swerve::Constants>
        *constants_fetcher,
    int camera_number) {
  // Get the camera extrinsics
  std::string node_name =
      std::string(mapping_event_loop->node()->name()->string_view());
  const auto *calibration = FindCameraCalibration(
      constants_fetcher->constants(), node_name, camera_number);
  cv::Mat extrinsics_cv = frc971::vision::CameraExtrinsics(calibration).value();
  Eigen::Matrix4d extrinsics_matrix;
  cv::cv2eigen(extrinsics_cv, extrinsics_matrix);
  const auto extrinsics = Eigen::Affine3d(extrinsics_matrix);
  std::string camera_name = absl::StrFormat(
      "/%s/camera%d", mapping_event_loop->node()->name()->str(), camera_number);

  mapping_event_loop->MakeWatcher(
      camera_name.c_str(), [this, mapping_event_loop, extrinsics,
                            camera_name](const TargetMap &map) {
        aos::distributed_clock::time_point node_distributed_time =
            reader_->event_loop_factory()
                ->GetNodeEventLoopFactory(mapping_event_loop->node())
                ->ToDistributedClock(aos::monotonic_clock::time_point(
                    aos::monotonic_clock::duration(
                        map.monotonic_timestamp_ns())));

        HandleAprilTags(map, node_distributed_time, camera_name, extrinsics);
      });
}

void TargetMapperReplay::MaybeSolve() {
  if (FLAGS_solve) {
    auto target_constraints =
        DataAdapter::MatchTargetDetections(timestamped_target_detections_);

    if (FLAGS_split_field) {
      // Remove constraints between the two sides of the field - these are
      // basically garbage because of how far the camera is. We will use seeding
      // below to connect the two sides
      target_constraints.erase(
          std::remove_if(
              target_constraints.begin(), target_constraints.end(),
              [](const auto &constraint) {
                return (
                    kIdAllianceMap[static_cast<uint>(constraint.id_begin)] !=
                    kIdAllianceMap[static_cast<uint>(constraint.id_end)]);
              }),
          target_constraints.end());
    }

    LOG(INFO) << "Solving for locations of tags with "
              << target_constraints.size() << " constraints";
    TargetMapper mapper(FLAGS_json_path, target_constraints);
    mapper.Solve(FLAGS_field_name, FLAGS_output_dir);

    if (!FLAGS_dump_constraints_to.empty()) {
      mapper.DumpConstraints(FLAGS_dump_constraints_to);
    }
    if (!FLAGS_dump_stats_to.empty()) {
      mapper.DumpStats(FLAGS_dump_stats_to);
    }
    mapper.PrintDiffs();
  }
}

void MappingMain(int argc, char *argv[]) {
  std::vector<DataAdapter::TimestampedDetection> timestamped_target_detections;

  std::optional<aos::FlatbufferDetachedBuffer<aos::Configuration>> config =
      (FLAGS_config.empty()
           ? std::nullopt
           : std::make_optional(aos::configuration::ReadConfig(FLAGS_config)));

  // Open logfiles
  aos::logger::LogReader reader(
      aos::logger::SortParts(aos::logger::FindLogs(argc, argv)),
      config.has_value() ? &config->message() : nullptr);

  TargetMapperReplay mapper_replay(&reader);
  reader.event_loop_factory()->Run();
  mapper_replay.MaybeSolve();
}

}  // namespace y2024_swerve::vision

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  y2024_swerve::vision::MappingMain(argc, argv);
}

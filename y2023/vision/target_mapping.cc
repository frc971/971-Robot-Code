#include "aos/configuration.h"
#include "aos/events/logging/log_reader.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/init.h"
#include "aos/util/mcap_logger.h"
#include "frc971/control_loops/pose.h"
#include "frc971/vision/calibration_generated.h"
#include "frc971/vision/target_mapper.h"
#include "frc971/vision/visualize_robot.h"
#include "opencv2/aruco.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/core/eigen.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "y2023/constants/simulated_constants_sender.h"
#include "y2023/vision/aprilrobotics.h"
#include "y2023/vision/vision_util.h"

DEFINE_string(json_path, "y2023/vision/maps/target_map.json",
              "Specify path for json with initial pose guesses.");
DEFINE_string(config, "",
              "If set, override the log's config file with this one.");
DEFINE_string(constants_path, "y2023/constants/constants.json",
              "Path to the constant file");
DEFINE_string(output_dir, "y2023/vision/maps",
              "Directory to write solved target map to");
DEFINE_string(field_name, "charged_up",
              "Field name, for the output json filename and flatbuffer field");
DEFINE_int32(team_number, 0,
             "Required: Use the calibration for a node with this team number");
DEFINE_string(mcap_output_path, "/tmp/log.mcap", "Log to output.");
DEFINE_string(pi, "pi1", "Pi name to generate mcap log for; defaults to pi1.");
DEFINE_double(max_pose_error, 1e-6,
              "Throw out target poses with a higher pose error than this");
DEFINE_double(
    max_pose_error_ratio, 0.4,
    "Throw out target poses with a higher pose error ratio than this");
DEFINE_uint64(wait_key, 0,
              "Time in ms to wait between images, if no click (0 to wait "
              "indefinitely until click).");
DEFINE_double(pause_on_distance, 1.0,
              "Pause if two consecutive implied robot positions differ by more "
              "than this many meters");
DEFINE_uint64(skip_to, 1,
              "Start at combined image of this number (1 is the first image)");
DEFINE_bool(solve, true, "Whether to solve for the field's target map.");
DEFINE_string(dump_constraints_to, "/tmp/constraints.txt",
              "Write the target constraints to this path");
DECLARE_int32(frozen_target_id);
DECLARE_int32(min_target_id);
DECLARE_int32(max_target_id);
DECLARE_bool(visualize_solver);

namespace y2023 {
namespace vision {
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
  ~TargetMapperReplay();

  // Solves for the target poses with the accumulated detections if FLAGS_solve.
  void MaybeSolve();

 private:
  static constexpr int kImageWidth = 1280;
  // Map from pi node name to color for drawing
  static const std::map<std::string, cv::Scalar> kPiColors;
  // Contains fixed target poses without solving, for use with visualization
  static const TargetMapper kFixedTargetMapper;

  // Change reference frame from camera to robot
  static Eigen::Affine3d CameraToRobotDetection(Eigen::Affine3d H_camera_target,
                                                Eigen::Affine3d extrinsics);

  // Adds april tag detections into the detection list, and handles
  // visualization
  void HandleAprilTags(const TargetMap &map,
                       aos::distributed_clock::time_point pi_distributed_time,
                       std::string node_name, Eigen::Affine3d extrinsics);

  // Gets images from the given pi and passes apriltag positions to
  // HandleAprilTags()
  void HandlePiCaptures(aos::EventLoop *detection_event_loop,
                        aos::EventLoop *mapping_event_loop);

  aos::logger::LogReader *reader_;
  // April tag detections from all pis
  std::vector<DataAdapter::TimestampedDetection> timestamped_target_detections_;
  std::vector<std::unique_ptr<AprilRoboticsDetector>> detectors_;

  VisualizeRobot vis_robot_;
  // Set of node names which are currently drawn on the display
  std::set<std::string> drawn_nodes_;
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

  std::vector<std::unique_ptr<aos::EventLoop>> detection_event_loops_;
  std::vector<std::unique_ptr<aos::EventLoop>> mapping_event_loops_;

  std::unique_ptr<aos::EventLoop> mcap_event_loop_;
  std::unique_ptr<aos::McapLogger> relogger_;
};

const auto TargetMapperReplay::kPiColors = std::map<std::string, cv::Scalar>{
    {"pi1", cv::Scalar(255, 0, 255)},
    {"pi2", cv::Scalar(255, 255, 0)},
    {"pi3", cv::Scalar(0, 255, 255)},
    {"pi4", cv::Scalar(255, 165, 0)},
};

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
      detectors_(),
      vis_robot_(cv::Size(1280, 1000)),
      drawn_nodes_(),
      display_count_(0),
      last_draw_time_(aos::distributed_clock::min_time),
      last_H_world_robot_(Eigen::Matrix4d::Identity()),
      max_delta_T_world_robot_(0.0) {
  // Send new april tag poses. This allows us to take a log of images, then
  // play with the april detection code and see those changes take effect in
  // mapping
  constexpr size_t kNumPis = 4;
  for (size_t i = 1; i <= kNumPis; i++) {
    reader_->RemapLoggedChannel(absl::StrFormat("/pi%u/camera", i),
                                "frc971.vision.TargetMap");
    reader_->RemapLoggedChannel(absl::StrFormat("/pi%u/camera", i),
                                "foxglove.ImageAnnotations");
    reader_->RemapLoggedChannel(absl::StrFormat("/pi%u/constants", i),
                                "y2023.Constants");
  }

  reader_->RemapLoggedChannel("/imu/constants", "y2023.Constants");

  reader_->Register();

  SendSimulationConstants(reader_->event_loop_factory(), FLAGS_team_number,
                          FLAGS_constants_path);

  for (size_t i = 1; i < kNumPis; i++) {
    std::string node_name = "pi" + std::to_string(i);
    const aos::Node *pi =
        aos::configuration::GetNode(reader_->configuration(), node_name);
    detection_event_loops_.emplace_back(
        reader_->event_loop_factory()->MakeEventLoop(node_name + "_detection",
                                                     pi));
    mapping_event_loops_.emplace_back(
        reader_->event_loop_factory()->MakeEventLoop(node_name + "_mapping",
                                                     pi));
    HandlePiCaptures(
        detection_event_loops_[detection_event_loops_.size() - 1].get(),
        mapping_event_loops_[mapping_event_loops_.size() - 1].get());
  }

  if (!FLAGS_mcap_output_path.empty()) {
    LOG(INFO) << "Writing out mcap file to " << FLAGS_mcap_output_path;
    const aos::Node *node =
        aos::configuration::GetNode(reader_->configuration(), FLAGS_pi);
    reader_->event_loop_factory()->GetNodeEventLoopFactory(node)->OnStartup(
        [this, node]() {
          mcap_event_loop_ =
              reader_->event_loop_factory()->MakeEventLoop("mcap", node);
          relogger_ = std::make_unique<aos::McapLogger>(
              mcap_event_loop_.get(), FLAGS_mcap_output_path,
              aos::McapLogger::Serialization::kFlatbuffer,
              aos::McapLogger::CanonicalChannelNames::kShortened,
              aos::McapLogger::Compression::kLz4);
        });
  }

  if (FLAGS_visualize_solver) {
    vis_robot_.ClearImage();
    const double kFocalLength = 500.0;
    vis_robot_.SetDefaultViewpoint(kImageWidth, kFocalLength);
  }
}

TargetMapperReplay::~TargetMapperReplay() {
  // Clean up all the pointers
  for (auto &detector_ptr : detectors_) {
    detector_ptr.reset();
  }
}

// Add detected apriltag poses relative to the robot to
// timestamped_target_detections
void TargetMapperReplay::HandleAprilTags(
    const TargetMap &map,
    aos::distributed_clock::time_point pi_distributed_time,
    std::string node_name, Eigen::Affine3d extrinsics) {
  bool drew = false;
  std::stringstream label;
  label << node_name << " - ";

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

    CHECK(map.has_monotonic_timestamp_ns())
        << "Need detection timestamps for mapping";

    timestamped_target_detections_.emplace_back(
        DataAdapter::TimestampedDetection{
            .time = pi_distributed_time,
            .H_robot_target = H_robot_target,
            .distance_from_camera = distance_from_camera,
            .distortion_factor = distortion_factor,
            .id = static_cast<TargetMapper::TargetId>(target_pose.id)});

    if (FLAGS_visualize_solver) {
      // If we've already drawn in the current image,
      // display it before clearing and adding the new poses
      if (drawn_nodes_.count(node_name) != 0) {
        display_count_++;
        cv::putText(vis_robot_.image_,
                    "Poses #" + std::to_string(display_count_),
                    cv::Point(600, 10), cv::FONT_HERSHEY_PLAIN, 1.0,
                    cv::Scalar(255, 255, 255));

        if (display_count_ >= FLAGS_skip_to) {
          cv::imshow("View", vis_robot_.image_);
          cv::waitKey(max_delta_T_world_robot_ > FLAGS_pause_on_distance
                          ? 0
                          : FLAGS_wait_key);
          max_delta_T_world_robot_ = 0.0;
        } else {
          VLOG(1) << "At poses #" << std::to_string(display_count_);
        }
        vis_robot_.ClearImage();
        drawn_nodes_.clear();
      }

      Eigen::Affine3d H_world_target = PoseUtils::Pose3dToAffine3d(
          kFixedTargetMapper.GetTargetPoseById(target_pose_fbs->id())->pose);
      Eigen::Affine3d H_world_robot = H_world_target * H_robot_target.inverse();
      VLOG(2) << node_name << ", " << target_pose_fbs->id()
              << ", t = " << pi_distributed_time
              << ", pose_error = " << target_pose_fbs->pose_error()
              << ", pose_error_ratio = " << target_pose_fbs->pose_error_ratio()
              << ", robot_pos (x,y,z) + "
              << H_world_robot.translation().transpose();

      label << "id " << target_pose_fbs->id() << ": err "
            << (target_pose_fbs->pose_error() / FLAGS_max_pose_error)
            << " ratio " << target_pose_fbs->pose_error_ratio() << " ";

      vis_robot_.DrawRobotOutline(H_world_robot,
                                  std::to_string(target_pose_fbs->id()),
                                  kPiColors.at(node_name));

      vis_robot_.DrawFrameAxes(H_world_target,
                               std::to_string(target_pose_fbs->id()),
                               kPiColors.at(node_name));

      double delta_T_world_robot =
          (H_world_robot.translation() - last_H_world_robot_.translation())
              .norm();
      max_delta_T_world_robot_ =
          std::max(delta_T_world_robot, max_delta_T_world_robot_);

      drew = true;
      last_draw_time_ = pi_distributed_time;
      last_H_world_robot_ = H_world_robot;
    }
  }
  if (drew) {
    size_t pi_number =
        static_cast<size_t>(node_name[node_name.size() - 1] - '0');
    cv::putText(vis_robot_.image_, label.str(),
                cv::Point(10, 10 + 20 * pi_number), cv::FONT_HERSHEY_PLAIN, 1.0,
                kPiColors.at(node_name));

    drawn_nodes_.emplace(node_name);
  } else if (pi_distributed_time - last_draw_time_ >
                 std::chrono::milliseconds(30) &&
             display_count_ >= FLAGS_skip_to) {
    // Clear the image if we haven't draw in a while
    vis_robot_.ClearImage();
    cv::imshow("View", vis_robot_.image_);
    cv::waitKey(FLAGS_wait_key);
    max_delta_T_world_robot_ = 0.0;
  }
}

void TargetMapperReplay::HandlePiCaptures(aos::EventLoop *detection_event_loop,
                                          aos::EventLoop *mapping_event_loop) {
  static constexpr std::string_view kImageChannel = "/camera";
  // For the robots, we need to flip the image since the cameras are rolled by
  // 180 degrees
  bool flip_image = (FLAGS_team_number != 7971);
  auto detector_ptr = std::make_unique<AprilRoboticsDetector>(
      detection_event_loop, kImageChannel, flip_image);
  // Get the camera extrinsics
  cv::Mat extrinsics_cv = detector_ptr->extrinsics().value();
  Eigen::Matrix4d extrinsics_matrix;
  cv::cv2eigen(extrinsics_cv, extrinsics_matrix);
  const auto extrinsics = Eigen::Affine3d(extrinsics_matrix);

  detectors_.emplace_back(std::move(detector_ptr));

  mapping_event_loop->MakeWatcher("/camera", [=](const TargetMap &map) {
    aos::distributed_clock::time_point pi_distributed_time =
        reader_->event_loop_factory()
            ->GetNodeEventLoopFactory(mapping_event_loop->node())
            ->ToDistributedClock(aos::monotonic_clock::time_point(
                aos::monotonic_clock::duration(map.monotonic_timestamp_ns())));

    std::string node_name = detection_event_loop->node()->name()->str();
    HandleAprilTags(map, pi_distributed_time, node_name, extrinsics);
  });
}

void TargetMapperReplay::MaybeSolve() {
  if (FLAGS_solve) {
    auto target_constraints =
        DataAdapter::MatchTargetDetections(timestamped_target_detections_);

    // Remove constraints between the two sides of the field - these are
    // basically garbage because of how far the camera is. We will use seeding
    // below to connect the two sides
    target_constraints.erase(
        std::remove_if(target_constraints.begin(), target_constraints.end(),
                       [](const auto &constraint) {
                         constexpr TargetMapper::TargetId kMaxRedId = 4;
                         TargetMapper::TargetId min_id =
                             std::min(constraint.id_begin, constraint.id_end);
                         TargetMapper::TargetId max_id =
                             std::max(constraint.id_begin, constraint.id_end);
                         return (min_id <= kMaxRedId && max_id > kMaxRedId);
                       }),
        target_constraints.end());

    if (!FLAGS_dump_constraints_to.empty()) {
      std::ofstream fout(FLAGS_dump_constraints_to);
      for (const auto &constraint : target_constraints) {
        fout << constraint << std::endl;
      }
      fout.flush();
      fout.close();
    }

    // Give seed constraints with a higher confidence to ground the solver.
    // This "distance from camera" controls the noise of the seed measurement
    constexpr double kSeedDistanceFromCamera = 1.0;

    constexpr double kSeedDistortionFactor = 0.0;
    const DataAdapter::TimestampedDetection frozen_detection_seed = {
        .time = aos::distributed_clock::min_time,
        .H_robot_target = PoseUtils::Pose3dToAffine3d(
            kFixedTargetMapper.GetTargetPoseById(FLAGS_frozen_target_id)
                .value()
                .pose),
        .distance_from_camera = kSeedDistanceFromCamera,
        .distortion_factor = kSeedDistortionFactor,
        .id = FLAGS_frozen_target_id};

    constexpr TargetMapper::TargetId kAbsMinTargetId = 1;
    constexpr TargetMapper::TargetId kAbsMaxTargetId = 8;
    for (TargetMapper::TargetId id = kAbsMinTargetId; id <= kAbsMaxTargetId;
         id++) {
      if (id == FLAGS_frozen_target_id) {
        continue;
      }

      const DataAdapter::TimestampedDetection detection_seed = {
          .time = aos::distributed_clock::min_time,
          .H_robot_target = PoseUtils::Pose3dToAffine3d(
              kFixedTargetMapper.GetTargetPoseById(id).value().pose),
          .distance_from_camera = kSeedDistanceFromCamera,
          .distortion_factor = kSeedDistortionFactor,
          .id = id};
      target_constraints.emplace_back(DataAdapter::ComputeTargetConstraint(
          frozen_detection_seed, detection_seed,
          DataAdapter::ComputeConfidence(frozen_detection_seed,
                                         detection_seed)));
    }

    TargetMapper mapper(FLAGS_json_path, target_constraints);
    mapper.Solve(FLAGS_field_name, FLAGS_output_dir);
  }
}

void MappingMain(int argc, char *argv[]) {
  std::vector<std::string> unsorted_logfiles =
      aos::logger::FindLogs(argc, argv);

  std::vector<DataAdapter::TimestampedDetection> timestamped_target_detections;

  std::optional<aos::FlatbufferDetachedBuffer<aos::Configuration>> config =
      (FLAGS_config.empty()
           ? std::nullopt
           : std::make_optional(aos::configuration::ReadConfig(FLAGS_config)));

  // Open logfiles
  aos::logger::LogReader reader(
      aos::logger::SortParts(unsorted_logfiles),
      config.has_value() ? &config->message() : nullptr);

  TargetMapperReplay mapper_replay(&reader);
  reader.event_loop_factory()->Run();
  mapper_replay.MaybeSolve();
}

}  // namespace vision
}  // namespace y2023

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  y2023::vision::MappingMain(argc, argv);
}

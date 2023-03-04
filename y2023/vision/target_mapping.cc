#include "aos/configuration.h"
#include "aos/events/logging/log_reader.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/init.h"
#include "aos/util/mcap_logger.h"
#include "frc971/control_loops/pose.h"
#include "frc971/vision/calibration_generated.h"
#include "frc971/vision/charuco_lib.h"
#include "frc971/vision/target_mapper.h"
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
DEFINE_string(config, "y2023/aos_config.json",
              "Path to the config file to use.");
DEFINE_string(constants_path, "y2023/constants/constants.json",
              "Path to the constant file");
DEFINE_string(output_dir, "y2023/vision/maps",
              "Directory to write solved target map to");
DEFINE_string(field_name, "charged_up",
              "Field name, for the output json filename and flatbuffer field");
DEFINE_int32(team_number, 7971,
             "Use the calibration for a node with this team number");
DEFINE_string(mcap_output_path, "/tmp/log.mcap", "Log to output.");
DEFINE_string(pi, "pi1", "Pi name to generate mcap log for; defaults to pi1.");

namespace y2023 {
namespace vision {
using frc971::vision::DataAdapter;
using frc971::vision::ImageCallback;
using frc971::vision::PoseUtils;
using frc971::vision::TargetMap;
using frc971::vision::TargetMapper;
namespace calibration = frc971::vision::calibration;

// Change reference frame from camera to robot
Eigen::Affine3d CameraToRobotDetection(Eigen::Affine3d H_camera_target,
                                       Eigen::Affine3d extrinsics) {
  const Eigen::Affine3d H_robot_camera = extrinsics;
  const Eigen::Affine3d H_robot_target = H_robot_camera * H_camera_target;
  return H_robot_target;
}

// Add detected apriltag poses relative to the robot to
// timestamped_target_detections
void HandleAprilTag(const TargetMap &map,
                    aos::distributed_clock::time_point pi_distributed_time,
                    std::vector<DataAdapter::TimestampedDetection>
                        *timestamped_target_detections,
                    Eigen::Affine3d extrinsics) {
  for (const auto *target_pose_fbs : *map.target_poses()) {
    const TargetMapper::TargetPose target_pose =
        PoseUtils::TargetPoseFromFbs(*target_pose_fbs);

    Eigen::Affine3d H_camera_target =
        Eigen::Translation3d(target_pose.pose.p) * target_pose.pose.q;
    Eigen::Affine3d H_robot_target =
        CameraToRobotDetection(H_camera_target, extrinsics);

    ceres::examples::Pose3d target_pose_camera =
        PoseUtils::Affine3dToPose3d(H_camera_target);
    double distance_from_camera = target_pose_camera.p.norm();

    CHECK(map.has_monotonic_timestamp_ns())
        << "Need detection timestamps for mapping";

    timestamped_target_detections->emplace_back(
        DataAdapter::TimestampedDetection{
            .time = pi_distributed_time,
            .H_robot_target = H_robot_target,
            .distance_from_camera = distance_from_camera,
            .id = static_cast<TargetMapper::TargetId>(target_pose.id)});
  }
}

// Get images from pi and pass apriltag positions to HandleAprilTag()
void HandlePiCaptures(
    aos::EventLoop *detection_event_loop, aos::EventLoop *mapping_event_loop,
    aos::logger::LogReader *reader,
    std::vector<DataAdapter::TimestampedDetection>
        *timestamped_target_detections,
    std::vector<std::unique_ptr<AprilRoboticsDetector>> *detectors) {
  static constexpr std::string_view kImageChannel = "/camera";
  auto detector_ptr = std::make_unique<AprilRoboticsDetector>(
      detection_event_loop, kImageChannel);
  // Get the camera extrinsics
  cv::Mat extrinsics_cv = detector_ptr->extrinsics().value();
  Eigen::Matrix4d extrinsics_matrix;
  cv::cv2eigen(extrinsics_cv, extrinsics_matrix);
  const auto extrinsics = Eigen::Affine3d(extrinsics_matrix);

  detectors->emplace_back(std::move(detector_ptr));

  mapping_event_loop->MakeWatcher("/camera", [=](const TargetMap &map) {
    aos::distributed_clock::time_point pi_distributed_time =
        reader->event_loop_factory()
            ->GetNodeEventLoopFactory(mapping_event_loop->node())
            ->ToDistributedClock(aos::monotonic_clock::time_point(
                aos::monotonic_clock::duration(map.monotonic_timestamp_ns())));

    HandleAprilTag(map, pi_distributed_time, timestamped_target_detections,
                   extrinsics);
  });
}

void MappingMain(int argc, char *argv[]) {
  std::vector<std::string> unsorted_logfiles =
      aos::logger::FindLogs(argc, argv);

  std::vector<DataAdapter::TimestampedDetection> timestamped_target_detections;

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(FLAGS_config);

  // open logfiles
  aos::logger::LogReader reader(aos::logger::SortParts(unsorted_logfiles),
                                &config.message());
  // Send new april tag poses. This allows us to take a log of images, then play
  // with the april detection code and see those changes take effect in mapping
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

  std::vector<std::unique_ptr<AprilRoboticsDetector>> detectors;

  const aos::Node *pi1 =
      aos::configuration::GetNode(reader.configuration(), "pi1");
  std::unique_ptr<aos::EventLoop> pi1_detection_event_loop =
      reader.event_loop_factory()->MakeEventLoop("pi1_detection", pi1);
  std::unique_ptr<aos::EventLoop> pi1_mapping_event_loop =
      reader.event_loop_factory()->MakeEventLoop("pi1_mapping", pi1);
  HandlePiCaptures(pi1_detection_event_loop.get(), pi1_mapping_event_loop.get(),
                   &reader, &timestamped_target_detections, &detectors);

  const aos::Node *pi2 =
      aos::configuration::GetNode(reader.configuration(), "pi2");
  std::unique_ptr<aos::EventLoop> pi2_detection_event_loop =
      reader.event_loop_factory()->MakeEventLoop("pi2_detection", pi2);
  std::unique_ptr<aos::EventLoop> pi2_mapping_event_loop =
      reader.event_loop_factory()->MakeEventLoop("pi2_mapping", pi2);
  HandlePiCaptures(pi2_detection_event_loop.get(), pi2_mapping_event_loop.get(),
                   &reader, &timestamped_target_detections, &detectors);

  const aos::Node *pi3 =
      aos::configuration::GetNode(reader.configuration(), "pi3");
  std::unique_ptr<aos::EventLoop> pi3_detection_event_loop =
      reader.event_loop_factory()->MakeEventLoop("pi3_detection", pi3);
  std::unique_ptr<aos::EventLoop> pi3_mapping_event_loop =
      reader.event_loop_factory()->MakeEventLoop("pi3_mapping", pi3);
  HandlePiCaptures(pi3_detection_event_loop.get(), pi3_mapping_event_loop.get(),
                   &reader, &timestamped_target_detections, &detectors);

  const aos::Node *pi4 =
      aos::configuration::GetNode(reader.configuration(), "pi4");
  std::unique_ptr<aos::EventLoop> pi4_detection_event_loop =
      reader.event_loop_factory()->MakeEventLoop("pi4_detection", pi4);
  std::unique_ptr<aos::EventLoop> pi4_mapping_event_loop =
      reader.event_loop_factory()->MakeEventLoop("pi4_mapping", pi4);
  HandlePiCaptures(pi4_detection_event_loop.get(), pi4_mapping_event_loop.get(),
                   &reader, &timestamped_target_detections, &detectors);

  std::unique_ptr<aos::EventLoop> mcap_event_loop;
  std::unique_ptr<aos::McapLogger> relogger;
  if (!FLAGS_mcap_output_path.empty()) {
    LOG(INFO) << "Writing out mcap file to " << FLAGS_mcap_output_path;
    // TODO: Should make this work for any pi
    const aos::Node *node =
        aos::configuration::GetNode(reader.configuration(), FLAGS_pi);
    reader.event_loop_factory()->GetNodeEventLoopFactory(node)->OnStartup(
        [&relogger, &mcap_event_loop, &reader, node]() {
          mcap_event_loop =
              reader.event_loop_factory()->MakeEventLoop("mcap", node);
          relogger = std::make_unique<aos::McapLogger>(
              mcap_event_loop.get(), FLAGS_mcap_output_path,
              aos::McapLogger::Serialization::kFlatbuffer,
              aos::McapLogger::CanonicalChannelNames::kShortened,
              aos::McapLogger::Compression::kLz4);
        });
  }

  reader.event_loop_factory()->Run();

  auto target_constraints =
      DataAdapter::MatchTargetDetections(timestamped_target_detections);

  frc971::vision::TargetMapper mapper(FLAGS_json_path, target_constraints);
  mapper.Solve(FLAGS_field_name, FLAGS_output_dir);

  // Clean up all the pointers
  for (auto &detector_ptr : detectors) {
    detector_ptr.reset();
  }
}

}  // namespace vision
}  // namespace y2023

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  y2023::vision::MappingMain(argc, argv);
}

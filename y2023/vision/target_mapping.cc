#include "aos/events/logging/log_reader.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/init.h"
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
#include "y2023/vision/aprilrobotics.h"
#include "y2023/vision/vision_util.h"

DEFINE_string(json_path, "target_map.json",
              "Specify path for json with initial pose guesses.");
DECLARE_int32(team_number);

namespace y2023 {
namespace vision {
using frc971::vision::DataAdapter;
using frc971::vision::ImageCallback;
using frc971::vision::PoseUtils;
using frc971::vision::TargetMap;
using frc971::vision::TargetMapper;
namespace calibration = frc971::vision::calibration;

// Change reference frame from camera to robot
Eigen::Affine3d CameraToRobotDetection(Eigen::Affine3d H_camrob_target,
                                       Eigen::Affine3d extrinsics) {
  const Eigen::Affine3d H_robot_camrob = extrinsics;
  const Eigen::Affine3d H_robot_target = H_robot_camrob * H_camrob_target;
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

    Eigen::Affine3d H_camcv_target =
        Eigen::Translation3d(target_pose.pose.p) * target_pose.pose.q;
    // With X, Y, Z being robot axes and x, y, z being camera axes,
    // x = -Y, y = -Z, z = X
    static const Eigen::Affine3d H_camcv_camrob =
        Eigen::Affine3d((Eigen::Matrix4d() << 0.0, -1.0, 0.0, 0.0, 0.0, 0.0,
                         -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
                            .finished());
    Eigen::Affine3d H_camrob_target = H_camcv_camrob.inverse() * H_camcv_target;
    Eigen::Affine3d H_robot_target =
        CameraToRobotDetection(H_camrob_target, extrinsics);

    ceres::examples::Pose3d target_pose_camera =
        PoseUtils::Affine3dToPose3d(H_camrob_target);
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
    aos::EventLoop *pi_event_loop, aos::logger::LogReader *reader,
    std::vector<DataAdapter::TimestampedDetection>
        *timestamped_target_detections,
    std::vector<std::unique_ptr<AprilRoboticsDetector>> *detectors) {
  // TODO(milind): change to /camera once we log at full frequency
  static constexpr std::string_view kImageChannel = "/camera/decimated";
  auto detector_ptr =
      std::make_unique<AprilRoboticsDetector>(pi_event_loop, kImageChannel);
  // Get the camera extrinsics
  cv::Mat extrinsics_cv = detector_ptr->extrinsics();
  Eigen::Matrix4d extrinsics_matrix;
  cv::cv2eigen(extrinsics_cv, extrinsics_matrix);
  const auto extrinsics = Eigen::Affine3d(extrinsics_matrix);

  detectors->emplace_back(std::move(detector_ptr));

  pi_event_loop->MakeWatcher("/camera", [=](const TargetMap &map) {
    aos::distributed_clock::time_point pi_distributed_time =
        reader->event_loop_factory()
            ->GetNodeEventLoopFactory(pi_event_loop->node())
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

  // open logfiles
  aos::logger::LogReader reader(aos::logger::SortParts(unsorted_logfiles));
  reader.Register();

  std::vector<std::unique_ptr<AprilRoboticsDetector>> detectors;

  const aos::Node *pi1 =
      aos::configuration::GetNode(reader.configuration(), "pi1");
  std::unique_ptr<aos::EventLoop> pi1_event_loop =
      reader.event_loop_factory()->MakeEventLoop("pi1", pi1);
  HandlePiCaptures(pi1_event_loop.get(), &reader,
                   &timestamped_target_detections, &detectors);

  const aos::Node *pi2 =
      aos::configuration::GetNode(reader.configuration(), "pi2");
  std::unique_ptr<aos::EventLoop> pi2_event_loop =
      reader.event_loop_factory()->MakeEventLoop("pi2", pi2);
  HandlePiCaptures(pi2_event_loop.get(), &reader,
                   &timestamped_target_detections, &detectors);

  const aos::Node *pi3 =
      aos::configuration::GetNode(reader.configuration(), "pi3");
  std::unique_ptr<aos::EventLoop> pi3_event_loop =
      reader.event_loop_factory()->MakeEventLoop("pi3", pi3);
  HandlePiCaptures(pi3_event_loop.get(), &reader,
                   &timestamped_target_detections, &detectors);

  const aos::Node *pi4 =
      aos::configuration::GetNode(reader.configuration(), "pi4");
  std::unique_ptr<aos::EventLoop> pi4_event_loop =
      reader.event_loop_factory()->MakeEventLoop("pi4", pi4);
  HandlePiCaptures(pi4_event_loop.get(), &reader,
                   &timestamped_target_detections, &detectors);

  reader.event_loop_factory()->Run();

  auto target_constraints =
      DataAdapter::MatchTargetDetections(timestamped_target_detections);

  frc971::vision::TargetMapper mapper(FLAGS_json_path, target_constraints);
  mapper.Solve("charged_up");

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

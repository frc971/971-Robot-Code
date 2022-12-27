#include "aos/events/logging/log_reader.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/init.h"
#include "frc971/control_loops/pose.h"
#include "frc971/vision/charuco_lib.h"
#include "frc971/vision/target_mapper.h"
#include "opencv2/aruco.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/core/eigen.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "y2022/control_loops/superstructure/superstructure_status_generated.h"
#include "y2022/vision/camera_reader.h"

DEFINE_string(json_path, "target_map.json",
              "Specify path for json with initial pose guesses.");
DEFINE_int32(team_number, 971,
             "Use the calibration for a node with this team number");

DEFINE_bool(
    use_robot_position, false,
    "If true, use localizer output messages to get the robot position, and "
    "superstructure status messages to get the turret angle, at the "
    "times of target detections. Currently does not work reliably on the box "
    "of pis.");

namespace y2022 {
namespace vision {
using frc971::vision::DataAdapter;
using frc971::vision::PoseUtils;
using frc971::vision::TargetMapper;
namespace superstructure = ::y2022::control_loops::superstructure;

// Find transformation from camera to robot reference frame
Eigen::Affine3d CameraTransform(Eigen::Affine3d fixed_extrinsics,
                                Eigen::Affine3d turret_extrinsics,
                                double turret_position) {
  // Calculate the pose of the camera relative to the robot origin.
  Eigen::Affine3d H_robot_camrob =
      fixed_extrinsics *
      Eigen::Affine3d(frc971::control_loops::TransformationMatrixForYaw<double>(
          turret_position)) *
      turret_extrinsics;

  return H_robot_camrob;
}

// Change reference frame from camera to robot
Eigen::Affine3d CameraToRobotDetection(Eigen::Affine3d H_camcv_target,
                                       Eigen::Affine3d fixed_extrinsics,
                                       Eigen::Affine3d turret_extrinsics,
                                       double turret_position) {
  // With X, Y, Z being robot axes and x, y, z being camera axes,
  // x = -Y, y = -Z, z = X
  const Eigen::Affine3d H_camcv_camrob =
      Eigen::Affine3d((Eigen::Matrix4d() << 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, -1.0,
                       0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
                          .finished());

  const Eigen::Affine3d H_robot_camrob =
      CameraTransform(fixed_extrinsics, turret_extrinsics, turret_position);
  const Eigen::Affine3d H_robot_target =
      H_robot_camrob * H_camcv_camrob.inverse() * H_camcv_target;
  return H_robot_target;
}

// Add detected apriltag poses relative to the robot to
// timestamped_target_detections
void HandleAprilTag(aos::distributed_clock::time_point pi_distributed_time,
                    std::vector<cv::Vec4i> april_ids,
                    std::vector<Eigen::Vector3d> rvecs_eigen,
                    std::vector<Eigen::Vector3d> tvecs_eigen,
                    std::vector<DataAdapter::TimestampedDetection>
                        *timestamped_target_detections,
                    std::optional<aos::Fetcher<superstructure::Status>>
                        *superstructure_status_fetcher,
                    Eigen::Affine3d fixed_extrinsics,
                    Eigen::Affine3d turret_extrinsics) {
  double turret_position = 0.0;

  if (superstructure_status_fetcher->has_value()) {
    CHECK(superstructure_status_fetcher->value().Fetch());
    turret_position =
        superstructure_status_fetcher->value().get()->turret()->position();
  }

  for (size_t tag = 0; tag < april_ids.size(); tag++) {
    Eigen::Translation3d T_camera_target = Eigen::Translation3d(
        tvecs_eigen[tag][0], tvecs_eigen[tag][1], tvecs_eigen[tag][2]);
    Eigen::AngleAxisd r_angle = Eigen::AngleAxisd(
        rvecs_eigen[tag].norm(), rvecs_eigen[tag] / rvecs_eigen[tag].norm());
    CHECK(rvecs_eigen[tag].norm() != 0) << "rvecs norm = 0; divide by 0";
    Eigen::Affine3d H_camcv_target = T_camera_target * r_angle;

    Eigen::Affine3d H_robot_target = CameraToRobotDetection(
        H_camcv_target, fixed_extrinsics, turret_extrinsics, turret_position);

    timestamped_target_detections->emplace_back(
        DataAdapter::TimestampedDetection{.time = pi_distributed_time,
                                          .H_robot_target = H_robot_target,
                                          .id = april_ids[tag][0]});
  }
}

Eigen::Affine3d CameraFixedExtrinsics(
    const calibration::CameraCalibration *camera_calibration) {
  cv::Mat extrinsics(
      4, 4, CV_32F,
      const_cast<void *>(static_cast<const void *>(
          camera_calibration->fixed_extrinsics()->data()->data())));
  extrinsics.convertTo(extrinsics, CV_64F);
  CHECK_EQ(extrinsics.total(),
           camera_calibration->fixed_extrinsics()->data()->size());
  Eigen::Matrix4d extrinsics_eigen;
  cv::cv2eigen(extrinsics, extrinsics_eigen);
  return Eigen::Affine3d(extrinsics_eigen);
}

Eigen::Affine3d CameraTurretExtrinsics(
    const calibration::CameraCalibration *camera_calibration) {
  cv::Mat extrinsics(
      4, 4, CV_32F,
      const_cast<void *>(static_cast<const void *>(
          camera_calibration->turret_extrinsics()->data()->data())));
  extrinsics.convertTo(extrinsics, CV_64F);
  CHECK_EQ(extrinsics.total(),
           camera_calibration->turret_extrinsics()->data()->size());
  Eigen::Matrix4d extrinsics_eigen;
  cv::cv2eigen(extrinsics, extrinsics_eigen);
  return Eigen::Affine3d(extrinsics_eigen);
}

// Get images from pi and pass apriltag positions to HandleAprilTag()
void HandlePiCaptures(
    int pi_number, aos::EventLoop *pi_event_loop,
    aos::logger::LogReader *reader,
    std::optional<aos::Fetcher<superstructure::Status>>
        *superstructure_status_fetcher,
    std::vector<DataAdapter::TimestampedDetection>
        *timestamped_target_detections,
    std::vector<std::unique_ptr<CharucoExtractor>> *charuco_extractors,
    std::vector<std::unique_ptr<ImageCallback>> *image_callbacks) {
  const aos::FlatbufferSpan<calibration::CalibrationData> calibration_data(
      CalibrationData());
  const calibration::CameraCalibration *calibration =
      CameraReader::FindCameraCalibration(&calibration_data.message(),
                                          "pi" + std::to_string(pi_number),
                                          FLAGS_team_number);
  const auto turret_extrinsics = CameraTurretExtrinsics(calibration);
  const auto fixed_extrinsics = CameraFixedExtrinsics(calibration);

  charuco_extractors->emplace_back(std::make_unique<CharucoExtractor>(
      pi_event_loop,
      "pi-" + std::to_string(FLAGS_team_number) + "-" +
          std::to_string(pi_number),
      [=](cv::Mat /*rgb_image*/, aos::monotonic_clock::time_point eof,
          std::vector<cv::Vec4i> april_ids,
          std::vector<std::vector<cv::Point2f>> /*april_corners*/, bool valid,
          std::vector<Eigen::Vector3d> rvecs_eigen,
          std::vector<Eigen::Vector3d> tvecs_eigen) {
        aos::distributed_clock::time_point pi_distributed_time =
            reader->event_loop_factory()
                ->GetNodeEventLoopFactory(pi_event_loop->node())
                ->ToDistributedClock(eof);

        if (valid) {
          HandleAprilTag(pi_distributed_time, april_ids, rvecs_eigen,
                         tvecs_eigen, timestamped_target_detections,
                         superstructure_status_fetcher, fixed_extrinsics,
                         turret_extrinsics);
        }
      }));

  std::string channel =
      absl::StrCat("/pi", std::to_string(pi_number), "/camera");

  image_callbacks->emplace_back(std::make_unique<ImageCallback>(
      pi_event_loop, "/pi" + std::to_string(pi_number) + "/camera",
      [&, charuco_extractor =
              charuco_extractors->at(charuco_extractors->size() - 1).get()](
          cv::Mat rgb_image, const aos::monotonic_clock::time_point eof) {
        charuco_extractor->HandleImage(rgb_image, eof);
      }));
}

void MappingMain(int argc, char *argv[]) {
  std::vector<std::string> unsorted_logfiles =
      aos::logger::FindLogs(argc, argv);

  std::vector<DataAdapter::TimestampedPose> timestamped_robot_poses;
  std::vector<DataAdapter::TimestampedDetection> timestamped_target_detections;

  // open logfiles
  aos::logger::LogReader reader(aos::logger::SortParts(unsorted_logfiles));
  reader.Register();

  std::optional<aos::Fetcher<superstructure::Status>>
      superstructure_status_fetcher;

  if (FLAGS_use_robot_position) {
    const aos::Node *imu_node =
        aos::configuration::GetNode(reader.configuration(), "imu");
    std::unique_ptr<aos::EventLoop> imu_event_loop =
        reader.event_loop_factory()->MakeEventLoop("imu", imu_node);

    imu_event_loop->MakeWatcher(
        "/localizer", [&](const frc971::controls::LocalizerOutput &localizer) {
          aos::monotonic_clock::time_point imu_monotonic_time =
              aos::monotonic_clock::time_point(aos::monotonic_clock::duration(
                  localizer.monotonic_timestamp_ns()));
          aos::distributed_clock::time_point imu_distributed_time =
              reader.event_loop_factory()
                  ->GetNodeEventLoopFactory(imu_node)
                  ->ToDistributedClock(imu_monotonic_time);

          timestamped_robot_poses.emplace_back(DataAdapter::TimestampedPose{
              .time = imu_distributed_time,
              .pose =
                  ceres::examples::Pose2d{.x = localizer.x(),
                                          .y = localizer.y(),
                                          .yaw_radians = localizer.theta()}});
        });

    const aos::Node *roborio_node =
        aos::configuration::GetNode(reader.configuration(), "roborio");
    std::unique_ptr<aos::EventLoop> roborio_event_loop =
        reader.event_loop_factory()->MakeEventLoop("roborio", roborio_node);

    superstructure_status_fetcher =
        roborio_event_loop->MakeFetcher<superstructure::Status>(
            "/superstructure");
  }

  // Override target_type to AprilTag, since that's what we're using here
  FLAGS_target_type = "april_tag";

  std::vector<std::unique_ptr<CharucoExtractor>> charuco_extractors;
  std::vector<std::unique_ptr<ImageCallback>> image_callbacks;

  const aos::Node *pi1 =
      aos::configuration::GetNode(reader.configuration(), "pi1");
  std::unique_ptr<aos::EventLoop> pi1_event_loop =
      reader.event_loop_factory()->MakeEventLoop("pi1", pi1);
  HandlePiCaptures(
      1, pi1_event_loop.get(), &reader, &superstructure_status_fetcher,
      &timestamped_target_detections, &charuco_extractors, &image_callbacks);

  const aos::Node *pi2 =
      aos::configuration::GetNode(reader.configuration(), "pi2");
  std::unique_ptr<aos::EventLoop> pi2_event_loop =
      reader.event_loop_factory()->MakeEventLoop("pi2", pi2);
  HandlePiCaptures(
      2, pi2_event_loop.get(), &reader, &superstructure_status_fetcher,
      &timestamped_target_detections, &charuco_extractors, &image_callbacks);

  const aos::Node *pi3 =
      aos::configuration::GetNode(reader.configuration(), "pi3");
  std::unique_ptr<aos::EventLoop> pi3_event_loop =
      reader.event_loop_factory()->MakeEventLoop("pi3", pi3);
  HandlePiCaptures(
      3, pi3_event_loop.get(), &reader, &superstructure_status_fetcher,
      &timestamped_target_detections, &charuco_extractors, &image_callbacks);

  const aos::Node *pi4 =
      aos::configuration::GetNode(reader.configuration(), "pi4");
  std::unique_ptr<aos::EventLoop> pi4_event_loop =
      reader.event_loop_factory()->MakeEventLoop("pi4", pi4);
  HandlePiCaptures(
      4, pi4_event_loop.get(), &reader, &superstructure_status_fetcher,
      &timestamped_target_detections, &charuco_extractors, &image_callbacks);

  reader.event_loop_factory()->Run();

  auto target_constraints =
      (FLAGS_use_robot_position
           ? DataAdapter::MatchTargetDetections(timestamped_robot_poses,
                                                timestamped_target_detections)
                 .first
           : DataAdapter::MatchTargetDetections(timestamped_target_detections));

  frc971::vision::TargetMapper mapper(FLAGS_json_path, target_constraints);
  mapper.Solve("rapid_react");

  // Pointers need to be deleted to destruct all fetchers
  for (auto &charuco_extractor_ptr : charuco_extractors) {
    charuco_extractor_ptr.reset();
  }

  for (auto &image_callback_ptr : image_callbacks) {
    image_callback_ptr.reset();
  }
}

}  // namespace vision
}  // namespace y2022

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  y2022::vision::MappingMain(argc, argv);
}

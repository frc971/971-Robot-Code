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
#include "y2022/vision/camera_reader.h"

DEFINE_string(json_path, "target_map.json",
              "Specify path for json with initial pose guesses.");
DEFINE_int32(team_number, 7971,
             "Use the calibration for a node with this team number");

DECLARE_string(image_channel);

namespace y2022 {
namespace vision {
using frc971::vision::DataAdapter;
using frc971::vision::PoseUtils;
using frc971::vision::TargetMapper;

// Change reference frame from camera to robot
Eigen::Affine3d CameraToRobotDetection(Eigen::Affine3d H_camrob_target,
                                       Eigen::Affine3d extrinsics) {
  const Eigen::Affine3d H_robot_camrob = extrinsics;
  const Eigen::Affine3d H_robot_target = H_robot_camrob * H_camrob_target;
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
                    Eigen::Affine3d extrinsics) {
  for (size_t tag = 0; tag < april_ids.size(); tag++) {
    Eigen::Translation3d T_camera_target = Eigen::Translation3d(
        tvecs_eigen[tag][0], tvecs_eigen[tag][1], tvecs_eigen[tag][2]);
    Eigen::AngleAxisd r_angle = Eigen::AngleAxisd(
        rvecs_eigen[tag].norm(), rvecs_eigen[tag] / rvecs_eigen[tag].norm());
    CHECK(rvecs_eigen[tag].norm() != 0) << "rvecs norm = 0; divide by 0";

    Eigen::Affine3d H_camcv_target = T_camera_target * r_angle;
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

    timestamped_target_detections->emplace_back(
        DataAdapter::TimestampedDetection{
            .time = pi_distributed_time,
            .H_robot_target = H_robot_target,
            .distance_from_camera = distance_from_camera,
            .id = april_ids[tag][0]});
  }
}

Eigen::Affine3d CameraExtrinsics(
    const calibration::CameraCalibration *camera_calibration) {
  cv::Mat extrinsics = CameraReader::CameraExtrinsics(camera_calibration);
  Eigen::Matrix4d extrinsics_eigen;
  cv::cv2eigen(extrinsics, extrinsics_eigen);
  return Eigen::Affine3d(extrinsics_eigen);
}

// Get images from pi and pass apriltag positions to HandleAprilTag()
void HandlePiCaptures(
    int pi_number, aos::EventLoop *pi_event_loop,
    aos::logger::LogReader *reader,
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
  const auto extrinsics = CameraExtrinsics(calibration);

  // TODO(milind): change to /camera once we log at full frequency
  static constexpr std::string_view kImageChannel = "/camera/decimated";
  charuco_extractors->emplace_back(std::make_unique<CharucoExtractor>(
      pi_event_loop,
      "pi-" + std::to_string(FLAGS_team_number) + "-" +
          std::to_string(pi_number),
      TargetType::kAprilTag, kImageChannel,
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
                         extrinsics);
        }
      }));

  image_callbacks->emplace_back(std::make_unique<ImageCallback>(
      pi_event_loop, kImageChannel,
      [&, charuco_extractor =
              charuco_extractors->at(charuco_extractors->size() - 1).get()](
          cv::Mat rgb_image, const aos::monotonic_clock::time_point eof) {
        charuco_extractor->HandleImage(rgb_image, eof);
      }));
}

void MappingMain(int argc, char *argv[]) {
  std::vector<std::string> unsorted_logfiles =
      aos::logger::FindLogs(argc, argv);

  std::vector<DataAdapter::TimestampedDetection> timestamped_target_detections;

  // open logfiles
  aos::logger::LogReader reader(aos::logger::SortParts(unsorted_logfiles));
  reader.Register();

  std::vector<std::unique_ptr<CharucoExtractor>> charuco_extractors;
  std::vector<std::unique_ptr<ImageCallback>> image_callbacks;

  const aos::Node *pi1 =
      aos::configuration::GetNode(reader.configuration(), "pi1");
  std::unique_ptr<aos::EventLoop> pi1_event_loop =
      reader.event_loop_factory()->MakeEventLoop("pi1", pi1);
  HandlePiCaptures(1, pi1_event_loop.get(), &reader,
                   &timestamped_target_detections, &charuco_extractors,
                   &image_callbacks);

  const aos::Node *pi2 =
      aos::configuration::GetNode(reader.configuration(), "pi2");
  std::unique_ptr<aos::EventLoop> pi2_event_loop =
      reader.event_loop_factory()->MakeEventLoop("pi2", pi2);
  HandlePiCaptures(2, pi2_event_loop.get(), &reader,
                   &timestamped_target_detections, &charuco_extractors,
                   &image_callbacks);

  const aos::Node *pi3 =
      aos::configuration::GetNode(reader.configuration(), "pi3");
  std::unique_ptr<aos::EventLoop> pi3_event_loop =
      reader.event_loop_factory()->MakeEventLoop("pi3", pi3);
  HandlePiCaptures(3, pi3_event_loop.get(), &reader,
                   &timestamped_target_detections, &charuco_extractors,
                   &image_callbacks);

  const aos::Node *pi4 =
      aos::configuration::GetNode(reader.configuration(), "pi4");
  std::unique_ptr<aos::EventLoop> pi4_event_loop =
      reader.event_loop_factory()->MakeEventLoop("pi4", pi4);
  HandlePiCaptures(4, pi4_event_loop.get(), &reader,
                   &timestamped_target_detections, &charuco_extractors,
                   &image_callbacks);

  reader.event_loop_factory()->Run();

  auto target_constraints =
      DataAdapter::MatchTargetDetections(timestamped_target_detections);

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

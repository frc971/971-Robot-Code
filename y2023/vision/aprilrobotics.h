
#include <string>

#include "aos/events/event_loop.h"
#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/network/team_number.h"
#include "aos/realtime.h"
#include "frc971/vision/calibration_generated.h"
#include "frc971/vision/charuco_lib.h"
#include "frc971/vision/target_map_generated.h"
#include "frc971/vision/target_mapper.h"
#include "frc971/vision/vision_generated.h"
#include "opencv2/core/eigen.hpp"
#include "opencv2/imgproc.hpp"
#include "third_party/apriltag/apriltag.h"
#include "third_party/apriltag/apriltag_pose.h"
#include "third_party/apriltag/tag16h5.h"
#include "y2023/vision/calibration_data.h"

DECLARE_int32(team_number);

namespace y2023 {
namespace vision {

class AprilRoboticsDetector {
 public:
  AprilRoboticsDetector(aos::EventLoop *event_loop,
                        std::string_view channel_name);

  ~AprilRoboticsDetector();

  void SetWorkerpoolAffinities();

  std::vector<std::pair<apriltag_detection_t, apriltag_pose_t>> DetectTags(
      cv::Mat image);

 private:
  void HandleImage(cv::Mat image);

  flatbuffers::Offset<frc971::vision::TargetPoseFbs> BuildTargetPose(
      const apriltag_pose_t &pose,
      frc971::vision::TargetMapper::TargetId target_id,
      flatbuffers::FlatBufferBuilder *fbb);

  static const frc971::vision::calibration::CameraCalibration *
  FindCameraCalibration(
      const frc971::vision::calibration::CalibrationData *calibration_data,
      std::string_view node_name) {
    for (const frc971::vision::calibration::CameraCalibration *candidate :
         *calibration_data->camera_calibrations()) {
      if (candidate->node_name()->string_view() != node_name) {
        continue;
      }
      if (candidate->team_number() != FLAGS_team_number) {
        continue;
      }
      return candidate;
    }
    LOG(FATAL) << ": Failed to find camera calibration for " << node_name
               << " on " << FLAGS_team_number;
  }

  static cv::Mat CameraIntrinsics(
      const frc971::vision::calibration::CameraCalibration
          *camera_calibration) {
    cv::Mat result(3, 3, CV_32F,
                   const_cast<void *>(static_cast<const void *>(
                       camera_calibration->intrinsics()->data())));
    result.convertTo(result, CV_64F);
    CHECK_EQ(result.total(), camera_calibration->intrinsics()->size());

    return result;
  }

  static cv::Mat CameraDistCoeffs(
      const frc971::vision::calibration::CameraCalibration
          *camera_calibration) {
    const cv::Mat result(5, 1, CV_32F,
                         const_cast<void *>(static_cast<const void *>(
                             camera_calibration->dist_coeffs()->data())));
    CHECK_EQ(result.total(), camera_calibration->dist_coeffs()->size());
    return result;
  }

  apriltag_family_t *tag_family_;
  apriltag_detector_t *tag_detector_;

  const aos::FlatbufferSpan<frc971::vision::calibration::CalibrationData>
      calibration_data_;
  const frc971::vision::calibration::CameraCalibration *calibration_;
  cv::Mat intrinsics_;
  cv::Mat camera_distortion_coeffs_;

  aos::Ftrace ftrace_;

  frc971::vision::ImageCallback image_callback_;
  aos::Sender<frc971::vision::TargetMap> target_map_sender_;
};

}  // namespace vision
}  // namespace y2023

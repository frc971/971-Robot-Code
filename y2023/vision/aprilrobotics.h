
#include <string>

#include "aos/events/event_loop.h"
#include "aos/events/shm_event_loop.h"
#include "aos/network/team_number.h"
#include "aos/realtime.h"
#include "frc971/constants/constants_sender_lib.h"
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
#include "y2023/constants/constants_generated.h"

namespace y2023 {
namespace vision {

class AprilRoboticsDetector {
 public:
  // Aprilrobotics representation of a tag detection
  struct Detection {
    apriltag_detection_t det;
    apriltag_pose_t pose;
    double pose_error;
    double distortion_factor;
  };

  AprilRoboticsDetector(aos::EventLoop *event_loop,
                        std::string_view channel_name);
  ~AprilRoboticsDetector();

  void SetWorkerpoolAffinities();

  // Undistorts the april tag corners using the camera calibration
  void UndistortDetection(apriltag_detection_t *det) const;

  std::vector<Detection> DetectTags(cv::Mat image,
                                    aos::monotonic_clock::time_point eof);

  const std::optional<cv::Mat> extrinsics() const { return extrinsics_; }
  const cv::Mat intrinsics() const { return intrinsics_; }
  const cv::Mat dist_coeffs() const { return dist_coeffs_; }

 private:
  void HandleImage(cv::Mat image, aos::monotonic_clock::time_point eof);

  flatbuffers::Offset<frc971::vision::TargetPoseFbs> BuildTargetPose(
      const Detection &detection, flatbuffers::FlatBufferBuilder *fbb);

  // Computes the distortion effect on this detection taking the scaled average
  // delta between orig_corners (distorted corners) and corners (undistorted
  // corners)
  double ComputeDistortionFactor(const std::vector<cv::Point2f> &orig_corners,
                                 const std::vector<cv::Point2f> &corners);

  apriltag_family_t *tag_family_;
  apriltag_detector_t *tag_detector_;

  const frc971::constants::ConstantsFetcher<Constants> calibration_data_;
  const frc971::vision::calibration::CameraCalibration *calibration_;
  cv::Mat intrinsics_;
  cv::Mat projection_matrix_;
  std::optional<cv::Mat> extrinsics_;
  cv::Mat dist_coeffs_;
  cv::Size image_size_;

  aos::Ftrace ftrace_;

  frc971::vision::ImageCallback image_callback_;
  aos::Sender<frc971::vision::TargetMap> target_map_sender_;
  aos::Sender<foxglove::ImageAnnotations> image_annotations_sender_;
};

}  // namespace vision
}  // namespace y2023

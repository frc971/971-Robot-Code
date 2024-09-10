
#include <string>

#include "Eigen/Dense"
#include "opencv2/core/eigen.hpp"
#include "opencv2/imgproc.hpp"
#include "third_party/apriltag/apriltag.h"
#include "third_party/apriltag/apriltag_pose.h"
#include "third_party/apriltag/tag16h5.h"
#include "third_party/apriltag/tag36h11.h"

#include "aos/events/event_loop.h"
#include "aos/events/shm_event_loop.h"
#include "aos/network/team_number.h"
#include "aos/realtime.h"
#include "frc971/constants/constants_sender_lib.h"
#include "frc971/orin/apriltag.h"
#include "frc971/vision/calibration_generated.h"
#include "frc971/vision/charuco_lib.h"
#include "frc971/vision/target_map_generated.h"
#include "frc971/vision/vision_generated.h"

namespace frc971 {
namespace apriltag {

class ApriltagDetector {
 public:
  // Aprilrobotics representation of a tag detection
  struct Detection {
    apriltag_detection_t det;
    apriltag_pose_t pose;
    double pose_error;
    double distortion_factor;
    double pose_error_ratio;
  };

  // Class to detect Apriltags in images and publish poses of the tags
  ApriltagDetector(
      aos::EventLoop *event_loop, std::string_view channel_name,
      const frc971::vision::calibration::CameraCalibration *calibration,
      size_t width = 1456, size_t height = 1088);

  ~ApriltagDetector();

  // Creates the GPU-based Apriltag detector.
  apriltag_detector_t *MakeTagDetector(apriltag_family_t *tag_family);

  // Builds the output TargetPose flatbuffer from Apriltag detection.
  flatbuffers::Offset<frc971::vision::TargetPoseFbs> BuildTargetPose(
      const Detection &detection, flatbuffers::FlatBufferBuilder *fbb);

  // Undistorts the april tag corners using the camera calibration.
  // Returns the detections in place.
  // Returns false if any of the corner undistortions fail to converge
  bool UndistortDetection(apriltag_detection_t *det) const;

  // Computes the distortion effect on this detection taking the scaled average
  // delta between orig_corners (distorted corners) and corners (undistorted
  // corners).
  double ComputeDistortionFactor(const std::vector<cv::Point2f> &orig_corners,
                                 const std::vector<cv::Point2f> &corners);

  // Helper function to store detection points in vector of Point2f's.
  std::vector<cv::Point2f> MakeCornerVector(const apriltag_detection_t *det);

  // Deletes the heap-allocated rotation and translation pointers in the given
  // pose.
  void DestroyPose(apriltag_pose_t *pose) const;

  void HandleImage(cv::Mat color_image, aos::monotonic_clock::time_point eof);

 private:
  apriltag_family_t *tag_family_;
  apriltag_detector_t *tag_detector_;
  std::string_view node_name_;

  const frc971::vision::calibration::CameraCalibration *calibration_;
  cv::Mat intrinsics_;
  cv::Mat projection_matrix_;
  std::optional<cv::Mat> extrinsics_;
  cv::Mat dist_coeffs_;

  // The distortion constants passed into gpu_detector_ and used later on to do
  // undistort before passing corners into apriltags pose estimation.
  CameraMatrix distortion_camera_matrix_;
  DistCoeffs distortion_coefficients_;

  frc971::apriltag::GpuDetector<frc971::apriltag::InputFormat::YCbCr422> gpu_detector_;
  cv::Size image_size_;

  frc971::vision::ImageCallback image_callback_;
  aos::Sender<frc971::vision::TargetMap> target_map_sender_;
  aos::Sender<foxglove::ImageAnnotations> image_annotations_sender_;
  size_t rejections_;
};

}  // namespace apriltag
}  // namespace frc971

#ifndef FRC971_VISION_VISION_UTIL_LIB_H_
#define FRC971_VISION_VISION_UTIL_LIB_H_
#include <optional>
#include <string_view>

#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "Eigen/SVD"
#include "opencv2/imgproc.hpp"

#include "frc971/vision/calibration_generated.h"
#include "frc971/vision/ceres/types.h"

// Extract the CameraExtrinsics from a CameraCalibration struct
namespace frc971::vision {
std::optional<cv::Mat> CameraExtrinsics(
    const frc971::vision::calibration::CameraCalibration *camera_calibration);

// Extract the CameraIntrinsics from a CameraCalibration struct
cv::Mat CameraIntrinsics(
    const frc971::vision::calibration::CameraCalibration *camera_calibration);

// Extract the CameraDistCoeffs from a CameraCalibration struct
cv::Mat CameraDistCoeffs(
    const frc971::vision::calibration::CameraCalibration *camera_calibration);

// Get the camera number from a camera channel name, e.g., return 2 from
// "/camera2".  Returns nullopt if string doesn't start with "/camera" or does
// not have a number
std::optional<uint16_t> CameraNumberFromChannel(std::string camera_channel);

// Return a calibration filename to save to based on the given data
std::string CalibrationFilename(std::string calibration_folder,
                                std::string node_name, int team_number,
                                int camera_number, std::string camera_id,
                                std::string timestamp);

// Utility functions for dealing with ceres::examples::Pose3d structs
class PoseUtils {
 public:
  // Embeds a 3d pose into an affine transformation
  static Eigen::Affine3d Pose3dToAffine3d(
      const ceres::examples::Pose3d &pose3d);
  // Inverse of above function
  static ceres::examples::Pose3d Affine3dToPose3d(const Eigen::Affine3d &H);

  // Computes pose_2 relative to pose_1. This is equivalent to (pose_1^-1 *
  // pose_2)
  static ceres::examples::Pose3d ComputeRelativePose(
      const ceres::examples::Pose3d &pose_1,
      const ceres::examples::Pose3d &pose_2);

  // Computes pose_2 given a pose_1 and pose_2 relative to pose_1. This is
  // equivalent to (pose_1 * pose_2_relative)
  static ceres::examples::Pose3d ComputeOffsetPose(
      const ceres::examples::Pose3d &pose_1,
      const ceres::examples::Pose3d &pose_2_relative);

  // Converts a rotation with roll, pitch, and yaw into a quaternion
  static Eigen::Quaterniond EulerAnglesToQuaternion(const Eigen::Vector3d &rpy);
  // Inverse of above function
  static Eigen::Vector3d QuaternionToEulerAngles(const Eigen::Quaterniond &q);
  // Converts a 3d rotation matrix into a rotation with roll, pitch, and yaw
  static Eigen::Vector3d RotationMatrixToEulerAngles(const Eigen::Matrix3d &R);
};

// Compute the average of a set of quaternions
Eigen::Vector4d QuaternionAverage(std::vector<Eigen::Vector4d> quaternions);

// Compute the average pose given a list of translations (as
// Eigen::Vector3d's) and rotations (as Eigen::Vector4d quaternions)
// Returned as an Eigen::Affine3d pose
// Also, compute the variance of each of these list of vectors
Eigen::Affine3d ComputeAveragePose(
    std::vector<Eigen::Vector3d> &translation_list,
    std::vector<Eigen::Vector4d> &rotation_list,
    Eigen::Vector3d *translation_variance = nullptr,
    Eigen::Vector3d *rotation_variance = nullptr);

// Helper function to compute average pose when supplied with list
// of Eigen::Affine3d's
Eigen::Affine3d ComputeAveragePose(
    std::vector<Eigen::Affine3d> &pose_list,
    Eigen::Vector3d *translation_variance = nullptr,
    Eigen::Vector3d *rotation_variance = nullptr);

}  // namespace frc971::vision

#endif  // FRC971_VISION_VISION_UTIL_LIB_H_

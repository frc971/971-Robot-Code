#include "frc971/vision/vision_util_lib.h"

#include <numeric>

#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/strings/str_format.h"

#include "aos/util/math.h"
#include "frc971/control_loops/quaternion_utils.h"

namespace frc971::vision {

std::optional<cv::Mat> CameraExtrinsics(
    const frc971::vision::calibration::CameraCalibration *camera_calibration) {
  CHECK(!camera_calibration->has_turret_extrinsics())
      << "Turret not currently supported";

  if (!camera_calibration->has_fixed_extrinsics()) {
    return std::nullopt;
  }
  CHECK(camera_calibration->fixed_extrinsics()->has_data());
  cv::Mat result(4, 4, CV_32F,
                 const_cast<void *>(static_cast<const void *>(
                     camera_calibration->fixed_extrinsics()->data()->data())));
  result.convertTo(result, CV_64F);
  CHECK_EQ(result.total(),
           camera_calibration->fixed_extrinsics()->data()->size());

  return result;
}

cv::Mat CameraIntrinsics(
    const frc971::vision::calibration::CameraCalibration *camera_calibration) {
  cv::Mat result(3, 3, CV_32F,
                 const_cast<void *>(static_cast<const void *>(
                     camera_calibration->intrinsics()->data())));
  CHECK_EQ(result.total(), camera_calibration->intrinsics()->size());

  return result;
}

cv::Mat CameraDistCoeffs(
    const frc971::vision::calibration::CameraCalibration *camera_calibration) {
  const cv::Mat result(camera_calibration->dist_coeffs()->size(), 1, CV_32F,
                       const_cast<void *>(static_cast<const void *>(
                           camera_calibration->dist_coeffs()->data())));
  CHECK_EQ(result.total(), camera_calibration->dist_coeffs()->size());
  return result;
}

std::optional<uint16_t> CameraNumberFromChannel(std::string camera_channel) {
  if (camera_channel.find("/camera") == std::string::npos) {
    return std::nullopt;
  }
  // If the string doesn't end in /camera#, return nullopt
  uint16_t cam_len = std::string("/camera").length();
  if (camera_channel.length() != camera_channel.find("/camera") + cam_len + 1) {
    return std::nullopt;
  }

  uint16_t camera_number = std::stoi(
      camera_channel.substr(camera_channel.find("/camera") + cam_len, 1));
  return camera_number;
}

std::string CalibrationFilename(std::string calibration_folder,
                                std::string node_name, int team_number,
                                int camera_number, std::string camera_id,
                                std::string timestamp) {
  // Get rid of any fractional seconds-- we shouldn't need those and it makes
  // the string unnecessarily longer
  timestamp = timestamp.substr(0, timestamp.find("."));
  std::string calibration_filename =
      calibration_folder +
      absl::StrFormat("/calibration_%s-%d-%d_cam-%s_%s.json", node_name.c_str(),
                      team_number, camera_number, camera_id.c_str(),
                      timestamp.c_str());
  return calibration_filename;
}

Eigen::Affine3d PoseUtils::Pose3dToAffine3d(
    const ceres::examples::Pose3d &pose3d) {
  Eigen::Affine3d H_world_pose =
      Eigen::Translation3d(pose3d.p(0), pose3d.p(1), pose3d.p(2)) * pose3d.q;
  return H_world_pose;
}

ceres::examples::Pose3d PoseUtils::Affine3dToPose3d(const Eigen::Affine3d &H) {
  return ceres::examples::Pose3d{.p = H.translation(),
                                 .q = Eigen::Quaterniond(H.rotation())};
}

ceres::examples::Pose3d PoseUtils::ComputeRelativePose(
    const ceres::examples::Pose3d &pose_1,
    const ceres::examples::Pose3d &pose_2) {
  Eigen::Affine3d H_world_1 = Pose3dToAffine3d(pose_1);
  Eigen::Affine3d H_world_2 = Pose3dToAffine3d(pose_2);

  // Get the location of 2 in the 1 frame
  Eigen::Affine3d H_1_2 = H_world_1.inverse() * H_world_2;
  return Affine3dToPose3d(H_1_2);
}

ceres::examples::Pose3d PoseUtils::ComputeOffsetPose(
    const ceres::examples::Pose3d &pose_1,
    const ceres::examples::Pose3d &pose_2_relative) {
  auto H_world_1 = Pose3dToAffine3d(pose_1);
  auto H_1_2 = Pose3dToAffine3d(pose_2_relative);
  auto H_world_2 = H_world_1 * H_1_2;

  return Affine3dToPose3d(H_world_2);
}

Eigen::Quaterniond PoseUtils::EulerAnglesToQuaternion(
    const Eigen::Vector3d &rpy) {
  Eigen::AngleAxisd roll_angle(rpy.x(), Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitch_angle(rpy.y(), Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yaw_angle(rpy.z(), Eigen::Vector3d::UnitZ());

  return yaw_angle * pitch_angle * roll_angle;
}

Eigen::Vector3d PoseUtils::QuaternionToEulerAngles(
    const Eigen::Quaterniond &q) {
  return RotationMatrixToEulerAngles(q.toRotationMatrix());
}

Eigen::Vector3d PoseUtils::RotationMatrixToEulerAngles(
    const Eigen::Matrix3d &R) {
  double roll = aos::math::NormalizeAngle(std::atan2(R(2, 1), R(2, 2)));
  double pitch = aos::math::NormalizeAngle(-std::asin(R(2, 0)));
  double yaw = aos::math::NormalizeAngle(std::atan2(R(1, 0), R(0, 0)));

  return Eigen::Vector3d(roll, pitch, yaw);
}

// Compute the average pose given a list of translations (as
// Eigen::Vector3d's) and rotations (as Eigen::Vector4d quaternions)
// Returned as an Eigen::Affine3d pose
// Also, compute the variance of each of these list of vectors

// NOTE: variance for rotations can get dicey, so we've cheated a
// little by doing two things:
// 1) Computing variance relative to the mean, so that we're doing
// this on small angles and don't have to deal with wrapping at
// 180/360 degrees
// 2) Returning the variance in Euler angles, since I'm not sure of a
// better way to represent variance for rotations.  (Maybe log of
// rotations, in the Lie algebra?)

Eigen::Affine3d ComputeAveragePose(
    std::vector<Eigen::Vector3d> &translation_list,
    std::vector<Eigen::Vector4d> &rotation_list,
    Eigen::Vector3d *translation_variance, Eigen::Vector3d *rotation_variance) {
  Eigen::Vector3d avg_translation =
      std::accumulate(translation_list.begin(), translation_list.end(),
                      Eigen::Vector3d{0, 0, 0}) /
      translation_list.size();

  Eigen::Quaterniond avg_rotation_q(
      frc971::controls::QuaternionMean(rotation_list));
  Eigen::Affine3d average_pose =
      Eigen::Translation3d(avg_translation) * avg_rotation_q;

  CHECK_EQ(translation_list.size(), rotation_list.size());
  if (translation_variance != nullptr) {
    CHECK(rotation_variance != nullptr);
    Eigen::Vector3d translation_variance_sum(0.0, 0.0, 0.0);
    Eigen::Vector3d rotation_variance_sum(0.0, 0.0, 0.0);
    for (uint i = 0; i < translation_list.size(); i++) {
      Eigen::Quaterniond rotation_q(rotation_list[i]);
      Eigen::Affine3d pose =
          Eigen::Translation3d(translation_list[i]) * rotation_q;
      Eigen::Affine3d delta_pose = average_pose * pose.inverse();
      translation_variance_sum =
          translation_variance_sum +
          Eigen::Vector3d(delta_pose.translation().array().square());
      rotation_variance_sum =
          rotation_variance_sum +
          Eigen::Vector3d(PoseUtils::RotationMatrixToEulerAngles(
                              delta_pose.rotation().matrix())
                              .array()
                              .square());
    }
    // Compute the variance on the translations (in m)
    if (translation_variance != nullptr) {
      CHECK(translation_list.size() > 1)
          << "Have to have at least two translations to compute variance";
      *translation_variance =
          translation_variance_sum / translation_list.size();
    }

    // Compute the variance on the rotations (in euler angles, radians),
    // referenced to the mean, to remove issues with Euler angles by
    // keeping them near zero
    if (rotation_variance != nullptr) {
      CHECK(rotation_list.size() > 1)
          << "Have to have at least two rotations to compute variance";
      *rotation_variance = rotation_variance_sum / rotation_list.size();
    }
  }
  return average_pose;
}

// Helper function to compute average pose when supplied with list
// of Eigen::Affine3d's
Eigen::Affine3d ComputeAveragePose(std::vector<Eigen::Affine3d> &pose_list,
                                   Eigen::Vector3d *translation_variance,
                                   Eigen::Vector3d *rotation_variance) {
  std::vector<Eigen::Vector3d> translation_list;
  std::vector<Eigen::Vector4d> rotation_list;

  for (Eigen::Affine3d pose : pose_list) {
    translation_list.push_back(pose.translation());
    Eigen::Quaterniond quat(pose.rotation().matrix());
    rotation_list.push_back(
        Eigen::Vector4d(quat.x(), quat.y(), quat.z(), quat.w()));
  }

  return ComputeAveragePose(translation_list, rotation_list,
                            translation_variance, rotation_variance);
}

}  // namespace frc971::vision

#ifndef FRC971_VISION_TARGET_MAPPER_H_
#define FRC971_VISION_TARGET_MAPPER_H_

#include <unordered_map>

#include "aos/events/simulated_event_loop.h"
#include "ceres/ceres.h"
#include "frc971/vision/ceres/types.h"
#include "frc971/vision/target_map_generated.h"
#include "frc971/vision/visualize_robot.h"

namespace frc971::vision {

// Estimates positions of vision targets (ex. April Tags) using
// target detections relative to a robot (which were computed using robot
// positions at the time of those detections). Solves SLAM problem to estimate
// target locations using deltas between consecutive target detections.
class TargetMapper {
 public:
  using TargetId = int;
  using ConfidenceMatrix = Eigen::Matrix<double, 6, 6>;

  struct TargetPose {
    TargetId id;
    ceres::examples::Pose3d pose;
  };

  // target_poses_path is the path to a TargetMap json with initial guesses for
  // the actual locations of the targets on the field.
  // target_constraints are the deltas between consecutive target detections,
  // and are usually prepared by the DataAdapter class below.
  TargetMapper(std::string_view target_poses_path,
               const ceres::examples::VectorOfConstraints &target_constraints);
  // Alternate constructor for tests.
  // Takes in the actual intial guesses instead of a file containing them
  TargetMapper(const ceres::examples::MapOfPoses &target_poses,
               const ceres::examples::VectorOfConstraints &target_constraints);

  // Solves for the target map. If output_dir is set, the map will be saved to
  // output_dir/field_name.json
  void Solve(std::string_view field_name,
             std::optional<std::string_view> output_dir = std::nullopt);

  // Prints target poses into a TargetMap flatbuffer json
  std::string MapToJson(std::string_view field_name) const;

  static std::optional<TargetPose> GetTargetPoseById(
      std::vector<TargetPose> target_poses, TargetId target_id);

  // Version that gets based on internal target_poses
  std::optional<TargetPose> GetTargetPoseById(TargetId target_id) const;

  ceres::examples::MapOfPoses target_poses() { return target_poses_; }

  // Cost function for the secondary solver finding out where the whole map fits
  // in the world
  template <typename S>
  bool operator()(const S *const translation, const S *const rotation,
                  S *residual) const;

 private:
  // Constructs the nonlinear least squares optimization problem from the
  // pose graph constraints.
  void BuildTargetPoseOptimizationProblem(
      const ceres::examples::VectorOfConstraints &constraints,
      ceres::examples::MapOfPoses *poses, ceres::Problem *problem);

  // Constructs the nonlinear least squares optimization problem for the solved
  // -> actual pose solver.
  void BuildMapFittingOptimizationProblem(ceres::Problem *problem);

  // Returns true if the solve was successful.
  bool SolveOptimizationProblem(ceres::Problem *problem);

  ceres::examples::MapOfPoses ideal_target_poses_;
  ceres::examples::MapOfPoses target_poses_;
  ceres::examples::VectorOfConstraints target_constraints_;

  // Transformation moving the target map we solved for to where it actually
  // should be in the world
  Eigen::Translation3d T_frozen_actual_;
  Eigen::Quaterniond R_frozen_actual_;

  mutable VisualizeRobot vis_robot_;
};

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

  // Builds a TargetPoseFbs from a TargetPose
  static flatbuffers::Offset<TargetPoseFbs> TargetPoseToFbs(
      const TargetMapper::TargetPose &target_pose,
      flatbuffers::FlatBufferBuilder *fbb);
  // Converts a TargetPoseFbs to a TargetPose
  static TargetMapper::TargetPose TargetPoseFromFbs(
      const TargetPoseFbs &target_pose_fbs);
};

// Transforms robot position and target detection data into target constraints
// to be used for mapping.
class DataAdapter {
 public:
  // Pairs target detection with a time point
  struct TimestampedDetection {
    aos::distributed_clock::time_point time;
    // Pose of target relative to robot
    Eigen::Affine3d H_robot_target;
    // Horizontal distance from camera to target, used for confidence
    // calculation
    double distance_from_camera;
    // A measure of how much distortion affected this detection from 0-1.
    double distortion_factor;
    TargetMapper::TargetId id;
  };

  // Pairs consecutive target detections that are not too far apart in time into
  // constraints. Meant to be used on a system without a position measurement.
  // Assumes timestamped_target_detections is in chronological order.
  // max_dt is the maximum time between two target detections to match them up.
  // If too much time passes, the recoding device (box of pis) could have moved
  // too much
  static ceres::examples::VectorOfConstraints MatchTargetDetections(
      const std::vector<TimestampedDetection> &timestamped_target_detections,
      aos::distributed_clock::duration max_dt = std::chrono::milliseconds(10));

  // Computes inverse of covariance matrix, assuming there was a target
  // detection between robot movement over the given time period. Ceres calls
  // this matrix the "information"
  static TargetMapper::ConfidenceMatrix ComputeConfidence(
      const TimestampedDetection &detection_start,
      const TimestampedDetection &detection_end);

  // Computes the constraint between the start and end pose of the targets: the
  // relative pose between the start and end target locations in the frame of
  // the start target.
  static ceres::examples::Constraint3d ComputeTargetConstraint(
      const TimestampedDetection &target_detection_start,
      const TimestampedDetection &target_detection_end,
      const TargetMapper::ConfidenceMatrix &confidence);
};

}  // namespace frc971::vision

std::ostream &operator<<(std::ostream &os, ceres::examples::Pose3d pose);
std::ostream &operator<<(std::ostream &os,
                         ceres::examples::Constraint3d constraint);

#endif  // FRC971_VISION_TARGET_MAPPER_H_

#ifndef FRC971_VISION_TARGET_MAPPER_H_
#define FRC971_VISION_TARGET_MAPPER_H_

#include <unordered_map>

#include "aos/events/simulated_event_loop.h"
#include "frc971/vision/ceres/types.h"
#include "frc971/vision/target_map_generated.h"

namespace frc971::vision {

// Estimates positions of vision targets (ex. April Tags) using
// target detections relative to a robot (which were computed using robot
// positions at the time of those detections). Solves SLAM problem to estimate
// target locations using deltas between consecutive target detections.
class TargetMapper {
 public:
  using TargetId = int;

  struct TargetPose {
    TargetId id;
    // TOOD(milind): switch everything to 3d once we're more confident in 2d
    // solving
    ceres::examples::Pose2d pose;
  };

  // target_poses_path is the path to a TargetMap json with initial guesses for
  // the actual locations of the targets on the field.
  // target_constraints are the deltas between consecutive target detections,
  // and are usually prepared by the DataAdapter class below.
  TargetMapper(std::string_view target_poses_path,
               std::vector<ceres::examples::Constraint2d> target_constraints);
  // Alternate constructor for tests.
  // Takes in the actual intial guesses instead of a file containing them
  TargetMapper(std::map<TargetId, ceres::examples::Pose2d> target_poses,
               std::vector<ceres::examples::Constraint2d> target_constraints);

  // Solves for the target map. If output_dir is set, the map will be saved to
  // output_dir/field_name.json
  void Solve(std::string_view field_name,
             std::optional<std::string_view> output_dir = std::nullopt);

  // Prints target poses into a TargetMap flatbuffer json
  std::string MapToJson(std::string_view field_name) const;

  static std::optional<TargetPose> GetTargetPoseById(
      std::vector<TargetPose> target_poses, TargetId target_id);

  std::map<TargetId, ceres::examples::Pose2d> target_poses() {
    return target_poses_;
  }

 private:
  // Constructs the nonlinear least squares optimization problem from the
  // pose graph constraints.
  void BuildOptimizationProblem(
      std::map<TargetId, ceres::examples::Pose2d> *target_poses,
      const std::vector<ceres::examples::Constraint2d> &constraints,
      ceres::Problem *problem);

  // Returns true if the solve was successful.
  bool SolveOptimizationProblem(ceres::Problem *problem);

  std::map<TargetId, ceres::examples::Pose2d> target_poses_;
  std::vector<ceres::examples::Constraint2d> target_constraints_;
};

// Utility functions for dealing with ceres::examples::Pose2d structs
class PoseUtils {
 public:
  // Embeds a 2d pose into a 3d affine transformation to be used in 3d
  // computation
  static Eigen::Affine3d Pose2dToAffine3d(ceres::examples::Pose2d pose2d);
  // Assumes only x and y translation, and only z rotation (yaw)
  static ceres::examples::Pose2d Affine3dToPose2d(Eigen::Affine3d H);

  // Computes pose_2 relative to pose_1. This is equivalent to (pose_1^-1 *
  // pose_2)
  static ceres::examples::Pose2d ComputeRelativePose(
      ceres::examples::Pose2d pose_1, ceres::examples::Pose2d pose_2);

  // Computes pose_2 given a pose_1 and pose_2 relative to pose_1. This is
  // equivalent to (pose_1 * pose_2_relative)
  static ceres::examples::Pose2d ComputeOffsetPose(
      ceres::examples::Pose2d pose_1, ceres::examples::Pose2d pose_2_relative);
};

// Transforms robot position and target detection data into target constraints
// to be used for mapping. Interpolates continous-time data, converting it to
// discrete detection time steps.
class DataAdapter {
 public:
  // Pairs pose with a time point
  struct TimestampedPose {
    aos::distributed_clock::time_point time;
    ceres::examples::Pose2d pose;
  };

  // Pairs target detection with a time point
  struct TimestampedDetection {
    aos::distributed_clock::time_point time;
    // Pose of target relative to robot
    Eigen::Affine3d H_robot_target;
    TargetMapper::TargetId id;
  };

  // Pairs consecutive target detections into constraints, and interpolates
  // robot poses based on time points to compute motion between detections. This
  // prepares data to be used by TargetMapper. Also returns vector of delta
  // robot poses corresponding to each constraint, to be used for testing.
  //
  // Assumes both inputs are in chronological order.
  static std::pair<std::vector<ceres::examples::Constraint2d>,
                   std::vector<ceres::examples::Pose2d>>
  MatchTargetDetections(
      const std::vector<TimestampedPose> &timestamped_robot_poses,
      const std::vector<TimestampedDetection> &timestamped_target_detections);

  // Pairs consecutive target detections that are not too far apart in time into
  // constraints. Meant to be used on a system without a position measurement.
  // Assumes timestamped_target_detections is in chronological order.
  // max_dt is the maximum time between two target detections to match them up.
  // If too much time passes, the recoding device (box of pis) could have moved
  // too much
  static std::vector<ceres::examples::Constraint2d> MatchTargetDetections(
      const std::vector<TimestampedDetection> &timestamped_target_detections,
      aos::distributed_clock::duration max_dt = std::chrono::milliseconds(1));

  // Computes inverse of covariance matrix, assuming there was a target
  // detection between robot movement over the given time period. Ceres calls
  // this matrix the "information"
  static Eigen::Matrix3d ComputeConfidence(
      aos::distributed_clock::time_point start,
      aos::distributed_clock::time_point end);

 private:
  static ceres::examples::Pose2d InterpolatePose(
      const TimestampedPose &pose_start, const TimestampedPose &pose_end,
      aos::distributed_clock::time_point time);

  // Computes the constraint between the start and end pose of the targets: the
  // relative pose between the start and end target locations in the frame of
  // the start target. Takes into account the robot motion in the time between
  // the two detections.
  static ceres::examples::Constraint2d ComputeTargetConstraint(
      const TimestampedDetection &target_detection_start,
      const Eigen::Affine3d &H_robotstart_robotend,
      const TimestampedDetection &target_detection_end,
      const Eigen::Matrix3d &confidence);
  // Same as above function, but assumes no robot motion between the two
  // detections
  static ceres::examples::Constraint2d ComputeTargetConstraint(
      const TimestampedDetection &target_detection_start,
      const TimestampedDetection &target_detection_end,
      const Eigen::Matrix3d &confidence);
};

}  // namespace frc971::vision

#endif  // FRC971_VISION_TARGET_MAPPER_H_

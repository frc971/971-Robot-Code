#include "frc971/vision/target_mapper.h"

#include "absl/strings/str_format.h"
#include "frc971/control_loops/control_loop.h"
#include "frc971/vision/ceres/pose_graph_3d_error_term.h"
#include "frc971/vision/geometry.h"

DEFINE_uint64(max_num_iterations, 100,
              "Maximum number of iterations for the ceres solver");
DEFINE_double(distortion_noise_scalar, 1.0,
              "Scale the target pose distortion factor by this when computing "
              "the noise.");
DEFINE_uint64(
    frozen_target_id, 1,
    "Freeze the pose of this target so the map can have one fixed point.");

namespace frc971::vision {
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

flatbuffers::Offset<TargetPoseFbs> PoseUtils::TargetPoseToFbs(
    const TargetMapper::TargetPose &target_pose,
    flatbuffers::FlatBufferBuilder *fbb) {
  const auto position_offset =
      CreatePosition(*fbb, target_pose.pose.p(0), target_pose.pose.p(1),
                     target_pose.pose.p(2));
  const auto orientation_offset =
      CreateQuaternion(*fbb, target_pose.pose.q.w(), target_pose.pose.q.x(),
                       target_pose.pose.q.y(), target_pose.pose.q.z());
  return CreateTargetPoseFbs(*fbb, target_pose.id, position_offset,
                             orientation_offset);
}

TargetMapper::TargetPose PoseUtils::TargetPoseFromFbs(
    const TargetPoseFbs &target_pose_fbs) {
  return {.id = static_cast<TargetMapper::TargetId>(target_pose_fbs.id()),
          .pose = ceres::examples::Pose3d{
              Eigen::Vector3d(target_pose_fbs.position()->x(),
                              target_pose_fbs.position()->y(),
                              target_pose_fbs.position()->z()),
              Eigen::Quaterniond(target_pose_fbs.orientation()->w(),
                                 target_pose_fbs.orientation()->x(),
                                 target_pose_fbs.orientation()->y(),
                                 target_pose_fbs.orientation()->z())}};
}

ceres::examples::VectorOfConstraints DataAdapter::MatchTargetDetections(
    const std::vector<DataAdapter::TimestampedDetection>
        &timestamped_target_detections,
    aos::distributed_clock::duration max_dt) {
  CHECK_GE(timestamped_target_detections.size(), 2ul)
      << "Must have at least 2 detections";

  // Match consecutive detections
  ceres::examples::VectorOfConstraints target_constraints;
  for (auto detection = timestamped_target_detections.begin() + 1;
       detection < timestamped_target_detections.end(); detection++) {
    auto last_detection = detection - 1;

    // Skip two consecutive detections of the same target, because the solver
    // doesn't allow this
    if (detection->id == last_detection->id) {
      continue;
    }

    // Don't take into account constraints too far apart in time, because the
    // recording device could have moved too much
    if ((detection->time - last_detection->time) > max_dt) {
      continue;
    }

    auto confidence = ComputeConfidence(*last_detection, *detection);
    target_constraints.emplace_back(
        ComputeTargetConstraint(*last_detection, *detection, confidence));
  }

  return target_constraints;
}

TargetMapper::ConfidenceMatrix DataAdapter::ComputeConfidence(
    const TimestampedDetection &detection_start,
    const TimestampedDetection &detection_end) {
  constexpr size_t kX = 0;
  constexpr size_t kY = 1;
  constexpr size_t kZ = 2;
  constexpr size_t kOrientation1 = 3;
  constexpr size_t kOrientation2 = 4;
  constexpr size_t kOrientation3 = 5;

  // Uncertainty matrix between start and end
  TargetMapper::ConfidenceMatrix P = TargetMapper::ConfidenceMatrix::Zero();

  {
    // Noise for odometry-based robot position measurements
    TargetMapper::ConfidenceMatrix Q_odometry =
        TargetMapper::ConfidenceMatrix::Zero();
    Q_odometry(kX, kX) = std::pow(0.045, 2);
    Q_odometry(kY, kY) = std::pow(0.045, 2);
    Q_odometry(kZ, kZ) = std::pow(0.045, 2);
    Q_odometry(kOrientation1, kOrientation1) = std::pow(0.01, 2);
    Q_odometry(kOrientation2, kOrientation2) = std::pow(0.01, 2);
    Q_odometry(kOrientation3, kOrientation3) = std::pow(0.01, 2);

    // Add uncertainty for robot position measurements from start to end
    int iterations = (detection_end.time - detection_start.time) /
                     frc971::controls::kLoopFrequency;
    P += static_cast<double>(iterations) * Q_odometry;
  }

  {
    // Noise for vision-based target localizations. Multiplying this matrix by
    // the distance from camera to target squared results in the uncertainty
    // in that measurement
    TargetMapper::ConfidenceMatrix Q_vision =
        TargetMapper::ConfidenceMatrix::Zero();
    Q_vision(kX, kX) = std::pow(0.045, 2);
    Q_vision(kY, kY) = std::pow(0.045, 2);
    Q_vision(kZ, kZ) = std::pow(0.045, 2);
    Q_vision(kOrientation1, kOrientation1) = std::pow(0.02, 2);
    Q_vision(kOrientation2, kOrientation2) = std::pow(0.02, 2);
    Q_vision(kOrientation3, kOrientation3) = std::pow(0.02, 2);

    // Add uncertainty for the 2 vision measurements (1 at start and 1 at end)
    P += Q_vision * std::pow(detection_start.distance_from_camera, 2) *
         (1.0 +
          FLAGS_distortion_noise_scalar * detection_start.distortion_factor);
    P +=
        Q_vision * std::pow(detection_end.distance_from_camera, 2) *
        (1.0 + FLAGS_distortion_noise_scalar * detection_end.distortion_factor);
  }

  return P.inverse();
}

ceres::examples::Constraint3d DataAdapter::ComputeTargetConstraint(
    const TimestampedDetection &target_detection_start,
    const TimestampedDetection &target_detection_end,
    const TargetMapper::ConfidenceMatrix &confidence) {
  // Compute the relative pose (constraint) between the two targets
  Eigen::Affine3d H_targetstart_targetend =
      target_detection_start.H_robot_target.inverse() *
      target_detection_end.H_robot_target;
  ceres::examples::Pose3d target_constraint =
      PoseUtils::Affine3dToPose3d(H_targetstart_targetend);

  const auto constraint_3d =
      ceres::examples::Constraint3d{target_detection_start.id,
                                    target_detection_end.id,
                                    {target_constraint.p, target_constraint.q},
                                    confidence};

  VLOG(2) << "Computed constraint: " << constraint_3d;
  return constraint_3d;
}

TargetMapper::TargetMapper(
    std::string_view target_poses_path,
    const ceres::examples::VectorOfConstraints &target_constraints)
    : target_constraints_(target_constraints) {
  aos::FlatbufferDetachedBuffer<TargetMap> target_map =
      aos::JsonFileToFlatbuffer<TargetMap>(target_poses_path);
  for (const auto *target_pose_fbs : *target_map.message().target_poses()) {
    target_poses_[target_pose_fbs->id()] =
        PoseUtils::TargetPoseFromFbs(*target_pose_fbs).pose;
  }
}

TargetMapper::TargetMapper(
    const ceres::examples::MapOfPoses &target_poses,
    const ceres::examples::VectorOfConstraints &target_constraints)
    : target_poses_(target_poses), target_constraints_(target_constraints) {}

std::optional<TargetMapper::TargetPose> TargetMapper::GetTargetPoseById(
    std::vector<TargetMapper::TargetPose> target_poses, TargetId target_id) {
  for (auto target_pose : target_poses) {
    if (target_pose.id == target_id) {
      return target_pose;
    }
  }

  return std::nullopt;
}

std::optional<TargetMapper::TargetPose> TargetMapper::GetTargetPoseById(
    TargetId target_id) {
  if (target_poses_.count(target_id) > 0) {
    return TargetMapper::TargetPose{target_id, target_poses_[target_id]};
  }

  return std::nullopt;
}

// Taken from ceres/examples/slam/pose_graph_3d/pose_graph_3d.cc
// Constructs the nonlinear least squares optimization problem from the pose
// graph constraints.
void TargetMapper::BuildOptimizationProblem(
    const ceres::examples::VectorOfConstraints &constraints,
    ceres::examples::MapOfPoses *poses, ceres::Problem *problem) {
  CHECK(poses != nullptr);
  CHECK(problem != nullptr);
  if (constraints.empty()) {
    LOG(INFO) << "No constraints, no problem to optimize.";
    return;
  }

  ceres::LossFunction *loss_function = new ceres::HuberLoss(2.0);
  ceres::LocalParameterization *quaternion_local_parameterization =
      new ceres::EigenQuaternionParameterization;

  for (ceres::examples::VectorOfConstraints::const_iterator constraints_iter =
           constraints.begin();
       constraints_iter != constraints.end(); ++constraints_iter) {
    const ceres::examples::Constraint3d &constraint = *constraints_iter;

    ceres::examples::MapOfPoses::iterator pose_begin_iter =
        poses->find(constraint.id_begin);
    CHECK(pose_begin_iter != poses->end())
        << "Pose with ID: " << constraint.id_begin << " not found.";
    ceres::examples::MapOfPoses::iterator pose_end_iter =
        poses->find(constraint.id_end);
    CHECK(pose_end_iter != poses->end())
        << "Pose with ID: " << constraint.id_end << " not found.";

    const Eigen::Matrix<double, 6, 6> sqrt_information =
        constraint.information.llt().matrixL();
    // Ceres will take ownership of the pointer.
    ceres::CostFunction *cost_function =
        ceres::examples::PoseGraph3dErrorTerm::Create(constraint.t_be,
                                                      sqrt_information);

    problem->AddResidualBlock(cost_function, loss_function,
                              pose_begin_iter->second.p.data(),
                              pose_begin_iter->second.q.coeffs().data(),
                              pose_end_iter->second.p.data(),
                              pose_end_iter->second.q.coeffs().data());

    problem->SetParameterization(pose_begin_iter->second.q.coeffs().data(),
                                 quaternion_local_parameterization);
    problem->SetParameterization(pose_end_iter->second.q.coeffs().data(),
                                 quaternion_local_parameterization);
  }

  // The pose graph optimization problem has six DOFs that are not fully
  // constrained. This is typically referred to as gauge freedom. You can
  // apply a rigid body transformation to all the nodes and the optimization
  // problem will still have the exact same cost. The Levenberg-Marquardt
  // algorithm has internal damping which mitigates this issue, but it is
  // better to properly constrain the gauge freedom. This can be done by
  // setting one of the poses as constant so the optimizer cannot change it.
  CHECK_NE(poses->count(FLAGS_frozen_target_id), 0ul)
      << "Got no poses for frozen target id " << FLAGS_frozen_target_id;
  ceres::examples::MapOfPoses::iterator pose_start_iter =
      poses->find(FLAGS_frozen_target_id);
  CHECK(pose_start_iter != poses->end()) << "There are no poses.";
  problem->SetParameterBlockConstant(pose_start_iter->second.p.data());
  problem->SetParameterBlockConstant(pose_start_iter->second.q.coeffs().data());
}

// Taken from ceres/examples/slam/pose_graph_3d/pose_graph_3d.cc
bool TargetMapper::SolveOptimizationProblem(ceres::Problem *problem) {
  CHECK_NOTNULL(problem);

  ceres::Solver::Options options;
  options.max_num_iterations = FLAGS_max_num_iterations;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  options.minimizer_progress_to_stdout = true;

  ceres::Solver::Summary summary;
  ceres::Solve(options, problem, &summary);

  LOG(INFO) << summary.FullReport() << '\n';

  return summary.IsSolutionUsable();
}

void TargetMapper::Solve(std::string_view field_name,
                         std::optional<std::string_view> output_dir) {
  ceres::Problem problem;
  BuildOptimizationProblem(target_constraints_, &target_poses_, &problem);

  CHECK(SolveOptimizationProblem(&problem))
      << "The solve was not successful, exiting.";

  auto map_json = MapToJson(field_name);
  VLOG(1) << "Solved target poses: " << map_json;

  if (output_dir.has_value()) {
    std::string output_path =
        absl::StrCat(output_dir.value(), "/", field_name, ".json");
    LOG(INFO) << "Writing map to file: " << output_path;
    aos::util::WriteStringToFileOrDie(output_path, map_json);
  }
}

std::string TargetMapper::MapToJson(std::string_view field_name) const {
  flatbuffers::FlatBufferBuilder fbb;

  // Convert poses to flatbuffers
  std::vector<flatbuffers::Offset<TargetPoseFbs>> target_poses_fbs;
  for (const auto &[id, pose] : target_poses_) {
    target_poses_fbs.emplace_back(
        PoseUtils::TargetPoseToFbs(TargetPose{.id = id, .pose = pose}, &fbb));
  }

  const auto field_name_offset = fbb.CreateString(field_name);
  flatbuffers::Offset<TargetMap> target_map_offset = CreateTargetMap(
      fbb, fbb.CreateVector(target_poses_fbs), field_name_offset);

  return aos::FlatbufferToJson(
      flatbuffers::GetMutableTemporaryPointer(fbb, target_map_offset),
      {.multi_line = true});
}

std::ostream &operator<<(std::ostream &os, ceres::examples::Pose3d pose) {
  auto rpy = PoseUtils::QuaternionToEulerAngles(pose.q);
  os << absl::StrFormat(
      "{x: %.3f, y: %.3f, z: %.3f, roll: %.3f, pitch: "
      "%.3f, yaw: %.3f}",
      pose.p(0), pose.p(1), pose.p(2), rpy(0), rpy(1), rpy(2));
  return os;
}

std::ostream &operator<<(std::ostream &os,
                         ceres::examples::Constraint3d constraint) {
  os << absl::StrFormat("{id_begin: %d, id_end: %d, pose: ",
                        constraint.id_begin, constraint.id_end)
     << constraint.t_be << "}";
  return os;
}

}  // namespace frc971::vision

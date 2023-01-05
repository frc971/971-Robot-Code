#include "frc971/vision/target_mapper.h"

#include "frc971/control_loops/control_loop.h"
#include "frc971/vision/ceres/angle_local_parameterization.h"
#include "frc971/vision/ceres/normalize_angle.h"
#include "frc971/vision/ceres/pose_graph_2d_error_term.h"
#include "frc971/vision/geometry.h"

DEFINE_uint64(max_num_iterations, 100,
              "Maximum number of iterations for the ceres solver");

namespace frc971::vision {

Eigen::Affine3d PoseUtils::Pose2dToAffine3d(ceres::examples::Pose2d pose2d) {
  Eigen::Affine3d H_world_pose =
      Eigen::Translation3d(pose2d.x, pose2d.y, 0.0) *
      Eigen::AngleAxisd(pose2d.yaw_radians, Eigen::Vector3d::UnitZ());
  return H_world_pose;
}

ceres::examples::Pose2d PoseUtils::Affine3dToPose2d(Eigen::Affine3d H) {
  Eigen::Vector3d T = H.translation();
  double heading = std::atan2(H.rotation()(1, 0), H.rotation()(0, 0));
  return ceres::examples::Pose2d{T[0], T[1],
                                 ceres::examples::NormalizeAngle(heading)};
}

ceres::examples::Pose2d PoseUtils::ComputeRelativePose(
    ceres::examples::Pose2d pose_1, ceres::examples::Pose2d pose_2) {
  Eigen::Affine3d H_world_1 = Pose2dToAffine3d(pose_1);
  Eigen::Affine3d H_world_2 = Pose2dToAffine3d(pose_2);

  // Get the location of 2 in the 1 frame
  Eigen::Affine3d H_1_2 = H_world_1.inverse() * H_world_2;
  return Affine3dToPose2d(H_1_2);
}

ceres::examples::Pose2d PoseUtils::ComputeOffsetPose(
    ceres::examples::Pose2d pose_1, ceres::examples::Pose2d pose_2_relative) {
  auto H_world_1 = Pose2dToAffine3d(pose_1);
  auto H_1_2 = Pose2dToAffine3d(pose_2_relative);
  auto H_world_2 = H_world_1 * H_1_2;

  return Affine3dToPose2d(H_world_2);
}

namespace {
double ExponentiatedSinTerm(double theta) {
  return (theta == 0.0 ? 1.0 : std::sin(theta) / theta);
}

double ExponentiatedCosTerm(double theta) {
  return (theta == 0.0 ? 0.0 : (1 - std::cos(theta)) / theta);
}
}  // namespace

ceres::examples::Pose2d DataAdapter::InterpolatePose(
    const TimestampedPose &pose_start, const TimestampedPose &pose_end,
    aos::distributed_clock::time_point time) {
  auto delta_pose =
      PoseUtils::ComputeRelativePose(pose_start.pose, pose_end.pose);
  // Time from start of period, on the scale 0-1 where 1 is the end.
  double interpolation_scalar =
      static_cast<double>((time - pose_start.time).count()) /
      static_cast<double>((pose_end.time - pose_start.time).count());

  double theta = delta_pose.yaw_radians;
  // Take the log of the transformation matrix:
  // https://mathoverflow.net/questions/118533/how-to-compute-se2-group-exponential-and-logarithm
  StdFormLine dx_line = {.a = ExponentiatedSinTerm(theta),
                         .b = -ExponentiatedCosTerm(theta),
                         .c = delta_pose.x};
  StdFormLine dy_line = {.a = ExponentiatedCosTerm(theta),
                         .b = ExponentiatedSinTerm(theta),
                         .c = delta_pose.y};

  std::optional<cv::Point2d> solution = dx_line.Intersection(dy_line);
  CHECK(solution.has_value());

  // Re-exponentiate with the new values scaled by the interpolation scalar to
  // get an interpolated tranformation matrix
  double a = solution->x * interpolation_scalar;
  double b = solution->y * interpolation_scalar;
  double alpha = theta * interpolation_scalar;

  ceres::examples::Pose2d interpolated_pose = {
      .x = a * ExponentiatedSinTerm(theta) - b * ExponentiatedCosTerm(theta),
      .y = a * ExponentiatedCosTerm(theta) + b * ExponentiatedSinTerm(theta),
      .yaw_radians = alpha};

  return PoseUtils::ComputeOffsetPose(pose_start.pose, interpolated_pose);
}  // namespace frc971::vision

std::pair<std::vector<ceres::examples::Constraint2d>,
          std::vector<ceres::examples::Pose2d>>
DataAdapter::MatchTargetDetections(
    const std::vector<TimestampedPose> &timestamped_robot_poses,
    const std::vector<TimestampedDetection> &timestamped_target_detections) {
  // Interpolate robot poses
  std::map<aos::distributed_clock::time_point, ceres::examples::Pose2d>
      interpolated_poses;

  CHECK_GT(timestamped_robot_poses.size(), 1ul)
      << "Need more than 1 robot pose";
  auto robot_pose_it = timestamped_robot_poses.begin();
  for (const auto &timestamped_detection : timestamped_target_detections) {
    aos::distributed_clock::time_point target_time = timestamped_detection.time;

    // Skip this target detection if we have no robot poses before it
    if (robot_pose_it->time > target_time) {
      continue;
    }

    // Find the robot point right before this localization
    while (robot_pose_it->time > target_time ||
           (robot_pose_it + 1)->time <= target_time) {
      robot_pose_it++;
      CHECK(robot_pose_it < timestamped_robot_poses.end() - 1)
          << "Need a robot pose before and after every target detection";
    }

    auto start = robot_pose_it;
    auto end = robot_pose_it + 1;
    interpolated_poses.emplace(target_time,
                               InterpolatePose(*start, *end, target_time));
  }

  // In the case that all target detections were before the first robot
  // detection, we would have no interpolated poses at this point
  CHECK_GT(interpolated_poses.size(), 0ul)
      << "Need a robot pose before and after every target detection";

  // Match consecutive detections
  std::vector<ceres::examples::Constraint2d> target_constraints;
  std::vector<ceres::examples::Pose2d> robot_delta_poses;

  auto last_detection = timestamped_target_detections[0];
  auto last_robot_pose =
      interpolated_poses[timestamped_target_detections[0].time];

  for (auto it = timestamped_target_detections.begin() + 1;
       it < timestamped_target_detections.end(); it++) {
    // Skip two consecutive detections of the same target, because the solver
    // doesn't allow this
    if (it->id == last_detection.id) {
      continue;
    }

    auto robot_pose = interpolated_poses[it->time];
    auto robot_delta_pose =
        PoseUtils::ComputeRelativePose(last_robot_pose, robot_pose);
    auto confidence = ComputeConfidence(last_detection.time, it->time,
                                        last_detection.distance_from_camera,
                                        it->distance_from_camera);

    target_constraints.emplace_back(ComputeTargetConstraint(
        last_detection, PoseUtils::Pose2dToAffine3d(robot_delta_pose), *it,
        confidence));
    robot_delta_poses.emplace_back(robot_delta_pose);

    last_detection = *it;
    last_robot_pose = robot_pose;
  }

  return {target_constraints, robot_delta_poses};
}

std::vector<ceres::examples::Constraint2d> DataAdapter::MatchTargetDetections(
    const std::vector<DataAdapter::TimestampedDetection>
        &timestamped_target_detections,
    aos::distributed_clock::duration max_dt) {
  CHECK_GE(timestamped_target_detections.size(), 2ul)
      << "Must have at least 2 detections";

  // Match consecutive detections
  std::vector<ceres::examples::Constraint2d> target_constraints;
  for (auto it = timestamped_target_detections.begin() + 1;
       it < timestamped_target_detections.end(); it++) {
    auto last_detection = *(it - 1);

    // Skip two consecutive detections of the same target, because the solver
    // doesn't allow this
    if (it->id == last_detection.id) {
      continue;
    }

    // Don't take into account constraints too far apart in time, because the
    // recording device could have moved too much
    if ((it->time - last_detection.time) > max_dt) {
      continue;
    }

    auto confidence = ComputeConfidence(last_detection.time, it->time,
                                        last_detection.distance_from_camera,
                                        it->distance_from_camera);
    target_constraints.emplace_back(
        ComputeTargetConstraint(last_detection, *it, confidence));
  }

  return target_constraints;
}

Eigen::Matrix3d DataAdapter::ComputeConfidence(
    aos::distributed_clock::time_point start,
    aos::distributed_clock::time_point end, double distance_from_camera_start,
    double distance_from_camera_end) {
  constexpr size_t kX = 0;
  constexpr size_t kY = 1;
  constexpr size_t kTheta = 2;

  // Uncertainty matrix between start and end
  Eigen::Matrix3d P = Eigen::Matrix3d::Zero();

  {
    // Noise for odometry-based robot position measurements
    Eigen::Matrix3d Q_odometry = Eigen::Matrix3d::Zero();
    Q_odometry(kX, kX) = std::pow(0.045, 2);
    Q_odometry(kY, kY) = std::pow(0.045, 2);
    Q_odometry(kTheta, kTheta) = std::pow(0.01, 2);

    // Add uncertainty for robot position measurements from start to end
    int iterations = (end - start) / frc971::controls::kLoopFrequency;
    P += static_cast<double>(iterations) * Q_odometry;
  }

  {
    // Noise for vision-based target localizations. Multiplying this matrix by
    // the distance from camera to target squared results in the uncertainty in
    // that measurement
    Eigen::Matrix3d Q_vision = Eigen::Matrix3d::Zero();
    Q_vision(kX, kX) = std::pow(0.045, 2);
    Q_vision(kY, kY) = std::pow(0.045, 2);
    Q_vision(kTheta, kTheta) = std::pow(0.02, 2);

    // Add uncertainty for the 2 vision measurements (1 at start and 1 at end)
    P += Q_vision * std::pow(distance_from_camera_start, 2);
    P += Q_vision * std::pow(distance_from_camera_end, 2);
  }

  return P.inverse();
}

ceres::examples::Constraint2d DataAdapter::ComputeTargetConstraint(
    const TimestampedDetection &target_detection_start,
    const Eigen::Affine3d &H_robotstart_robotend,
    const TimestampedDetection &target_detection_end,
    const Eigen::Matrix3d &confidence) {
  // Compute the relative pose (constraint) between the two targets
  Eigen::Affine3d H_targetstart_targetend =
      target_detection_start.H_robot_target.inverse() * H_robotstart_robotend *
      target_detection_end.H_robot_target;
  ceres::examples::Pose2d target_constraint =
      PoseUtils::Affine3dToPose2d(H_targetstart_targetend);

  return ceres::examples::Constraint2d{
      target_detection_start.id,     target_detection_end.id,
      target_constraint.x,           target_constraint.y,
      target_constraint.yaw_radians, confidence};
}

ceres::examples::Constraint2d DataAdapter::ComputeTargetConstraint(
    const TimestampedDetection &target_detection_start,
    const TimestampedDetection &target_detection_end,
    const Eigen::Matrix3d &confidence) {
  return ComputeTargetConstraint(target_detection_start,
                                 Eigen::Affine3d(Eigen::Matrix4d::Identity()),
                                 target_detection_end, confidence);
}

TargetMapper::TargetMapper(
    std::string_view target_poses_path,
    std::vector<ceres::examples::Constraint2d> target_constraints)
    : target_constraints_(target_constraints) {
  aos::FlatbufferDetachedBuffer<TargetMap> target_map =
      aos::JsonFileToFlatbuffer<TargetMap>(target_poses_path);
  for (const auto *target_pose_fbs : *target_map.message().target_poses()) {
    target_poses_[target_pose_fbs->id()] = ceres::examples::Pose2d{
        target_pose_fbs->x(), target_pose_fbs->y(), target_pose_fbs->yaw()};
  }
}

TargetMapper::TargetMapper(
    std::map<TargetId, ceres::examples::Pose2d> target_poses,
    std::vector<ceres::examples::Constraint2d> target_constraints)
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

// Taken from ceres/examples/slam/pose_graph_2d/pose_graph_2d.cc
void TargetMapper::BuildOptimizationProblem(
    std::map<int, ceres::examples::Pose2d> *poses,
    const std::vector<ceres::examples::Constraint2d> &constraints,
    ceres::Problem *problem) {
  CHECK_NOTNULL(poses);
  CHECK_NOTNULL(problem);
  if (constraints.empty()) {
    LOG(WARNING) << "No constraints, no problem to optimize.";
    return;
  }

  ceres::LossFunction *loss_function = new ceres::HuberLoss(2.0);
  ceres::LocalParameterization *angle_local_parameterization =
      ceres::examples::AngleLocalParameterization::Create();

  for (std::vector<ceres::examples::Constraint2d>::const_iterator
           constraints_iter = constraints.begin();
       constraints_iter != constraints.end(); ++constraints_iter) {
    const ceres::examples::Constraint2d &constraint = *constraints_iter;

    std::map<int, ceres::examples::Pose2d>::iterator pose_begin_iter =
        poses->find(constraint.id_begin);
    CHECK(pose_begin_iter != poses->end())
        << "Pose with ID: " << constraint.id_begin << " not found.";
    std::map<int, ceres::examples::Pose2d>::iterator pose_end_iter =
        poses->find(constraint.id_end);
    CHECK(pose_end_iter != poses->end())
        << "Pose with ID: " << constraint.id_end << " not found.";

    const Eigen::Matrix3d sqrt_information =
        constraint.information.llt().matrixL();
    // Ceres will take ownership of the pointer.
    ceres::CostFunction *cost_function =
        ceres::examples::PoseGraph2dErrorTerm::Create(
            constraint.x, constraint.y, constraint.yaw_radians,
            sqrt_information);
    problem->AddResidualBlock(
        cost_function, loss_function, &pose_begin_iter->second.x,
        &pose_begin_iter->second.y, &pose_begin_iter->second.yaw_radians,
        &pose_end_iter->second.x, &pose_end_iter->second.y,
        &pose_end_iter->second.yaw_radians);

    problem->SetParameterization(&pose_begin_iter->second.yaw_radians,
                                 angle_local_parameterization);
    problem->SetParameterization(&pose_end_iter->second.yaw_radians,
                                 angle_local_parameterization);
  }

  // The pose graph optimization problem has three DOFs that are not fully
  // constrained. This is typically referred to as gauge freedom. You can apply
  // a rigid body transformation to all the nodes and the optimization problem
  // will still have the exact same cost. The Levenberg-Marquardt algorithm has
  // internal damping which mitigates this issue, but it is better to properly
  // constrain the gauge freedom. This can be done by setting one of the poses
  // as constant so the optimizer cannot change it.
  std::map<int, ceres::examples::Pose2d>::iterator pose_start_iter =
      poses->begin();
  CHECK(pose_start_iter != poses->end()) << "There are no poses.";
  problem->SetParameterBlockConstant(&pose_start_iter->second.x);
  problem->SetParameterBlockConstant(&pose_start_iter->second.y);
  problem->SetParameterBlockConstant(&pose_start_iter->second.yaw_radians);
}

// Taken from ceres/examples/slam/pose_graph_2d/pose_graph_2d.cc
bool TargetMapper::SolveOptimizationProblem(ceres::Problem *problem) {
  CHECK_NOTNULL(problem);

  ceres::Solver::Options options;
  options.max_num_iterations = FLAGS_max_num_iterations;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;

  ceres::Solver::Summary summary;
  ceres::Solve(options, problem, &summary);

  LOG(INFO) << summary.FullReport() << '\n';

  return summary.IsSolutionUsable();
}

void TargetMapper::Solve(std::string_view field_name,
                         std::optional<std::string_view> output_dir) {
  ceres::Problem problem;
  BuildOptimizationProblem(&target_poses_, target_constraints_, &problem);

  CHECK(SolveOptimizationProblem(&problem))
      << "The solve was not successful, exiting.";

  // TODO(milind): add origin to first target offset to all poses

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
    TargetPoseFbs::Builder target_pose_builder(fbb);
    target_pose_builder.add_id(id);
    target_pose_builder.add_x(pose.x);
    target_pose_builder.add_y(pose.y);
    target_pose_builder.add_yaw(pose.yaw_radians);

    target_poses_fbs.emplace_back(target_pose_builder.Finish());
  }

  const auto field_name_offset = fbb.CreateString(field_name);
  flatbuffers::Offset<TargetMap> target_map_offset = CreateTargetMap(
      fbb, fbb.CreateVector(target_poses_fbs), field_name_offset);

  return aos::FlatbufferToJson(
      flatbuffers::GetMutableTemporaryPointer(fbb, target_map_offset),
      {.multi_line = true});
}

}  // namespace frc971::vision

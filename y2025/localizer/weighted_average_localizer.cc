#include "y2025/localizer/localizer.h"

ABSL_FLAG(double, vision_weight, 0.7,
          "How much to weigh the vision detection vs the velocity based pose "
          "estimation.");
ABSL_FLAG(double, distance_threshold, 3.0,
          "Distance in meters from the robot where we consider detections "
          "invalid due to the variance they have.");
ABSL_FLAG(bool, do_moving_average, false,
          "If true, use a moving average of previous samples.");
ABSL_FLAG(
    int, time_threshold, 100,
    "How long in ms to wait until ignoring a detection for being too old.");
ABSL_FLAG(double, displacement_threshold, 0.5,
          "How much displacement in meters are we willing to accept before "
          "rejecting a detected pose.");
ABSL_FLAG(
    double, moving_average_factor, 1,
    "Scales up the moving average accumulation. Increasing this number "
    "increases the weight of the most recent sample in the moving average.");

namespace y2025::localizer {
WeightedAverageLocalizer::WeightedAverageLocalizer(aos::EventLoop *event_loop)
    : Localizer(event_loop) {
  start_time_ = event_loop->context().monotonic_event_time;
}

void WeightedAverageLocalizer::HandleDetectedRobotPose(
    XYThetaPose pose, double distance_to_robot, uint64_t target_id,
    double distortion_factor, aos::monotonic_clock::time_point capture_time,
    aos::monotonic_clock::time_point now) {
  (void)now;
  (void)distortion_factor;

  if (detected_pose_.contains(target_id)) {
    detected_pose_.erase(target_id);
  }

  detected_pose_.emplace(target_id, WeightedAverageLocalizer::DetectionInfo{
                                        .distance_to_robot = distance_to_robot,
                                        .time_detected = capture_time,
                                        .pose = pose});
}

void WeightedAverageLocalizer::HandleEstimatedSwerveState(
    EstimatedSwerveState estimated_state,
    aos::monotonic_clock::time_point now) {
  (void)now;

  estimated_pose_(PoseIdx::kX) = estimated_state.x;
  estimated_pose_(PoseIdx::kY) = estimated_state.y;
  estimated_pose_(PoseIdx::kTheta) = estimated_state.theta;
}

void WeightedAverageLocalizer::HandleAutonomousInit(
    const frc971::control_loops::swerve::AutonomousInit &init_message) {
  estimated_pose_(PoseIdx::kX) = init_message.x();
  estimated_pose_(PoseIdx::kY) = init_message.y();
  estimated_pose_(PoseIdx::kTheta) = init_message.theta();

  detected_pose_.clear();
}

void WeightedAverageLocalizer::SendOutput(
    aos::Sender<LocalizerStateStatic>::StaticBuilder state_builder) {
  std::map<uint64_t, WeighingDetectionInfo> good_detections;

  double total_metric = 0.0;

  for (const auto &[tag_id, detection] : detected_pose_) {
    if ((event_loop_->context().monotonic_event_time -
         detection.time_detected) >=
        std::chrono::milliseconds(absl::GetFlag(FLAGS_time_threshold))) {
      continue;
    }

    double distance_to_robot = detection.distance_to_robot;
    if (distance_to_robot > absl::GetFlag(FLAGS_distance_threshold)) {
      VLOG(1) << "Rejecting because apriltag detection with distance "
              << distance_to_robot << " to the robot.";
      continue;
      // TODO(max): Add this to the rejection counter.
    }

    double weighing_metric =
        1.0 + (1.0 / (distance_to_robot * distance_to_robot));

    good_detections.emplace(tag_id,
                            WeightedAverageLocalizer::WeighingDetectionInfo{
                                .weighing_metric = weighing_metric,
                                .pose = detection.pose,
                            });

    total_metric += weighing_metric;
  }

  XYThetaPose average_detected_pose = XYThetaPose::Zero();
  Eigen::Matrix<double, 2, 1> average_detected_yaw =
      Eigen::Matrix<double, 2, 1>::Zero();

  for (const auto &[_, weighing_info] : good_detections) {
    average_detected_pose(PoseIdx::kX) +=
        weighing_info.pose(PoseIdx::kX) * weighing_info.weighing_metric;
    average_detected_pose(PoseIdx::kY) +=
        weighing_info.pose(PoseIdx::kY) * weighing_info.weighing_metric;

    // We're doing this so that we actually properly take the average of
    // theta. https://en.wikipedia.org/wiki/Circular_mean
    average_detected_yaw(0) += cos(weighing_info.pose(PoseIdx::kTheta)) *
                               weighing_info.weighing_metric;
    average_detected_yaw(1) += sin(weighing_info.pose(PoseIdx::kTheta)) *
                               weighing_info.weighing_metric;
  }

  average_detected_pose(PoseIdx::kX) =
      average_detected_pose(PoseIdx::kX) / total_metric;
  average_detected_pose(PoseIdx::kY) =
      average_detected_pose(PoseIdx::kY) / total_metric;
  average_detected_pose(PoseIdx::kTheta) =
      atan2(average_detected_yaw(1), average_detected_yaw(0));

  double vision_weight = absl::GetFlag(FLAGS_vision_weight);

  Eigen::Matrix<double, 2, 1> yaw_vector = Eigen::Matrix<double, 2, 1>::Zero();

  yaw_vector(0) = cos(average_detected_pose(PoseIdx::kTheta)) * vision_weight +
                  cos(estimated_pose_(PoseIdx::kTheta)) * (1 - vision_weight);

  yaw_vector(1) = sin(average_detected_pose(PoseIdx::kTheta)) * vision_weight +
                  sin(estimated_pose_(PoseIdx::kTheta)) * (1 - vision_weight);

  XYThetaPose final_robot_pose = XYThetaPose::Zero();

  if (std::isnan(average_detected_pose(PoseIdx::kX))) {
    final_robot_pose = estimated_pose_;
  } else {
    final_robot_pose << average_detected_pose(PoseIdx::kX) * vision_weight +
                            estimated_pose_(PoseIdx::kX) * (1 - vision_weight),
        average_detected_pose(PoseIdx::kY) * vision_weight +
            estimated_pose_(PoseIdx::kY) * (1 - vision_weight),
        atan2(yaw_vector(1), yaw_vector(0));
  }

  if ((estimated_pose_ - final_robot_pose).norm() >
          absl::GetFlag(FLAGS_displacement_threshold) &&
      !first_it_ &&
      (event_loop_->context().monotonic_event_time - start_time_) >=
          std::chrono::seconds(4)) {
    VLOG(1) << "Rejecting image for shoving us "
            << absl::GetFlag(FLAGS_displacement_threshold)
            << "m  away from our pose in one "
               "iteration.";
    return;
  }

  estimated_pose_ = final_robot_pose;

  if (previous_samples_.size() >=
      WeightedAverageLocalizer::kMaxPreviousSamples) {
    previous_samples_.pop_back();
  }

  previous_samples_.insert(previous_samples_.begin(), estimated_pose_);

  if (absl::GetFlag(FLAGS_do_moving_average)) {
    // Vector with x, y, sin(theta), cos(theta) for the moving average.
    Eigen::Matrix<double, 4, 1> accumulated_pose =
        Eigen::Matrix<double, 4, 1>::Zero();

    for (size_t i = 0; i < previous_samples_.size(); i++) {
      if (i == 0) {
        accumulated_pose(0) += absl::GetFlag(FLAGS_moving_average_factor) *
                               previous_samples_.at(i)(PoseIdx::kX);
        accumulated_pose(1) += absl::GetFlag(FLAGS_moving_average_factor) *
                               previous_samples_.at(i)(PoseIdx::kY);
        accumulated_pose(2) += absl::GetFlag(FLAGS_moving_average_factor) *
                               sin(previous_samples_.at(i)(PoseIdx::kTheta));
        accumulated_pose(3) += absl::GetFlag(FLAGS_moving_average_factor) *
                               cos(previous_samples_.at(i)(PoseIdx::kTheta));
      } else {
        accumulated_pose(0) += previous_samples_.at(i)(PoseIdx::kX);
        accumulated_pose(1) += previous_samples_.at(i)(PoseIdx::kY);
        accumulated_pose(2) += sin(previous_samples_.at(i)(PoseIdx::kTheta));
        accumulated_pose(3) += cos(previous_samples_.at(i)(PoseIdx::kTheta));
      }
    }
    size_t num_captured_samples_ = previous_samples_.size() - 1 +
                                   absl::GetFlag(FLAGS_moving_average_factor);

    estimated_pose_ << accumulated_pose(0) / num_captured_samples_,
        accumulated_pose(1) / num_captured_samples_,
        atan2(accumulated_pose(2), accumulated_pose(3));
  }

  if (first_it_ && good_detections.size() > 0 &&
      (event_loop_->context().monotonic_event_time - start_time_) >=
          std::chrono::seconds(2)) {
    state_builder->set_x(average_detected_pose(PoseIdx::kX));
    state_builder->set_y(average_detected_pose(PoseIdx::kY));
    state_builder->set_theta(average_detected_pose(PoseIdx::kTheta));

    state_builder.CheckOk(state_builder.Send());
    first_it_ = false;

    estimated_pose_ = average_detected_pose;
    return;
  }

  state_builder->set_x(final_robot_pose(PoseIdx::kX));
  state_builder->set_y(final_robot_pose(PoseIdx::kY));
  state_builder->set_theta(final_robot_pose(PoseIdx::kTheta));

  state_builder.CheckOk(state_builder.Send());
}

}  // namespace y2025::localizer

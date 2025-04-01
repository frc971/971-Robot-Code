#include <map>

#include "absl/flags/flag.h"

#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "frc971/constants/constants_sender_lib.h"
#include "frc971/control_loops/pose.h"
#include "frc971/control_loops/swerve/position_goal_static.h"
#include "frc971/control_loops/swerve/swerve_localizer_state_generated.h"
#include "frc971/vision/target_map_utils.h"
#include "y2025/control_loops/superstructure/superstructure_goal_generated.h"
#include "y2025/control_loops/swerve/parameters.h"
#include "y2025/localizer/localizer.h"

ABSL_FLAG(std::string, config, "aos_config.json",
          "Path to the config file to use.");

// Distance to offset the robots position past the auto align goal tangent to
// the tag.
constexpr double kTangentOffset = 6 * 0.0254;
// Distance to offset the robots position normal to the tag (left/right).
constexpr double kNormalOffset = 3 * 0.0254;
constexpr double kArmOffset = 1.603578 * 0.0254;
// Distance between the two poles in the reef.
constexpr double kPoleDistance = 13 * 0.0254;

constexpr double kGoalReefAvoidanceThreshold = 0.05;
constexpr double kGoalReefAvoidanceOffset = 0.2;

using frc971::control_loops::Pose;
using y2025::Constants;
using y2025::localizer::Localizer;

std::map<uint64_t, Localizer::Transform> GetTargetLocations(
    const Constants &constants) {
  CHECK(constants.has_common());
  CHECK(constants.common()->has_target_map());
  CHECK(constants.common()->target_map()->has_target_poses());
  std::map<uint64_t, Localizer::Transform> transforms;
  for (const frc971::vision::TargetPoseFieldFbs *target :
       *constants.common()->target_map()->target_poses()) {
    CHECK(target->has_id());
    CHECK(target->has_position());
    CHECK(target->has_orientation());
    CHECK_EQ(0u, transforms.count(target->id()));
    transforms[target->id()] = PoseToTransform(target);
  }
  return transforms;
}

class AutoAlignUpdater {
 public:
  AutoAlignUpdater(aos::EventLoop *event_loop)
      : event_loop_(event_loop),
        constants_fetcher_(event_loop_),
        target_poses_(GetTargetLocations(constants_fetcher_.constants())),
        position_goal_sender_(
            event_loop_
                ->MakeSender<frc971::control_loops::swerve::PositionGoalStatic>(
                    "/autonomous_auto_align")),
        goal_fetcher_(
            event_loop_
                ->MakeFetcher<y2025::control_loops::superstructure::Goal>(
                    "/roborio/superstructure")) {
    for (int id :
         *constants_fetcher_.constants().common()->reef_apriltag_ids()) {
      const Pose target_pose = Pose(target_poses_.at(id));
      Pose auto_align_pose = Pose(target_pose);
      (*auto_align_pose.mutable_pos())(0) +=
          ((y2025::control_loops::kSideLength / 2) + kTangentOffset) *
          sin(auto_align_pose.abs_theta());
      (*auto_align_pose.mutable_pos())(1) -=
          ((y2025::control_loops::kSideLength / 2) + kTangentOffset) *
          cos(auto_align_pose.abs_theta());

      auto_align_pose.set_theta(auto_align_pose.abs_theta() - M_PI / 2.0);

      reef_locations_.emplace(id, auto_align_pose);
    }

    event_loop_->MakeWatcher("/localizer", [this](const frc971::control_loops::
                                                      swerve::LocalizerState
                                                          &state) {
      int closest_id =
          constants_fetcher_.constants().common()->reef_apriltag_ids()->Get(0);

      Eigen::Matrix<double, 3, 1> robot_xyz =
          Eigen::Matrix<double, 3, 1>::Zero();

      robot_xyz << state.x(), state.y(), 0.0;

      Pose robot_pose(robot_xyz, state.theta());

      for (const auto &[id, pose] : reef_locations_) {
        if (std::abs(pose.Rebase(&robot_pose).xy_norm()) <
            std::abs(
                reef_locations_.at(closest_id).Rebase(&robot_pose).xy_norm())) {
          closest_id = id;
        }
      }

      VLOG(1) << "Closest auto align apriltag id: " << closest_id;

      auto builder = position_goal_sender_.MakeStaticBuilder();

      Pose final_pose = reef_locations_.at(closest_id);
      double offset = 0.0;
      goal_fetcher_.Fetch();

      auto final_transform = final_pose.AsTransformationMatrix();
      auto robot_transform = robot_pose.AsTransformationMatrix();

      bool flip = final_transform.block<3, 1>(0, 0).dot(
                      robot_transform.block<3, 1>(0, 0)) < 0.0;

      if (goal_fetcher_.get() != nullptr) {
        switch (goal_fetcher_->auto_align_direction()) {
          case y2025::control_loops::superstructure::AutoAlignDirection::LEFT:
            offset = kNormalOffset - (flip ? 0 : kPoleDistance);
            break;
          case y2025::control_loops::superstructure::AutoAlignDirection::RIGHT:
            offset = kNormalOffset + (flip ? kPoleDistance : 0);
            break;
          case y2025::control_loops::superstructure::AutoAlignDirection::CENTER:
            break;
        }

        if (flip) {
          offset -= 2 * kArmOffset;
        }
      }

      double goal_theta = final_pose.abs_theta();

      if (goal_fetcher_.get() &&
          goal_fetcher_->elevator_goal() ==
              y2025::control_loops::superstructure::ElevatorGoal::INTAKE_HP) {
        goal_theta += std::numbers::pi;
      }

      goal_theta =
          aos::math::NormalizeAngle(goal_theta + (flip ? std::numbers::pi : 0));

      double goal_x = final_pose.abs_pos()(0) +
                      offset * cos(final_pose.abs_theta() + M_PI / 2.0);
      double goal_y = final_pose.abs_pos()(1) +
                      offset * sin(final_pose.abs_theta() + M_PI / 2.0);

      // tangent distance to the tag vector
      // https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line#Line_defined_by_point_and_angle
      const double distance =
          std::abs((goal_y - state.y()) * std::cos(goal_theta) -
                   (goal_x - state.x()) * std::sin(goal_theta));
      if (distance > kGoalReefAvoidanceThreshold) {
        goal_x += kGoalReefAvoidanceOffset * std::cos(goal_theta);
        goal_y += kGoalReefAvoidanceOffset * std::sin(goal_theta);
      }

      builder->set_theta(goal_theta);
      if (goal_fetcher_.get() == nullptr || !goal_fetcher_->theta_lock()) {
        builder->set_x(goal_x);
        builder->set_y(goal_y);
      }

      builder.CheckOk(builder.Send());
    });
  }

 private:
  aos::EventLoop *event_loop_;

  frc971::constants::ConstantsFetcher<Constants> constants_fetcher_;
  const std::map<uint64_t, Localizer::Transform> target_poses_;
  aos::Sender<frc971::control_loops::swerve::PositionGoalStatic>
      position_goal_sender_;
  aos::Fetcher<y2025::control_loops::superstructure::Goal> goal_fetcher_;
  std::map<int, frc971::control_loops::Pose> reef_locations_;
};

int main(int argc, char *argv[]) {
  aos::InitGoogle(&argc, &argv);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(absl::GetFlag(FLAGS_config));

  frc971::constants::WaitForConstants<y2025::Constants>(&config.message());

  aos::ShmEventLoop event_loop(&config.message());
  AutoAlignUpdater auto_align_updater(&event_loop);

  event_loop.Run();

  return 0;
}

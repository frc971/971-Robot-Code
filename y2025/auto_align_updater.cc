#include <map>

#include "absl/flags/flag.h"

#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "frc971/constants/constants_sender_lib.h"
#include "frc971/control_loops/pose.h"
#include "frc971/control_loops/swerve/position_goal_static.h"
#include "frc971/control_loops/swerve/swerve_localizer_state_generated.h"
#include "frc971/vision/target_map_utils.h"
#include "y2025/localizer/localizer.h"

ABSL_FLAG(std::string, config, "aos_config.json",
          "Path to the config file to use.");

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
                    "/autonomous_auto_align")) {
    for (int id :
         *constants_fetcher_.constants().common()->reef_apriltag_ids()) {
      const Pose target_pose = Pose(target_poses_.at(id));
      Pose auto_align_pose = Pose(target_pose);
      (*auto_align_pose.mutable_pos())(0) -=
          0.5 * cos(auto_align_pose.abs_theta());
      (*auto_align_pose.mutable_pos())(1) -=
          0.5 * sin(auto_align_pose.abs_theta());

      reef_locations_.emplace(id, auto_align_pose);
    }

    event_loop_->MakeWatcher(
        "/localizer",
        [this](const frc971::control_loops::swerve::LocalizerState &state) {
          int closest_id =
              constants_fetcher_.constants().common()->reef_apriltag_ids()->Get(
                  0);

          Eigen::Matrix<double, 3, 1> robot_pos =
              Eigen::Matrix<double, 3, 1>::Zero();

          robot_pos << state.x(), state.y(), 0.0;

          Pose robot_pose(robot_pos, state.theta());

          for (const auto &[id, pose] : reef_locations_) {
            if (pose.Rebase(&robot_pose).xy_norm() <
                reef_locations_.at(closest_id).Rebase(&robot_pose).xy_norm()) {
              closest_id = id;
            }
          }
          auto builder = position_goal_sender_.MakeStaticBuilder();

          Pose final_pose = reef_locations_.at(closest_id);

          builder->set_x(final_pose.abs_pos()(0));
          builder->set_y(final_pose.abs_pos()(1));
          builder->set_theta(final_pose.abs_theta());

          builder.CheckOk(builder.Send());
        });
  }

 private:
  aos::EventLoop *event_loop_;

  frc971::constants::ConstantsFetcher<Constants> constants_fetcher_;
  const std::map<uint64_t, Localizer::Transform> target_poses_;
  aos::Sender<frc971::control_loops::swerve::PositionGoalStatic>
      position_goal_sender_;
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

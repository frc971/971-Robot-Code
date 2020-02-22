#include "y2020/control_loops/superstructure/turret/aiming.h"

#include "frc971/control_loops/pose.h"
#include "y2020/control_loops/drivetrain/drivetrain_base.h"

namespace y2020 {
namespace control_loops {
namespace superstructure {
namespace turret {

using frc971::control_loops::Pose;

namespace {
flatbuffers::DetachedBuffer MakePrefilledGoal() {
  flatbuffers::FlatBufferBuilder fbb;
  fbb.ForceDefaults(true);
  Aimer::Goal::Builder builder(fbb);
  builder.add_unsafe_goal(0);
  builder.add_goal_velocity(0);
  builder.add_ignore_profile(true);
  fbb.Finish(builder.Finish());
  return fbb.Release();
}
}  // namespace

Aimer::Aimer() : goal_(MakePrefilledGoal()) {}

void Aimer::Update(const Status *status) {
  // For now, just do enough to keep the turret pointed straight towards (0, 0).
  // Don't worry about properly handling shooting on the fly--just try to keep
  // the turret pointed straight towards one target.
  // This also doesn't do anything intelligent with wrapping--it just produces a
  // result in the range (-pi, pi] rather than taking advantage of the turret's
  // full range.
  Pose goal({0, 0, 0}, 0);
  const Pose robot_pose({status->x(), status->y(), 0}, status->theta());
  goal = goal.Rebase(&robot_pose);
  const double heading_to_goal = goal.heading();
  CHECK(status->has_localizer());
  // TODO(james): This code should probably just be in the localizer and have
  // xdot/ydot get populated in the status message directly... that way we don't
  // keep duplicating this math.
  // Also, this doesn't currently take into account the lateral velocity of the
  // robot. All of this would be helped by just doing this work in the Localizer
  // itself.
  const Eigen::Vector2d linear_angular =
      drivetrain::GetDrivetrainConfig().Tlr_to_la() *
      Eigen::Vector2d(status->localizer()->left_velocity(),
                      status->localizer()->right_velocity());
  // X and Y dot are negated because we are interested in the derivative of
  // (target_pos - robot_pos).
  const double xdot = -linear_angular(0) * std::cos(status->theta());
  const double ydot = -linear_angular(0) * std::sin(status->theta());
  const double rel_x = goal.rel_pos().x();
  const double rel_y = goal.rel_pos().y();
  const double squared_norm = rel_x * rel_x + rel_y * rel_y;
  // If squared_norm gets to be too close to zero, just zero out the relevant
  // term to prevent NaNs. Note that this doesn't address the chattering that
  // would likely occur if we were to get excessively close to the target.
  const double atan_diff = (squared_norm < 1e-3)
                               ? 0.0
                               : (rel_x * ydot - rel_y * xdot) / squared_norm;
  // heading = atan2(relative_y, relative_x) - robot_theta
  // dheading / dt = (rel_x * rel_y' - rel_y * rel_x') / (rel_x^2 + rel_y^2) - dtheta / dt
  const double dheading_dt = atan_diff - linear_angular(1);

  goal_.mutable_message()->mutate_unsafe_goal(heading_to_goal);
  goal_.mutable_message()->mutate_goal_velocity(dheading_dt);
}

flatbuffers::Offset<AimerStatus> Aimer::PopulateStatus(
    flatbuffers::FlatBufferBuilder *fbb) const {
  AimerStatus::Builder builder(*fbb);
  builder.add_turret_position(goal_.message().unsafe_goal());
  builder.add_turret_velocity(goal_.message().goal_velocity());
  return builder.Finish();
}

}  // namespace turret
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2020

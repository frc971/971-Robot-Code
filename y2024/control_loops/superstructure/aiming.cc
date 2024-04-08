#include "y2024/control_loops/superstructure/aiming.h"

#include "frc971/control_loops/aiming/aiming.h"
#include "frc971/control_loops/pose.h"

using frc971::control_loops::aiming::RobotState;
using frc971::control_loops::aiming::ShotConfig;
using frc971::control_loops::aiming::ShotMode;
using y2024::control_loops::superstructure::Aimer;

Aimer::Aimer(aos::EventLoop *event_loop,
             const y2024::Constants *robot_constants)
    : event_loop_(event_loop),
      robot_constants_(CHECK_NOTNULL(robot_constants)),
      drivetrain_config_(
          frc971::control_loops::drivetrain::DrivetrainConfig<double>::
              FromFlatbuffer(*robot_constants_->common()->drivetrain())),
      interpolation_table_(
          y2024::constants::Values::InterpolationTableFromFlatbuffer(
              robot_constants_->common()->shooter_interpolation_table())),
      interpolation_table_shuttle_(
          y2024::constants::Values::InterpolationTableFromFlatbuffer(
              robot_constants_->common()
                  ->shooter_shuttle_interpolation_table())),
      note_interpolation_table_(
          y2024::constants::Values::NoteInterpolationTableFromFlatbuffer(
              robot_constants_->common()->note_interpolation_table())),
      joystick_state_fetcher_(
          event_loop_->MakeFetcher<aos::JoystickState>("/aos")) {
  event_loop_->MakeWatcher(
      "/superstructure/rio",
      [this](const y2024::control_loops::superstructure::CANPosition &msg) {
        if (latch_current_) return;

        if (msg.has_retention_roller()) {
          note_current_average_.AddData(
              msg.retention_roller()->torque_current());
        }
      });
}

void Aimer::Update(
    const frc971::control_loops::drivetrain::Status *status,
    frc971::control_loops::aiming::ShotMode shot_mode,
    frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemGoalStatic
        *turret_goal,
    AutoAimMode auto_aim_mode) {
  // Default to aiming at the speaker in the absence of any other targets.
  if (auto_aim_mode == AutoAimMode::NONE) {
    auto_aim_mode = AutoAimMode::SPEAKER;
  }
  if (status == nullptr) {
    return;
  }
  aos::Alliance alliance = aos::Alliance::kRed;
  if (!joystick_state_fetcher_.Fetch() && !received_joystick_state_) {
    received_joystick_state_ = false;
  } else {
    received_joystick_state_ = true;

    CHECK_NOTNULL(joystick_state_fetcher_.get());
    alliance = joystick_state_fetcher_->alliance();
  }

  const bool ignore_localizer_pos =
      auto_aim_mode == AutoAimMode::TURRET_SHUTTLE;
  const Eigen::Vector3d ignore_localizer_position{
      0.0 * (alliance == aos::Alliance::kRed ? 1.0 : -1.0), -1.0, 0.0};
  const frc971::control_loops::Pose robot_pose(
      ignore_localizer_pos ? ignore_localizer_position
                           : Eigen::Vector3d{status->x(), status->y(), 0},
      status->theta());

  frc971::shooter_interpolation::InterpolationTable<
      y2024::constants::Values::ShotParams> *current_interpolation_table =
      interpolation_tables_.at(auto_aim_mode);

  const frc971::control_loops::Pose goal =
      alliance == aos::Alliance::kRed ? red_alliance_goals_.at(auto_aim_mode)
                                      : blue_alliance_goals_.at(auto_aim_mode);

  const Eigen::Vector2d linear_angular =
      drivetrain_config_.Tlr_to_la() *
      Eigen::Vector2d(status->estimated_left_velocity(),
                      status->estimated_right_velocity());
  const double xdot = linear_angular(0) * std::cos(status->theta());
  const double ydot = linear_angular(0) * std::sin(status->theta());

  // Use the previous shot distance to estimate the speed-over-ground of the
  // note.
  current_goal_ = frc971::control_loops::aiming::AimerGoal(
      ShotConfig{goal, shot_mode,
                 frc971::constants::Range::FromFlatbuffer(
                     robot_constants_->common()->turret()->range()),
                 current_interpolation_table->Get(current_goal_.target_distance)
                     .shot_speed_over_ground,
                 /*wrap_mode=*/0.15,
                 // If we don't have any retention roller data, the averager
                 // will return 0. If we get a current that is out-of-range of
                 // the interpolation table, we will use the terminal values of
                 // the interpolation table.
                 M_PI - note_interpolation_table_
                            .Get(note_current_average_.GetAverage()(0))
                            .turret_offset},
      RobotState{
          robot_pose, {xdot, ydot}, linear_angular(1), current_goal_.position});

  turret_goal->set_unsafe_goal(current_goal_.position);
}

flatbuffers::Offset<AimerStatus> Aimer::PopulateStatus(
    flatbuffers::FlatBufferBuilder *fbb) const {
  AimerStatus::Builder builder(*fbb);
  builder.add_turret_position(current_goal_.position);
  builder.add_turret_velocity(current_goal_.velocity);
  builder.add_target_distance(current_goal_.target_distance);
  builder.add_note_current(note_current_average_.GetAverage()(0));
  builder.add_current_turret_offset(
      note_interpolation_table_.Get(note_current_average_.GetAverage()(0))
          .turret_offset);
  return builder.Finish();
}

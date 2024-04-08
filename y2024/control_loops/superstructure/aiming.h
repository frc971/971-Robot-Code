#ifndef Y2024_CONTROL_LOOPS_SUPERSTRUCTURE_AIMING_H_
#define Y2024_CONTROL_LOOPS_SUPERSTRUCTURE_AIMING_H_

#include <map>

#include "frc971/control_loops/aiming/aiming.h"
#include "frc971/control_loops/drivetrain/drivetrain_status_generated.h"
#include "frc971/control_loops/pose.h"
#include "frc971/control_loops/static_zeroing_single_dof_profiled_subsystem.h"
#include "frc971/shooter_interpolation/interpolation.h"
#include "frc971/zeroing/averager.h"
#include "y2024/constants.h"
#include "y2024/constants/constants_generated.h"
#include "y2024/control_loops/drivetrain/drivetrain_base.h"
#include "y2024/control_loops/superstructure/superstructure_can_position_static.h"
#include "y2024/control_loops/superstructure/superstructure_goal_generated.h"
#include "y2024/control_loops/superstructure/superstructure_status_generated.h"
using y2024::control_loops::superstructure::AimerStatus;

namespace y2024::control_loops::superstructure {

class Aimer {
 public:
  Aimer(aos::EventLoop *event_loop, const Constants *robot_constants);

  void Update(
      const frc971::control_loops::drivetrain::Status *status,
      frc971::control_loops::aiming::ShotMode shot_mode,
      frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemGoalStatic
          *turret_goal,
      AutoAimMode auto_aim_mode);

  double DistanceToGoal() const { return current_goal_.virtual_shot_distance; }

  // Indicate that we are ready so that we can reset the note current filter to
  // avoid taking values from when we were still seating the note.
  void IndicateReady() { note_current_average_.Reset(); }

  void latch_note_current(bool latch) { latch_current_ = latch; }

  flatbuffers::Offset<AimerStatus> PopulateStatus(
      flatbuffers::FlatBufferBuilder *fbb) const;

 private:
  aos::EventLoop *event_loop_;

  const Constants *robot_constants_;

  bool latch_current_ = false;

  frc971::control_loops::drivetrain::DrivetrainConfig<double>
      drivetrain_config_;

  frc971::shooter_interpolation::InterpolationTable<
      y2024::constants::Values::ShotParams>
      interpolation_table_;

  frc971::shooter_interpolation::InterpolationTable<
      y2024::constants::Values::ShotParams>
      interpolation_table_shuttle_;

  frc971::shooter_interpolation::InterpolationTable<
      y2024::constants::Values::NoteParams>
      note_interpolation_table_;

  std::map<AutoAimMode, frc971::shooter_interpolation::InterpolationTable<
                            y2024::constants::Values::ShotParams> *>
      interpolation_tables_ = {
          {AutoAimMode::SPEAKER, &interpolation_table_},
          {AutoAimMode::SHUTTLE, &interpolation_table_shuttle_},
          {AutoAimMode::TURRET_SHUTTLE, &interpolation_table_shuttle_}};

  std::map<AutoAimMode, frc971::control_loops::Pose> red_alliance_goals_ = {
      {AutoAimMode::SPEAKER,
       frc971::control_loops::Pose(
           frc971::ToEigenOrDie<3, 1>(*robot_constants_->common()
                                           ->shooter_targets()
                                           ->red_alliance()
                                           ->pos()),
           robot_constants_->common()
               ->shooter_targets()
               ->red_alliance()
               ->theta())},
      {
          AutoAimMode::SHUTTLE,
          frc971::control_loops::Pose(
              frc971::ToEigenOrDie<3, 1>(*robot_constants_->common()
                                              ->shooter_shuttle_targets()
                                              ->red_alliance()
                                              ->pos()),
              robot_constants_->common()
                  ->shooter_shuttle_targets()
                  ->red_alliance()
                  ->theta()),
      },
      {
          AutoAimMode::TURRET_SHUTTLE,
          frc971::control_loops::Pose(
              frc971::ToEigenOrDie<3, 1>(*robot_constants_->common()
                                              ->shooter_shuttle_targets()
                                              ->red_alliance()
                                              ->pos()),
              robot_constants_->common()
                  ->shooter_shuttle_targets()
                  ->red_alliance()
                  ->theta()),
      }};

  std::map<AutoAimMode, frc971::control_loops::Pose> blue_alliance_goals_ = {
      {AutoAimMode::SPEAKER,
       frc971::control_loops::Pose(
           frc971::ToEigenOrDie<3, 1>(*robot_constants_->common()
                                           ->shooter_targets()
                                           ->blue_alliance()
                                           ->pos()),
           robot_constants_->common()
               ->shooter_targets()
               ->blue_alliance()
               ->theta())},
      {
          AutoAimMode::SHUTTLE,
          frc971::control_loops::Pose(
              frc971::ToEigenOrDie<3, 1>(*robot_constants_->common()
                                              ->shooter_shuttle_targets()
                                              ->blue_alliance()
                                              ->pos()),
              robot_constants_->common()
                  ->shooter_shuttle_targets()
                  ->blue_alliance()
                  ->theta()),
      },
      {
          AutoAimMode::TURRET_SHUTTLE,
          frc971::control_loops::Pose(
              frc971::ToEigenOrDie<3, 1>(*robot_constants_->common()
                                              ->shooter_shuttle_targets()
                                              ->blue_alliance()
                                              ->pos()),
              robot_constants_->common()
                  ->shooter_shuttle_targets()
                  ->blue_alliance()
                  ->theta()),
      }};

  aos::Fetcher<aos::JoystickState> joystick_state_fetcher_;

  frc971::zeroing::Averager<double, 50> note_current_average_;

  frc971::control_loops::aiming::TurretGoal current_goal_;

  bool received_joystick_state_ = false;
};

}  // namespace y2024::control_loops::superstructure
#endif  // Y2024_CONTROL_LOOPS_SUPERSTRUCTURE_TURRET_AIMING_H_

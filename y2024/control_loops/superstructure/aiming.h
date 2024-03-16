#ifndef Y2024_CONTROL_LOOPS_SUPERSTRUCTURE_AIMING_H_
#define Y2024_CONTROL_LOOPS_SUPERSTRUCTURE_AIMING_H_

#include "frc971/control_loops/aiming/aiming.h"
#include "frc971/control_loops/drivetrain/drivetrain_status_generated.h"
#include "frc971/control_loops/pose.h"
#include "frc971/control_loops/static_zeroing_single_dof_profiled_subsystem.h"
#include "frc971/shooter_interpolation/interpolation.h"
#include "y2024/constants.h"
#include "y2024/constants/constants_generated.h"
#include "y2024/control_loops/drivetrain/drivetrain_base.h"
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
          *turret_goal);

  double DistanceToGoal() const { return current_goal_.virtual_shot_distance; }

  flatbuffers::Offset<AimerStatus> PopulateStatus(
      flatbuffers::FlatBufferBuilder *fbb) const;

 private:
  aos::EventLoop *event_loop_;

  const Constants *robot_constants_;

  frc971::control_loops::drivetrain::DrivetrainConfig<double>
      drivetrain_config_;

  frc971::shooter_interpolation::InterpolationTable<
      y2024::constants::Values::ShotParams>
      interpolation_table_;

  aos::Fetcher<aos::JoystickState> joystick_state_fetcher_;

  frc971::control_loops::aiming::TurretGoal current_goal_;

  bool received_joystick_state_ = false;
};

}  // namespace y2024::control_loops::superstructure
#endif  // Y2024_CONTROL_LOOPS_SUPERSTRUCTURE_TURRET_AIMING_H_

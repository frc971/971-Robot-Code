#ifndef FRC971_CONTROL_LOOPS_SWERVE_AUTO_ALIGN_H_
#define FRC971_CONTROL_LOOPS_SWERVE_AUTO_ALIGN_H_

#include <math.h>

#include "absl/flags/flag.h"

#include "aos/events/event_loop.h"
#include "frc971/control_loops/swerve/autonomous_init_static.h"
#include "frc971/control_loops/swerve/position_goal_generated.h"
#include "frc971/control_loops/swerve/simplified_dynamics.h"
#include "frc971/control_loops/swerve/swerve_drivetrain_goal_static.h"
#include "frc971/control_loops/swerve/swerve_drivetrain_status_generated.h"
#include "frc971/input/joystick_state_generated.h"
#include "frc971/math/flatbuffers_matrix.h"

namespace frc971::control_loops::swerve {
class AutoAlign {
 public:
  AutoAlign(aos::EventLoop *event_loop);

  void Iterate();

 private:
  aos::Sender<GoalStatic> swerve_goal_sender_;
  aos::Sender<AutonomousInitStatic> autonomous_init_sender_;
  aos::Fetcher<aos::JoystickState> joystick_state_fetcher_;
  aos::Fetcher<frc971::control_loops::swerve::Status>
      swerve_drivetrain_status_fetcher_;
  aos::Fetcher<frc971::control_loops::swerve::PositionGoal>
      position_goal_fetcher_;

  double goal_x_ = 0.0;
  double goal_y_ = 0.0;
  double goal_theta_ = 0.0;
};

}  // namespace frc971::control_loops::swerve

#endif
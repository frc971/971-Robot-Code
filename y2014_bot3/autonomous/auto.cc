#include <stdio.h>

#include <memory>

#include "aos/common/util/phased_loop.h"
#include "aos/common/time.h"
#include "aos/common/util/trapezoid_profile.h"
#include "aos/common/logging/logging.h"
#include "aos/common/logging/queue_logging.h"

#include "y2014_bot3/autonomous/auto.q.h"
#include "y2014_bot3/control_loops/drivetrain/drivetrain.q.h"
#include "y2014_bot3/control_loops/drivetrain/drivetrain.h"
#include "y2014_bot3/control_loops/rollers/rollers.q.h"

using ::aos::time::Time;
using ::y2014_bot3::control_loops::drivetrain_queue;
using ::y2014_bot3::control_loops::rollers_queue;

namespace y2014_bot3 {
namespace autonomous {

namespace time = ::aos::time;

static double left_initial_position, right_initial_position;

bool ShouldExitAuto() {
  ::y2014_bot3::autonomous::autonomous.FetchLatest();
  bool ans = !::y2014_bot3::autonomous::autonomous->run_auto;
  if (ans) {
    LOG(INFO, "Time to exit auto mode\n");
  }
  return ans;
}

void ResetDrivetrain() {
  LOG(INFO, "resetting the drivetrain\n");
  control_loops::drivetrain_queue.goal.MakeWithBuilder()
      .control_loop_driving(false)
      .steering(0.0)
      .throttle(0.0)
      .left_goal(left_initial_position)
      .left_velocity_goal(0)
      .right_goal(right_initial_position)
      .right_velocity_goal(0)
      .Send();
}

void InitializeEncoders() {
  control_loops::drivetrain_queue.status.FetchAnother();
  left_initial_position =
      control_loops::drivetrain_queue.status->filtered_left_position;
  right_initial_position =
      control_loops::drivetrain_queue.status->filtered_right_position;
}

void HandleAuto() {
  ::aos::time::Time start_time = ::aos::time::Time::Now();
  LOG(INFO, "Starting auto mode at %f\n", start_time.ToSeconds());

  // TODO(comran): Add various options for different autos down below.
  ResetDrivetrain();
  InitializeEncoders();

  LOG(INFO, "Driving\n");
  control_loops::drivetrain_queue.goal.MakeWithBuilder()
      .control_loop_driving(false)
      .highgear(false)
      .quickturn(false)
      .steering(0.0)
      .throttle(0.5)
      .left_goal(left_initial_position)
      .left_velocity_goal(0)
      .right_goal(right_initial_position)
      .right_velocity_goal(0)
      .Send();
  time::SleepFor(time::Time::InSeconds(2.0));

  control_loops::drivetrain_queue.goal.MakeWithBuilder()
      .control_loop_driving(false)
      .highgear(false)
      .quickturn(false)
      .steering(0.0)
      .throttle(0.0)
      .left_goal(left_initial_position)
      .left_velocity_goal(0)
      .right_goal(right_initial_position)
      .right_velocity_goal(0)
      .Send();
}

}  // namespace autonomous
}  // namespace y2014_bot3

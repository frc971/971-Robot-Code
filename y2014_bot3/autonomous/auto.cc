#include <stdio.h>

#include <chrono>
#include <memory>

#include "aos/common/logging/logging.h"
#include "aos/common/logging/queue_logging.h"
#include "aos/common/time.h"
#include "aos/common/util/phased_loop.h"
#include "aos/common/util/trapezoid_profile.h"
#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "y2014_bot3/autonomous/auto.q.h"
#include "y2014_bot3/control_loops/rollers/rollers.q.h"

using ::frc971::control_loops::drivetrain_queue;
using ::y2014_bot3::control_loops::rollers_queue;

namespace y2014_bot3 {
namespace autonomous {

namespace chrono = ::std::chrono;
using ::aos::monotonic_clock;

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
  ::frc971::control_loops::drivetrain_queue.goal.MakeWithBuilder()
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
  ::frc971::control_loops::drivetrain_queue.status.FetchAnother();
  left_initial_position =
      ::frc971::control_loops::drivetrain_queue.status->estimated_left_position;
  right_initial_position =
      ::frc971::control_loops::drivetrain_queue.status->estimated_right_position;
}

void HandleAuto() {
  monotonic_clock::time_point start_time = monotonic_clock::now();
  LOG(INFO, "Starting auto mode at %f\n",
      chrono::duration_cast<chrono::duration<double>>(
          start_time.time_since_epoch()).count());

  // TODO(comran): Add various options for different autos down below.
  ResetDrivetrain();
  InitializeEncoders();

  LOG(INFO, "Driving\n");
  ::frc971::control_loops::drivetrain_queue.goal.MakeWithBuilder()
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
  ::std::this_thread::sleep_for(chrono::seconds(2));

  ::frc971::control_loops::drivetrain_queue.goal.MakeWithBuilder()
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

#include <stdio.h>

#include <memory>

#include "aos/common/util/phased_loop.h"
#include "aos/common/time.h"
#include "aos/common/util/trapezoid_profile.h"
#include "aos/common/logging/logging.h"
#include "aos/common/logging/queue_logging.h"

#include "bot3/autonomous/auto.q.h"
#include "bot3/control_loops/drivetrain/drivetrain.q.h"
#include "bot3/control_loops/drivetrain/drivetrain.h"

using ::aos::time::Time;
using ::bot3::control_loops::drivetrain_queue;

namespace bot3 {
namespace autonomous {

struct ProfileParams {
  double velocity;
  double acceleration;
};

namespace time = ::aos::time;

static double left_initial_position, right_initial_position;

bool ShouldExitAuto() {
  ::bot3::autonomous::autonomous.FetchLatest();
  bool ans = !::bot3::autonomous::autonomous->run_auto;
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

void GrabberForTime(double voltage, double wait_time) {
  ::aos::time::Time now = ::aos::time::Time::Now();
  ::aos::time::Time end_time = now + time::Time::InSeconds(wait_time);
  LOG(INFO, "Starting to grab at %f for %f seconds\n", voltage, wait_time);

  while (true) {
    autonomous::can_grabber_control.MakeWithBuilder()
      .can_grabber_voltage(voltage)
      .Send();

    // Poll the running bit and auto done bits.
    if (ShouldExitAuto()) {
      return;
    }
    if (::aos::time::Time::Now() > end_time) {
      LOG(INFO, "Done grabbing\n");
      return;
    }
    ::aos::time::PhasedLoopXMS(5, 2500);
  }
}

// Auto methods.
void CanGrabberAuto() {
  ResetDrivetrain();

  // Launch can grabbers.
  GrabberForTime(12.0, 0.26);
  if (ShouldExitAuto()) return;
  InitializeEncoders();
  ResetDrivetrain();
  if (ShouldExitAuto()) return;

  // Drive backwards, and pulse the can grabbers again to tip the cans.
  control_loops::drivetrain_queue.goal.MakeWithBuilder()
      .control_loop_driving(true)
      .steering(0.0)
      .throttle(0.0)
      .left_goal(left_initial_position + 1.75)
      .left_velocity_goal(0)
      .right_goal(right_initial_position + 1.75)
      .right_velocity_goal(0)
      .Send();
  GrabberForTime(12.0, 0.02);
  if (ShouldExitAuto()) return;

  // We shouldn't need as much power at this point, so lower the can grabber
  // voltages to avoid damaging the motors due to stalling.
  GrabberForTime(4.0, 0.75);
  if (ShouldExitAuto()) return;
  GrabberForTime(-3.0, 0.25);
  if (ShouldExitAuto()) return;
  GrabberForTime(-12.0, 1.0);
  if (ShouldExitAuto()) return;
  GrabberForTime(-3.0, 12.0);
}

void HandleAuto() {
  ::aos::time::Time start_time = ::aos::time::Time::Now();
  LOG(INFO, "Starting auto mode at %f\n", start_time.ToSeconds());

  // TODO(comran): Add various options for different autos down below.
  CanGrabberAuto();
}

}  // namespace autonomous
}  // namespace bot3

#include "y2016/control_loops/superstructure/superstructure.h"
#include "y2016/control_loops/superstructure/superstructure_controls.h"

#include "aos/common/controls/control_loops.q.h"
#include "aos/common/logging/logging.h"

#include "y2016/control_loops/superstructure/integral_intake_plant.h"
#include "y2016/control_loops/superstructure/integral_arm_plant.h"

#include "y2016/constants.h"

namespace y2016 {
namespace control_loops {
namespace superstructure {

namespace {
constexpr double kZeroingVoltage = 4.0;
}  // namespace

Superstructure::Superstructure(
    control_loops::SuperstructureQueue *superstructure_queue)
    : aos::controls::ControlLoop<control_loops::SuperstructureQueue>(
          superstructure_queue) {}

void Superstructure::UpdateZeroingState() {
  // TODO(austin): Explicit state transitions instead of this.
  // TODO(adam): Change this once we have zeroing written.
  if (!arm_.initialized() || !intake_.initialized()) {
    state_ = INITIALIZING;
  } else if (!intake_.zeroed()) {
    state_ = ZEROING_INTAKE;
  } else if (!arm_.zeroed()) {
    state_ = ZEROING_ARM;
  } else {
    state_ = RUNNING;
  }
}

void Superstructure::RunIteration(
    const control_loops::SuperstructureQueue::Goal *unsafe_goal,
    const control_loops::SuperstructureQueue::Position *position,
    control_loops::SuperstructureQueue::Output *output,
    control_loops::SuperstructureQueue::Status *status) {
  if (WasReset()) {
    LOG(ERROR, "WPILib reset, restarting\n");
    arm_.Reset();
    intake_.Reset();
    state_ = UNINITIALIZED;
  }

  // Bool to track if we should turn the motors on or not.
  bool disable = output == nullptr;

  arm_.Correct(position->shoulder, position->wrist);
  intake_.Correct(position->intake);

  // Zeroing will work as follows:
  // Start with the intake. Move it towards the center. Once zeroed, move it
  // back to the bottom. Rotate the shoulder towards the center. Once zeroed,
  // move it up enough to rotate the wrist towards the center.

  // We'll then need code to do sanity checking on values.

  switch (state_) {
    case UNINITIALIZED:
      LOG(DEBUG, "Uninitialized\n");
      state_ = INITIALIZING;
      disable = true;
      break;

    case INITIALIZING:
      LOG(DEBUG, "Waiting for accurate initial position.\n");
      disable = true;
      // Update state_ to accurately represent the state of the zeroing
      // estimators.
      UpdateZeroingState();
      if (state_ != INITIALIZING) {
        // Set the goals to where we are now.
        intake_.ForceGoal(intake_.angle());
        arm_.ForceGoal(arm_.shoulder_angle(), arm_.wrist_angle());
      }
      break;

    case ZEROING_INTAKE:
    case ZEROING_ARM:
      // TODO(adam): Add your magic here.
      state_ = RUNNING;
      break;

    case RUNNING:
      if (unsafe_goal) {
        arm_.AdjustProfile(unsafe_goal->max_angular_velocity_shoulder,
                           unsafe_goal->max_angular_acceleration_shoulder,
                           unsafe_goal->max_angular_velocity_wrist,
                           unsafe_goal->max_angular_acceleration_wrist);
        intake_.AdjustProfile(unsafe_goal->max_angular_velocity_wrist,
                              unsafe_goal->max_angular_acceleration_intake);

        arm_.set_unprofiled_goal(unsafe_goal->angle_shoulder,
                                 unsafe_goal->angle_wrist);
        intake_.set_unprofiled_goal(unsafe_goal->angle_intake);
      }

      // Update state_ to accurately represent the state of the zeroing
      // estimators.

      if (state_ != RUNNING && state_ != ESTOP) {
        state_ = UNINITIALIZED;
      }
      break;

    case ESTOP:
      LOG(ERROR, "Estop\n");
      disable = true;
      break;
  }

  // ESTOP if we hit any of the limits.  It is safe(ish) to hit the limits while
  // zeroing since we use such low power.
  if (state_ == RUNNING) {
    // ESTOP if we hit the hard limits.
    if ((arm_.CheckHardLimits() || intake_.CheckHardLimits()) && output) {
      state_ = ESTOP;
    }
  }

  // Set the voltage limits.
  const double max_voltage = state_ == RUNNING ? 12.0 : kZeroingVoltage;
  arm_.set_max_voltage(max_voltage, max_voltage);
  intake_.set_max_voltage(max_voltage);

  // Calculate the loops for a cycle.
  arm_.Update(disable);
  intake_.Update(disable);

  // Write out all the voltages.
  if (output) {
    output->voltage_intake = intake_.intake_voltage();
    output->voltage_shoulder = arm_.shoulder_voltage();
    output->voltage_wrist = arm_.wrist_voltage();
  }

  // Save debug/internal state.
  // TODO(austin): Save the voltage errors.
  status->zeroed = state_ == RUNNING;

  status->shoulder.angle = arm_.X_hat(0, 0);
  status->shoulder.angular_velocity = arm_.X_hat(1, 0);
  status->shoulder.goal_angle = arm_.goal(0, 0);
  status->shoulder.goal_angular_velocity = arm_.goal(1, 0);
  status->shoulder.unprofiled_goal_angle = arm_.unprofiled_goal(0, 0);
  status->shoulder.unprofiled_goal_angular_velocity =
      arm_.unprofiled_goal(1, 0);
  status->shoulder.estimator_state = arm_.ShoulderEstimatorState();

  status->wrist.angle = arm_.X_hat(2, 0);
  status->wrist.angular_velocity = arm_.X_hat(3, 0);
  status->wrist.goal_angle = arm_.goal(2, 0);
  status->wrist.goal_angular_velocity = arm_.goal(3, 0);
  status->wrist.unprofiled_goal_angle = arm_.unprofiled_goal(2, 0);
  status->wrist.unprofiled_goal_angular_velocity = arm_.unprofiled_goal(3, 0);
  status->wrist.estimator_state = arm_.WristEstimatorState();

  status->intake.angle = intake_.X_hat(0, 0);
  status->intake.angular_velocity = intake_.X_hat(1, 0);
  status->intake.goal_angle = intake_.goal(0, 0);
  status->intake.goal_angular_velocity = intake_.goal(1, 0);
  status->intake.unprofiled_goal_angle = intake_.unprofiled_goal(0, 0);
  status->intake.unprofiled_goal_angular_velocity =
      intake_.unprofiled_goal(1, 0);
  status->intake.estimator_state = intake_.IntakeEstimatorState();

  status->estopped = (state_ == ESTOP);

  status->state = state_;

  last_state_ = state_;
}

}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2016

#include "y2016_bot3/control_loops/intake/intake.h"
#include "y2016_bot3/control_loops/intake/intake_controls.h"

#include "aos/common/commonmath.h"
#include "aos/common/controls/control_loops.q.h"
#include "aos/common/logging/logging.h"

#include "y2016_bot3/control_loops/intake/integral_intake_plant.h"
#include "y2016_bot3/queues/ball_detector.q.h"

namespace y2016_bot3 {
namespace control_loops {
namespace intake {

namespace {
// The maximum voltage the intake roller will be allowed to use.
constexpr float kMaxIntakeTopVoltage = 12.0;
constexpr float kMaxIntakeBottomVoltage = 12.0;
constexpr float kMaxIntakeRollersVoltage = 12.0;
}
// namespace

void LimitChecker::UpdateGoal(double intake_angle_goal) {
  intake_->set_unprofiled_goal(intake_angle_goal);
}

Intake::Intake(control_loops::IntakeQueue *intake_queue)
    : aos::controls::ControlLoop<control_loops::IntakeQueue>(intake_queue),
      limit_checker_(&intake_) {}
bool Intake::IsIntakeNear(double tolerance) {
  return ((intake_.unprofiled_goal() - intake_.X_hat())
              .block<2, 1>(0, 0)
              .lpNorm<Eigen::Infinity>() < tolerance);
}

void Intake::RunIteration(const control_loops::IntakeQueue::Goal *unsafe_goal,
                          const control_loops::IntakeQueue::Position *position,
                          control_loops::IntakeQueue::Output *output,
                          control_loops::IntakeQueue::Status *status) {
  const State state_before_switch = state_;
  if (WasReset()) {
    LOG(ERROR, "WPILib reset, restarting\n");
    intake_.Reset();
    state_ = UNINITIALIZED;
  }

  // Bool to track if we should turn the motors on or not.
  bool disable = output == nullptr;

  intake_.Correct(position->intake);

  // There are 2 main zeroing paths, HIGH_ARM_ZERO and LOW_ARM_ZERO.
  //
  // HIGH_ARM_ZERO works by lifting the arm all the way up so it is clear,
  // moving the shooter to be horizontal, moving the intake out, and then moving
  // the arm back down.
  //
  // LOW_ARM_ZERO works by moving the intake out of the way, lifting the arm up,
  // leveling the shooter, and then moving back down.

  if (intake_.error()) {
    state_ = ESTOP;
  }

  switch (state_) {
    case UNINITIALIZED:
      // Wait in the uninitialized state until intake is initialized.
      LOG(DEBUG, "Uninitialized, waiting for intake\n");
      if (intake_.initialized()) {
        state_ = DISABLED_INITIALIZED;
      }
      disable = true;
      break;

    case DISABLED_INITIALIZED:
      // Wait here until we are either fully zeroed while disabled, or we become
      // enabled.
      if (disable) {
        if (intake_.zeroed()) {
          state_ = SLOW_RUNNING;
        }
      } else {
        if (intake_.angle() <= kIntakeMiddleAngle) {
          state_ = ZERO_LIFT_INTAKE;
        } else {
          state_ = ZERO_LOWER_INTAKE;
        }
      }

      // Set the goals to where we are now so when we start back up, we don't
      // jump.
      intake_.ForceGoal(intake_.angle());
      // Set up the profile to be the zeroing profile.
      intake_.AdjustProfile(0.5, 10);

      // We are not ready to start doing anything yet.
      disable = true;
      break;

    case ZERO_LOWER_INTAKE:
      if (disable) {
        state_ = DISABLED_INITIALIZED;
      } else {
        intake_.set_unprofiled_goal(kIntakeDownAngle);

        if (IsIntakeNear(kLooseTolerance)) {
          // Close enough, start the next move.
          state_ = RUNNING;
        }
      }
      break;

    case ZERO_LIFT_INTAKE:
      if (disable) {
        state_ = DISABLED_INITIALIZED;
      } else {
        intake_.set_unprofiled_goal(kIntakeUpAngle);

        if (IsIntakeNear(kLooseTolerance)) {
          // Close enough, start the next move.
          state_ = RUNNING;
        }
      }
      break;

    // These 4 cases are very similar.
    case SLOW_RUNNING:
    case RUNNING: {
      if (disable) {
        // If we are disabled, go to slow running if we are collided.
        // Reset the profile to the current position so it moves well from here.
        intake_.ForceGoal(intake_.angle());
      }

      double requested_intake = M_PI / 2.0;

      if (unsafe_goal) {
        intake_.AdjustProfile(unsafe_goal->max_angular_velocity_intake,
                              unsafe_goal->max_angular_acceleration_intake);

        requested_intake = unsafe_goal->angle_intake;
      }
      // Push the request out to the hardware.
      limit_checker_.UpdateGoal(requested_intake);

      // ESTOP if we hit the hard limits.
      if (intake_.CheckHardLimits() && output) {
        state_ = ESTOP;
      }
    } break;

    case ESTOP:
      LOG(ERROR, "Estop\n");
      disable = true;
      break;
  }

  // Set the voltage limits.
  const double max_voltage =
      (state_ == RUNNING) ? kOperatingVoltage : kZeroingVoltage;

  intake_.set_max_voltage(max_voltage);

  // Calculate the loops for a cycle.
  {
    Eigen::Matrix<double, 3, 1> error = intake_.controller().error();
    status->intake.position_power = intake_.controller().K(0, 0) * error(0, 0);
    status->intake.velocity_power = intake_.controller().K(0, 1) * error(1, 0);
  }

  intake_.Update(disable);

  // Write out all the voltages.
  if (output) {
    output->voltage_intake = intake_.intake_voltage();

    output->voltage_top_rollers = 0.0;
    output->voltage_bottom_rollers = 0.0;
    output->voltage_intake_rollers = 0.0;

    if (unsafe_goal) {
      // Ball detector lights.
      ::y2016_bot3::sensors::ball_detector.FetchLatest();
      bool ball_detected = false;
      if (::y2016_bot3::sensors::ball_detector.get()) {
        ball_detected = ::y2016_bot3::sensors::ball_detector->voltage > 2.5;
      }

      // Intake.
      if (unsafe_goal->force_intake || !ball_detected) {
        output->voltage_top_rollers = ::std::max(
            -kMaxIntakeTopVoltage,
            ::std::min(unsafe_goal->voltage_top_rollers, kMaxIntakeTopVoltage));
        output->voltage_intake_rollers =
            ::std::max(-kMaxIntakeRollersVoltage,
                       ::std::min(unsafe_goal->voltage_intake_rollers,
                                  kMaxIntakeRollersVoltage));
        output->voltage_bottom_rollers =
            ::std::max(-kMaxIntakeBottomVoltage,
                       ::std::min(unsafe_goal->voltage_bottom_rollers,
                                  kMaxIntakeBottomVoltage));
      } else {
        output->voltage_top_rollers = 0.0;
        output->voltage_bottom_rollers = 0.0;
      }

      // Traverse.
      output->traverse_down = unsafe_goal->traverse_down;
    }
  }

  // Save debug/internal state.
  status->zeroed = intake_.zeroed();

  status->intake.angle = intake_.X_hat(0, 0);
  status->intake.angular_velocity = intake_.X_hat(1, 0);
  status->intake.goal_angle = intake_.goal(0, 0);
  status->intake.goal_angular_velocity = intake_.goal(1, 0);
  status->intake.unprofiled_goal_angle = intake_.unprofiled_goal(0, 0);
  status->intake.unprofiled_goal_angular_velocity =
      intake_.unprofiled_goal(1, 0);
  status->intake.calculated_velocity =
      (intake_.angle() - last_intake_angle_) / 0.005;
  status->intake.voltage_error = intake_.X_hat(2, 0);
  status->intake.estimator_state = intake_.IntakeEstimatorState();
  status->intake.feedforwards_power = intake_.controller().ff_U(0, 0);

  last_intake_angle_ = intake_.angle();

  status->estopped = (state_ == ESTOP);

  status->state = state_;

  last_state_ = state_before_switch;
}

constexpr double Intake::kZeroingVoltage;
constexpr double Intake::kOperatingVoltage;
constexpr double Intake::kLooseTolerance;
constexpr double Intake::kTightTolerance;
constexpr double Intake::kIntakeUpAngle;
constexpr double Intake::kIntakeMiddleAngle;
constexpr double Intake::kIntakeDownAngle;

}  // namespace intake
}  // namespace control_loops
}  // namespace y2016_bot3

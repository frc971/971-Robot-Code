#include "y2016/control_loops/superstructure/superstructure.h"
#include "y2016/control_loops/superstructure/superstructure_controls.h"

#include "aos/common/commonmath.h"
#include "aos/common/controls/control_loops.q.h"
#include "aos/common/logging/logging.h"

#include "y2016/control_loops/superstructure/integral_intake_plant.h"
#include "y2016/control_loops/superstructure/integral_arm_plant.h"
#include "y2016/queues/ball_detector.q.h"

#include "y2016/constants.h"

namespace y2016 {
namespace control_loops {
namespace superstructure {

namespace {
// The maximum voltage the intake roller will be allowed to use.
constexpr float kMaxIntakeTopVoltage = 12.0;
constexpr float kMaxIntakeBottomVoltage = 12.0;
constexpr float kMaxClimberVoltage = 12.0;

// Aliases to reduce typing.
constexpr double kIntakeEncoderIndexDifference =
    constants::Values::kIntakeEncoderIndexDifference;
constexpr double kWristEncoderIndexDifference =
    constants::Values::kWristEncoderIndexDifference;
constexpr double kShoulderEncoderIndexDifference =
    constants::Values::kShoulderEncoderIndexDifference;
}  // namespace

// ///// CollisionAvoidance /////

void CollisionAvoidance::UpdateGoal(double shoulder_angle_goal,
                                    double wrist_angle_goal,
                                    double intake_angle_goal) {
  const double original_shoulder_angle_goal = shoulder_angle_goal;
  const double original_intake_angle_goal = intake_angle_goal;
  const double original_wrist_angle_goal = wrist_angle_goal;

  double shoulder_angle = arm_->shoulder_angle();
  double wrist_angle = arm_->wrist_angle();
  double intake_angle = intake_->position();

  // TODO(phil): This may need tuning to account for bounciness in the limbs or
  // some other thing that I haven't thought of. At the very least,
  // incorporating a small safety margin makes writing test cases much easier
  // since you can directly compare statuses against the constants in the
  // CollisionAvoidance class.
  constexpr double kSafetyMargin = 0.03;  // radians

  // Avoid colliding the shooter with the frame.
  // If the shoulder is below a certain angle or we want to move it below
  // that angle, then the shooter has to stay level to the ground. Otherwise,
  // it will crash into the frame.
  if (intake_angle < kMaxIntakeAngleBeforeArmInterference + kSafetyMargin) {
    if (shoulder_angle < kMinShoulderAngleForHorizontalShooter ||
        original_shoulder_angle_goal < kMinShoulderAngleForHorizontalShooter) {
      wrist_angle_goal = 0.0;
    } else if (shoulder_angle < kMinShoulderAngleForIntakeInterference ||
               original_shoulder_angle_goal <
                   kMinShoulderAngleForIntakeInterference) {
      wrist_angle_goal =
          aos::Clip(original_wrist_angle_goal,
                    kMinWristAngleForMovingByIntake + kSafetyMargin,
                    kMaxWristAngleForMovingByIntake - kSafetyMargin);
    }
  } else {
    if (shoulder_angle < kMinShoulderAngleForIntakeUpInterference ||
        original_shoulder_angle_goal <
            kMinShoulderAngleForIntakeUpInterference) {
      wrist_angle_goal = 0.0;
    }
  }

  if (shoulder_angle < kMinShoulderAngleForIntakeUpInterference ||
      original_shoulder_angle_goal < kMinShoulderAngleForIntakeUpInterference) {
    // Make sure that we don't move the shoulder below a certain angle until
    // the wrist is level with the ground.
    if (intake_angle < kMaxIntakeAngleBeforeArmInterference + kSafetyMargin) {
      if (wrist_angle > kMaxWristAngleForMovingByIntake ||
          wrist_angle < kMinWristAngleForMovingByIntake) {
        shoulder_angle_goal =
            ::std::max(original_shoulder_angle_goal,
                       kMinShoulderAngleForIntakeInterference + kSafetyMargin);
      }
    } else {
      if (wrist_angle > kMaxWristAngleForMovingByIntake ||
          wrist_angle < kMinWristAngleForMovingByIntake) {
        shoulder_angle_goal = ::std::max(
            original_shoulder_angle_goal,
            kMinShoulderAngleForIntakeUpInterference + kSafetyMargin);
      }
    }
    if (::std::abs(wrist_angle) > kMaxWristAngleForSafeArmStowing) {
      shoulder_angle_goal =
          ::std::max(shoulder_angle_goal,
                     kMinShoulderAngleForHorizontalShooter + kSafetyMargin);
    }
  }

  // Is the arm where it could interfere with the intake right now?
  bool shoulder_is_in_danger =
      (shoulder_angle < kMinShoulderAngleForIntakeUpInterference &&
       shoulder_angle > kMaxShoulderAngleUntilSafeIntakeStowing);

  // Is the arm moving into collision zone from above?
  bool shoulder_moving_into_danger_from_above =
      (shoulder_angle >= kMinShoulderAngleForIntakeUpInterference &&
       original_shoulder_angle_goal <=
           kMinShoulderAngleForIntakeUpInterference);

  // Is the arm moving into collision zone from below?
  bool shoulder_moving_into_danger_from_below =
      (shoulder_angle <= kMaxShoulderAngleUntilSafeIntakeStowing &&
       original_shoulder_angle_goal >= kMaxShoulderAngleUntilSafeIntakeStowing);

  // Avoid colliding the arm with the intake.
  if (shoulder_is_in_danger || shoulder_moving_into_danger_from_above ||
      shoulder_moving_into_danger_from_below) {
    // If the arm could collide with the intake, we make sure to move the
    // intake out of the way. The arm has priority.
    intake_angle_goal =
        ::std::min(original_intake_angle_goal,
                   kMaxIntakeAngleBeforeArmInterference - kSafetyMargin);

    // Don't let the shoulder move into the collision area until the intake is
    // out of the way.
    if (intake_angle > kMaxIntakeAngleBeforeArmInterference) {
      const double kHalfwayPointBetweenSafeZones =
          (kMinShoulderAngleForIntakeInterference +
           kMaxShoulderAngleUntilSafeIntakeStowing) /
          2.0;

      if (shoulder_angle >= kHalfwayPointBetweenSafeZones) {
        // The shoulder is closer to being above the collision area. Move it up
        // there.
        if (intake_angle <
            kMaxIntakeAngleBeforeArmInterference + kSafetyMargin) {
          shoulder_angle_goal = ::std::max(
              original_shoulder_angle_goal,
              kMinShoulderAngleForIntakeInterference + kSafetyMargin);
        } else {
          shoulder_angle_goal = ::std::max(
              original_shoulder_angle_goal,
              kMinShoulderAngleForIntakeUpInterference + kSafetyMargin);
        }
      } else {
        // The shoulder is closer to being below the collision zone (i.e. in
        // stowing/intake position), keep it there for now.
        shoulder_angle_goal =
            ::std::min(original_shoulder_angle_goal,
                       kMaxShoulderAngleUntilSafeIntakeStowing - kSafetyMargin);
      }
    }
  }

  // Send the possibly adjusted goals to the components.
  arm_->set_unprofiled_goal(shoulder_angle_goal, wrist_angle_goal);
  intake_->set_unprofiled_goal(intake_angle_goal);
}

bool CollisionAvoidance::collided() const {
  return collided_with_given_angles(arm_->shoulder_angle(), arm_->wrist_angle(),
                                    intake_->position());
}

bool CollisionAvoidance::collided_with_given_angles(double shoulder_angle,
                                                    double wrist_angle,
                                                    double intake_angle) {
  // The arm and the intake must not hit.
  if (shoulder_angle >=
          CollisionAvoidance::kMaxShoulderAngleUntilSafeIntakeStowing &&
      shoulder_angle <=
          CollisionAvoidance::kMinShoulderAngleForIntakeUpInterference &&
      intake_angle > CollisionAvoidance::kMaxIntakeAngleBeforeArmInterference) {
    LOG(DEBUG, "Collided: Intake %f > %f, and shoulder %f < %f < %f.\n",
        intake_angle, CollisionAvoidance::kMaxIntakeAngleBeforeArmInterference,
        CollisionAvoidance::kMaxShoulderAngleUntilSafeIntakeStowing,
        shoulder_angle,
        CollisionAvoidance::kMinShoulderAngleForIntakeUpInterference);
    return true;
  }

  if (shoulder_angle >=
          CollisionAvoidance::kMaxShoulderAngleUntilSafeIntakeStowing &&
      shoulder_angle <=
          CollisionAvoidance::kMinShoulderAngleForIntakeInterference &&
      intake_angle < CollisionAvoidance::kMaxIntakeAngleBeforeArmInterference &&
      intake_angle > Superstructure::kIntakeLowerClear &&
      (wrist_angle > CollisionAvoidance::kMaxWristAngleForMovingByIntake ||
       wrist_angle < CollisionAvoidance::kMinWristAngleForMovingByIntake)) {
    LOG(DEBUG,
        "Collided: Intake %f < %f < %f, shoulder %f < %f < %f, and %f < %f < "
        "%f.\n",
        Superstructure::kIntakeLowerClear, intake_angle,
        CollisionAvoidance::kMaxIntakeAngleBeforeArmInterference,
        CollisionAvoidance::kMaxShoulderAngleUntilSafeIntakeStowing,
        shoulder_angle,
        CollisionAvoidance::kMinShoulderAngleForIntakeInterference,
        CollisionAvoidance::kMinWristAngleForMovingByIntake, wrist_angle,
        CollisionAvoidance::kMaxWristAngleForMovingByIntake);
    return true;
  }

  // The wrist must go back to zero when the shoulder is moving the arm into
  // a stowed/intaking position.
  if (shoulder_angle <
          CollisionAvoidance::kMinShoulderAngleForHorizontalShooter &&
      ::std::abs(wrist_angle) > kMaxWristAngleForSafeArmStowing) {
    LOG(DEBUG, "Collided: Shoulder %f < %f and wrist |%f| > %f.\n",
        shoulder_angle,
        CollisionAvoidance::kMinShoulderAngleForHorizontalShooter, wrist_angle,
        kMaxWristAngleForSafeArmStowing);
    return true;
  }

  return false;
}

constexpr double CollisionAvoidance::kMinShoulderAngleForHorizontalShooter;
constexpr double CollisionAvoidance::kMinShoulderAngleForIntakeInterference;
constexpr double CollisionAvoidance::kMaxIntakeAngleBeforeArmInterference;
constexpr double CollisionAvoidance::kMaxWristAngleForSafeArmStowing;
constexpr double CollisionAvoidance::kMaxShoulderAngleUntilSafeIntakeStowing;

Superstructure::Superstructure(
    control_loops::SuperstructureQueue *superstructure_queue)
    : aos::controls::ControlLoop<control_loops::SuperstructureQueue>(
          superstructure_queue),
      collision_avoidance_(&intake_, &arm_) {}

bool Superstructure::IsArmNear(double shoulder_tolerance,
                               double wrist_tolerance) {
  return ((arm_.unprofiled_goal() - arm_.X_hat())
              .block<2, 1>(0, 0)
              .lpNorm<Eigen::Infinity>() < shoulder_tolerance) &&
         ((arm_.unprofiled_goal() - arm_.X_hat())
              .block<4, 1>(0, 0)
              .lpNorm<Eigen::Infinity>() < wrist_tolerance) &&
         ((arm_.unprofiled_goal() - arm_.goal())
              .block<4, 1>(0, 0)
              .lpNorm<Eigen::Infinity>() < 1e-6);
}

bool Superstructure::IsArmNear(double tolerance) {
  return ((arm_.unprofiled_goal() - arm_.X_hat())
              .block<4, 1>(0, 0)
              .lpNorm<Eigen::Infinity>() < tolerance) &&
         ((arm_.unprofiled_goal() - arm_.goal())
              .block<4, 1>(0, 0)
              .lpNorm<Eigen::Infinity>() < 1e-6);
}

bool Superstructure::IsIntakeNear(double tolerance) {
  return ((intake_.unprofiled_goal() - intake_.X_hat())
              .block<2, 1>(0, 0)
              .lpNorm<Eigen::Infinity>() < tolerance);
}

double Superstructure::MoveButKeepAbove(double reference_angle,
                                        double current_angle,
                                        double move_distance) {
  return -MoveButKeepBelow(-reference_angle, -current_angle, -move_distance);
}

double Superstructure::MoveButKeepBelow(double reference_angle,
                                        double current_angle,
                                        double move_distance) {
  // There are 3 interesting places to move to.
  const double small_negative_move = current_angle - move_distance;
  const double small_positive_move = current_angle + move_distance;
  // And the reference angle.

  // Move the the highest one that is below reference_angle.
  if (small_negative_move > reference_angle) {
    return reference_angle;
  } else if (small_positive_move > reference_angle) {
    return small_negative_move;
  } else {
    return small_positive_move;
  }
}

void Superstructure::RunIteration(
    const control_loops::SuperstructureQueue::Goal *unsafe_goal,
    const control_loops::SuperstructureQueue::Position *position,
    control_loops::SuperstructureQueue::Output *output,
    control_loops::SuperstructureQueue::Status *status) {
  const State state_before_switch = state_;
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

  // There are 2 main zeroing paths, HIGH_ARM_ZERO and LOW_ARM_ZERO.
  //
  // HIGH_ARM_ZERO works by lifting the arm all the way up so it is clear,
  // moving the shooter to be horizontal, moving the intake out, and then moving
  // the arm back down.
  //
  // LOW_ARM_ZERO works by moving the intake out of the way, lifting the arm up,
  // leveling the shooter, and then moving back down.

  if (arm_.error() || intake_.error()) {
    state_ = ESTOP;
  }

  const bool is_collided = collided();
  switch (state_) {
    case UNINITIALIZED:
      // Wait in the uninitialized state until both the arm and intake are
      // initialized.
      LOG(DEBUG, "Uninitialized, waiting for intake and arm\n");
      if (arm_.initialized() && intake_.initialized()) {
        state_ = DISABLED_INITIALIZED;
      }
      disable = true;
      break;

    case DISABLED_INITIALIZED:
      // Wait here until we are either fully zeroed while disabled, or we become
      // enabled.  At that point, figure out if we should HIGH_ARM_ZERO or
      // LOW_ARM_ZERO.
      if (disable) {
        if (arm_.zeroed() && intake_.zeroed()) {
          state_ = SLOW_RUNNING;
        }
      } else {
        if (arm_.shoulder_angle() >= kShoulderMiddleAngle) {
          state_ = HIGH_ARM_ZERO_LIFT_ARM;
        } else {
          state_ = LOW_ARM_ZERO_LOWER_INTAKE;
        }
      }

      // Set the goals to where we are now so when we start back up, we don't
      // jump.
      intake_.ForceGoal(intake_.position());
      arm_.ForceGoal(arm_.shoulder_angle(), arm_.wrist_angle());
      // Set up the profile to be the zeroing profile.
      intake_.AdjustProfile(0.5, 10);
      arm_.AdjustProfile(0.5, 10, 0.5, 10);

      // We are not ready to start doing anything yet.
      disable = true;
      break;

    case HIGH_ARM_ZERO_LIFT_ARM:
      if (disable) {
        state_ = DISABLED_INITIALIZED;
      } else {
        // Raise the shoulder up out of the way.
        arm_.set_unprofiled_goal(kShoulderUpAngle, arm_.wrist_angle());
        if (IsArmNear(kLooseTolerance)) {
          // Close enough, start the next move.
          state_ = HIGH_ARM_ZERO_LEVEL_SHOOTER;
        }
      }
      break;

    case HIGH_ARM_ZERO_LEVEL_SHOOTER:
      if (disable) {
        state_ = DISABLED_INITIALIZED;
      } else {
        // Move the shooter to be level.
        arm_.set_unprofiled_goal(kShoulderUpAngle, 0.0);

        if (IsArmNear(kLooseTolerance)) {
          // Close enough, start the next move.
          state_ = HIGH_ARM_ZERO_MOVE_INTAKE_OUT;
        }
      }
      break;

    case HIGH_ARM_ZERO_MOVE_INTAKE_OUT:
      if (disable) {
        state_ = DISABLED_INITIALIZED;
      } else {
        // If we were just asked to move the intake, make sure it moves far
        // enough.
        if (last_state_ != HIGH_ARM_ZERO_MOVE_INTAKE_OUT) {
          intake_.set_unprofiled_goal(
              MoveButKeepBelow(kIntakeUpperClear, intake_.position(),
                               kIntakeEncoderIndexDifference * 2.5));
        }

        if (IsIntakeNear(kLooseTolerance)) {
          // Close enough, start the next move.
          state_ = HIGH_ARM_ZERO_LOWER_ARM;
        }
      }
      break;

    case HIGH_ARM_ZERO_LOWER_ARM:
      if (disable) {
        state_ = DISABLED_INITIALIZED;
      } else {
        // Land the shooter in the belly-pan.  It should be zeroed by the time
        // it gets there.  If not, just estop.
        arm_.set_unprofiled_goal(kShoulderLanded, 0.0);
        if (arm_.zeroed() && intake_.zeroed()) {
          state_ = RUNNING;
        } else if (IsArmNear(kLooseTolerance)) {
          LOG(ERROR,
              "Failed to zero while executing the HIGH_ARM_ZERO sequence. "
              "Arm: %d Intake %d\n",
              arm_.zeroed(), intake_.zeroed());
          state_ = ESTOP;
        }
      }
      break;

    case LOW_ARM_ZERO_LOWER_INTAKE:
      if (disable) {
        state_ = DISABLED_INITIALIZED;
      } else {
        // Move the intake down out of the way of the arm.  Make sure to move it
        // far enough to zero.
        if (last_state_ != LOW_ARM_ZERO_LOWER_INTAKE) {
          intake_.set_unprofiled_goal(
              MoveButKeepBelow(kIntakeLowerClear, intake_.position(),
                               kIntakeEncoderIndexDifference * 2.5));
        }
        if (IsIntakeNear(kLooseTolerance)) {
          if (::std::abs(arm_.wrist_angle()) < kWristAlmostLevel) {
            state_ = LOW_ARM_ZERO_MAYBE_LEVEL_SHOOTER;
          } else {
            state_ = LOW_ARM_ZERO_LIFT_SHOULDER;
          }
        }
      }
      break;

    case LOW_ARM_ZERO_MAYBE_LEVEL_SHOOTER:
      if (disable) {
        state_ = DISABLED_INITIALIZED;
      } else {
        // If we are supposed to level the shooter, set it to level, and wait
        // until it is very close to level.
        arm_.set_unprofiled_goal(arm_.unprofiled_goal(0, 0), 0.0);
        if (IsArmNear(kLooseTolerance, kTightTolerance)) {
          state_ = LOW_ARM_ZERO_LIFT_SHOULDER;
        }
      }
      break;

    case LOW_ARM_ZERO_LIFT_SHOULDER:
      if (disable) {
        state_ = DISABLED_INITIALIZED;
      } else {
        // Decide where to move to.  We need to move far enough to see an index
        // pulse, but must also get high enough that we can safely level the
        // shooter.
        if (last_state_ != LOW_ARM_ZERO_LIFT_SHOULDER) {
          arm_.set_unprofiled_goal(
              MoveButKeepAbove(kShoulderWristClearAngle, arm_.shoulder_angle(),
                               ::std::max(kWristEncoderIndexDifference,
                                          kShoulderEncoderIndexDifference) *
                                   2.5),
              arm_.unprofiled_goal(2, 0));
        }

        // If we're about to ask the wrist to go past one of its limits, then
        // move the goal so it will be just at the limit when we finish lifting
        // the shoulder. If it wasn't intersecting something before, this can't
        // cause it to crash into anything.
        const double ungrounded_wrist = arm_.goal(2, 0) - arm_.goal(0, 0);
        const double unprofiled_ungrounded_wrist =
            arm_.unprofiled_goal(2, 0) - arm_.unprofiled_goal(0, 0);
        if (unprofiled_ungrounded_wrist >
                constants::Values::kWristRange.upper &&
            ungrounded_wrist >
                constants::Values::kWristRange.upper - kWristAlmostLevel) {
          arm_.set_unprofiled_goal(arm_.unprofiled_goal(0, 0),
                                   constants::Values::kWristRange.upper +
                                       arm_.unprofiled_goal(0, 0));
        } else if (unprofiled_ungrounded_wrist <
                       constants::Values::kWristRange.lower &&
                   ungrounded_wrist < constants::Values::kWristRange.lower +
                                          kWristAlmostLevel) {
          arm_.set_unprofiled_goal(arm_.unprofiled_goal(0, 0),
                                   constants::Values::kWristRange.lower +
                                       arm_.unprofiled_goal(0, 0));
        }

        // Wait until we are level and then go for it.
        if (IsArmNear(kLooseTolerance)) {
          state_ = LOW_ARM_ZERO_LEVEL_SHOOTER;
        }
      }
      break;

    case LOW_ARM_ZERO_LEVEL_SHOOTER:
      if (disable) {
        state_ = DISABLED_INITIALIZED;
      } else {
        // Move the shooter level (and keep the same height).  We don't want to
        // got to RUNNING until we are completely level so that we don't
        // give control back in a weird case where we might crash.
        arm_.set_unprofiled_goal(arm_.unprofiled_goal(0, 0), 0.0);
        if (IsArmNear(kLooseTolerance)) {
          if (arm_.zeroed() && intake_.zeroed()) {
            state_ = RUNNING;
          } else {
            LOG(ERROR,
                "Failed to zero while executing the LOW_ARM_ZERO sequence. "
                "Arm: %d Intake %d\n",
                arm_.zeroed(), intake_.zeroed());
            state_ = ESTOP;
          }
        }
      }
      break;

    // These 4 cases are very similar.
    case SLOW_RUNNING:
    case RUNNING:
    case LANDING_SLOW_RUNNING:
    case LANDING_RUNNING: {
      if (disable) {
        // If we are disabled, go to slow running (or landing slow running) if
        // we are collided.
        if (is_collided) {
          if (state_ == RUNNING) {
            state_ = SLOW_RUNNING;
          } else if (state_ == LANDING_RUNNING) {
            state_ = LANDING_SLOW_RUNNING;
          }
        }

        // Reset the profile to the current position so it moves well from here.
        intake_.ForceGoal(intake_.position());
        arm_.ForceGoal(arm_.shoulder_angle(), arm_.wrist_angle());
      } else {
        // If we are in slow_running and are no longer collided, let 'er rip.
        if (state_ == SLOW_RUNNING) {
          if (!is_collided) {
            state_ = RUNNING;
          }
        } else if (state_ == LANDING_SLOW_RUNNING) {
          if (!is_collided) {
            state_ = LANDING_RUNNING;
          }
        }
      }

      double requested_shoulder = constants::Values::kShoulderRange.lower;
      double requested_wrist = 0.0;
      double requested_intake = M_PI / 2.0;

      if (unsafe_goal) {
        // If we are in one of the landing states, limit the accelerations and
        // velocities to land cleanly.
        if (state_ == LANDING_SLOW_RUNNING || state_ == LANDING_RUNNING) {
          arm_.AdjustProfile(0.5,    // Shoulder Velocity
                             4.0,    // Shoulder acceleration,
                             4.0,    // Wrist velocity
                             10.0);  // Wrist acceleration.
          intake_.AdjustProfile(unsafe_goal->max_angular_velocity_intake,
                                unsafe_goal->max_angular_acceleration_intake);

          requested_shoulder =
              ::std::max(unsafe_goal->angle_shoulder,
                         constants::Values::kShoulderRange.lower);
          requested_wrist = 0.0;
          requested_intake = unsafe_goal->angle_intake;
          // Transition to landing once the profile is close to finished for the
          // shoulder.
          if (arm_.goal(0, 0) > kShoulderTransitionToLanded + 1e-4 ||
              arm_.unprofiled_goal(0, 0) > kShoulderTransitionToLanded + 1e-4) {
            if (state_ == LANDING_RUNNING) {
              state_ = RUNNING;
            } else {
              state_ = SLOW_RUNNING;
            }
          }
        } else {
          // Otherwise, give the user what he asked for.
          arm_.AdjustProfile(unsafe_goal->max_angular_velocity_shoulder,
                             unsafe_goal->max_angular_acceleration_shoulder,
                             unsafe_goal->max_angular_velocity_wrist,
                             unsafe_goal->max_angular_acceleration_wrist);
          intake_.AdjustProfile(unsafe_goal->max_angular_velocity_intake,
                                unsafe_goal->max_angular_acceleration_intake);

          // Except, don't let the shoulder go all the way down.
          requested_shoulder = ::std::max(unsafe_goal->angle_shoulder,
                                          kShoulderTransitionToLanded);
          requested_wrist = unsafe_goal->angle_wrist;
          requested_intake = unsafe_goal->angle_intake;

          // Transition to landing once the profile is close to finished for the
          // shoulder.
          if (arm_.goal(0, 0) <= kShoulderTransitionToLanded + 1e-4 &&
              arm_.unprofiled_goal(0, 0) <=
                  kShoulderTransitionToLanded + 1e-4) {
            if (state_ == RUNNING) {
              state_ = LANDING_RUNNING;
            } else {
              state_ = LANDING_SLOW_RUNNING;
            }
          }
        }
      }

      // Push the request out to hardware!
      collision_avoidance_.UpdateGoal(requested_shoulder, requested_wrist,
                                      requested_intake);

      // ESTOP if we hit the hard limits.
      if ((arm_.CheckHardLimits() || intake_.CheckHardLimits()) && output) {
        state_ = ESTOP;
      }
    } break;

    case ESTOP:
      LOG(ERROR, "Estop\n");
      disable = true;
      break;
  }

  // Set the voltage limits.
  const double max_voltage = (state_ == RUNNING || state_ == LANDING_RUNNING)
                                 ? kOperatingVoltage
                                 : kZeroingVoltage;
  if (unsafe_goal) {
    constexpr float kTriggerThreshold = 12.0 * 0.90 / 0.005;

    if (unsafe_goal->voltage_climber > 1.0) {
      kill_shoulder_accumulator_ +=
          ::std::min(kMaxClimberVoltage, unsafe_goal->voltage_climber);
    } else {
      kill_shoulder_accumulator_ = 0.0;
    }

    if (kill_shoulder_accumulator_ > kTriggerThreshold) {
      kill_shoulder_ = true;
    }
  }
  arm_.set_max_voltage(
      {{kill_shoulder_ ? 0.0 : max_voltage,
        kill_shoulder_ ? (arm_.X_hat(0, 0) < 0.05 ? kShooterHangingLowVoltage
                                                  : kShooterHangingVoltage)
                       : max_voltage}});
  intake_.set_max_voltage({{max_voltage}});

  if (IsRunning() && !kill_shoulder_) {
    // We don't want lots of negative voltage when we are near the bellypan on
    // the shoulder...
    // TODO(austin): Do I want to push negative power into the belly pan at this
    // point?  Maybe just put the goal slightly below the bellypan and call that
    // good enough.
    if (arm_.goal(0, 0) <= kShoulderTransitionToLanded + 1e-4 ||
        arm_.X_hat(0, 0) <= kShoulderTransitionToLanded + 1e-4) {
      arm_.set_shoulder_asymetric_limits(kLandingShoulderDownVoltage,
                                         max_voltage);
    }
  }

  // Calculate the loops for a cycle.
  {
    Eigen::Matrix<double, 3, 1> error = intake_.controller().error();
    status->intake.position_power =
        intake_.controller().controller().K(0, 0) * error(0, 0);
    status->intake.velocity_power =
        intake_.controller().controller().K(0, 1) * error(1, 0);
  }

  {
    Eigen::Matrix<double, 6, 1> error = arm_.controller().error();
    status->shoulder.position_power =
        arm_.controller().controller().K(0, 0) * error(0, 0);
    status->shoulder.velocity_power =
        arm_.controller().controller().K(0, 1) * error(1, 0);
    status->wrist.position_power =
        arm_.controller().controller().K(0, 2) * error(2, 0);
    status->wrist.velocity_power =
        arm_.controller().controller().K(0, 3) * error(3, 0);
  }

  arm_.Update(disable);
  intake_.Update(disable);

  // Write out all the voltages.
  if (output) {
    output->voltage_intake = intake_.voltage();
    output->voltage_shoulder = arm_.shoulder_voltage();
    output->voltage_wrist = arm_.wrist_voltage();

    output->voltage_top_rollers = 0.0;
    output->voltage_bottom_rollers = 0.0;

    output->voltage_climber = 0.0;
    output->unfold_climber = false;
    if (unsafe_goal) {
      // Ball detector lights.
      ::y2016::sensors::ball_detector.FetchLatest();
      bool ball_detected = false;
      if (::y2016::sensors::ball_detector.get()) {
        ball_detected = ::y2016::sensors::ball_detector->voltage > 2.5;
      }

      // Climber.
      output->voltage_climber = ::std::max(
          static_cast<float>(0.0),
          ::std::min(unsafe_goal->voltage_climber, kMaxClimberVoltage));
      output->unfold_climber = unsafe_goal->unfold_climber;

      // Intake.
      if (unsafe_goal->force_intake || !ball_detected) {
        output->voltage_top_rollers = ::std::max(
            -kMaxIntakeTopVoltage,
            ::std::min(unsafe_goal->voltage_top_rollers, kMaxIntakeTopVoltage));
        output->voltage_bottom_rollers =
            ::std::max(-kMaxIntakeBottomVoltage,
                       ::std::min(unsafe_goal->voltage_bottom_rollers,
                                  kMaxIntakeBottomVoltage));
      } else {
        output->voltage_top_rollers = 0.0;
        output->voltage_bottom_rollers = 0.0;
      }

      // Traverse.
      output->traverse_unlatched = unsafe_goal->traverse_unlatched;
      output->traverse_down = unsafe_goal->traverse_down;
    }
  }

  // Save debug/internal state.
  status->zeroed = arm_.zeroed() && intake_.zeroed();

  status->shoulder.angle = arm_.X_hat(0, 0);
  status->shoulder.angular_velocity = arm_.X_hat(1, 0);
  status->shoulder.goal_angle = arm_.goal(0, 0);
  status->shoulder.goal_angular_velocity = arm_.goal(1, 0);
  status->shoulder.unprofiled_goal_angle = arm_.unprofiled_goal(0, 0);
  status->shoulder.unprofiled_goal_angular_velocity =
      arm_.unprofiled_goal(1, 0);
  status->shoulder.voltage_error = arm_.X_hat(4, 0);
  status->shoulder.calculated_velocity =
      (arm_.shoulder_angle() - last_shoulder_angle_) / 0.005;
  status->shoulder.estimator_state = arm_.EstimatorState(0);

  status->wrist.angle = arm_.X_hat(2, 0);
  status->wrist.angular_velocity = arm_.X_hat(3, 0);
  status->wrist.goal_angle = arm_.goal(2, 0);
  status->wrist.goal_angular_velocity = arm_.goal(3, 0);
  status->wrist.unprofiled_goal_angle = arm_.unprofiled_goal(2, 0);
  status->wrist.unprofiled_goal_angular_velocity = arm_.unprofiled_goal(3, 0);
  status->wrist.voltage_error = arm_.X_hat(5, 0);
  status->wrist.calculated_velocity =
      (arm_.wrist_angle() - last_wrist_angle_) / 0.005;
  status->wrist.estimator_state = arm_.EstimatorState(1);

  status->intake.angle = intake_.X_hat(0, 0);
  status->intake.angular_velocity = intake_.X_hat(1, 0);
  status->intake.goal_angle = intake_.goal(0, 0);
  status->intake.goal_angular_velocity = intake_.goal(1, 0);
  status->intake.unprofiled_goal_angle = intake_.unprofiled_goal(0, 0);
  status->intake.unprofiled_goal_angular_velocity =
      intake_.unprofiled_goal(1, 0);
  status->intake.calculated_velocity =
      (intake_.position() - last_intake_angle_) / 0.005;
  status->intake.voltage_error = intake_.X_hat(2, 0);
  status->intake.estimator_state = intake_.EstimatorState(0);
  status->intake.feedforwards_power = intake_.controller().ff_U(0, 0);

  status->shoulder_controller_index = arm_.controller_index();

  last_shoulder_angle_ = arm_.shoulder_angle();
  last_wrist_angle_ = arm_.wrist_angle();
  last_intake_angle_ = intake_.position();

  status->estopped = (state_ == ESTOP);

  status->state = state_;
  status->is_collided = is_collided;

  last_state_ = state_before_switch;
}

constexpr double Superstructure::kZeroingVoltage;
constexpr double Superstructure::kOperatingVoltage;
constexpr double Superstructure::kLandingShoulderDownVoltage;
constexpr double Superstructure::kShoulderMiddleAngle;
constexpr double Superstructure::kLooseTolerance;
constexpr double Superstructure::kIntakeUpperClear;
constexpr double Superstructure::kIntakeLowerClear;
constexpr double Superstructure::kShoulderUpAngle;
constexpr double Superstructure::kShoulderLanded;
constexpr double Superstructure::kTightTolerance;
constexpr double Superstructure::kWristAlmostLevel;
constexpr double Superstructure::kShoulderWristClearAngle;
constexpr double Superstructure::kShoulderTransitionToLanded;

}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2016
